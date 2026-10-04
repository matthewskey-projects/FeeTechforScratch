// SO-100 Arm extension for TurboWarp.
// Talks to Feetech STS3215 bus servos through a Feetech URT-1 (or any USB-TTL
// bus servo adapter) using the Web Serial API. Must be loaded unsandboxed.
//
// Register addresses follow the STS/SMS control table used by LeRobot
// (lerobot/motors/feetech/tables.py).

(function (Scratch) {
  'use strict';

  // ---------------------------------------------------------------------------
  // Feetech STS protocol
  // ---------------------------------------------------------------------------

  const INST = {
    PING: 0x01,
    READ: 0x02,
    WRITE: 0x03,
    SYNC_READ: 0x82,
    SYNC_WRITE: 0x83
  };
  const BROADCAST_ID = 0xfe;

  const REG = {
    MIN_POSITION_LIMIT: 9, // 2 bytes, EEPROM
    MAX_POSITION_LIMIT: 11, // 2 bytes, EEPROM
    TORQUE_ENABLE: 40, // 1 byte
    ACCELERATION: 41, // 1 byte
    GOAL_POSITION: 42, // 2 bytes, sign bit 15
    GOAL_VELOCITY: 46, // 2 bytes, sign bit 15
    TORQUE_LIMIT: 48, // 2 bytes, 0..1000 (0.1 %)
    PRESENT_POSITION: 56, // 2 bytes, sign bit 15
    PRESENT_VELOCITY: 58, // 2 bytes, sign bit 15
    PRESENT_LOAD: 60, // 2 bytes, sign bit 10, 0..1000 (0.1 %)
    PRESENT_VOLTAGE: 62, // 1 byte, 0.1 V
    PRESENT_TEMPERATURE: 63, // 1 byte, deg C
    MOVING: 66, // 1 byte
    PRESENT_CURRENT: 69 // 2 bytes
  };

  // One SYNC READ covers Present_Position (56) through Present_Current (69-70).
  const STATE_START = REG.PRESENT_POSITION;
  const STATE_LENGTH = REG.PRESENT_CURRENT + 2 - STATE_START;

  const STEPS_PER_TURN = 4096;
  const CENTER = 2048;

  function checksum(bytes) {
    let sum = 0;
    for (const b of bytes) sum += b;
    return ~sum & 0xff;
  }

  function makePacket(id, instruction, params = []) {
    const body = [id, params.length + 2, instruction, ...params];
    return Uint8Array.from([0xff, 0xff, ...body, checksum(body)]);
  }

  function u16(value) {
    return [value & 0xff, (value >> 8) & 0xff];
  }

  function readU16(bytes, offset) {
    return bytes[offset] | (bytes[offset + 1] << 8);
  }

  function encodeSignMagnitude(value, signBit) {
    const max = (1 << signBit) - 1;
    const magnitude = Math.min(Math.abs(Math.round(value)), max);
    return value < 0 ? magnitude | (1 << signBit) : magnitude;
  }

  function decodeSignMagnitude(raw, signBit) {
    const magnitude = raw & ((1 << signBit) - 1);
    return raw & (1 << signBit) ? -magnitude : magnitude;
  }

  const packets = {
    ping: (id) => makePacket(id, INST.PING),
    read: (id, address, length) => makePacket(id, INST.READ, [address, length]),
    write: (id, address, data) => makePacket(id, INST.WRITE, [address, ...data]),
    syncRead: (address, length, ids) =>
      makePacket(BROADCAST_ID, INST.SYNC_READ, [address, length, ...ids]),
    syncWrite: (address, length, entries) => {
      const params = [address, length];
      for (const { id, data } of entries) params.push(id, ...data);
      return makePacket(BROADCAST_ID, INST.SYNC_WRITE, params);
    }
  };

  // Pulls the first complete status packet out of `buffer` (an array of bytes),
  // removing it and any garbage before it. Returns null when more bytes are needed.
  function takePacket(buffer) {
    for (;;) {
      let start = -1;
      for (let i = 0; i + 1 < buffer.length; i++) {
        if (buffer[i] === 0xff && buffer[i + 1] === 0xff && buffer[i + 2] !== 0xff) {
          start = i;
          break;
        }
      }
      if (start < 0) {
        // Keep a trailing 0xFF; it may be the start of the next header.
        buffer.splice(0, buffer.length && buffer[buffer.length - 1] === 0xff ? buffer.length - 1 : buffer.length);
        return null;
      }
      if (start > 0) buffer.splice(0, start);
      if (buffer.length < 4) return null;
      const length = buffer[3];
      if (length < 2) {
        buffer.splice(0, 2);
        continue;
      }
      const total = length + 4;
      if (buffer.length < total) return null;
      const raw = buffer.slice(0, total);
      if (checksum(raw.slice(2, total - 1)) !== raw[total - 1]) {
        buffer.splice(0, 2);
        continue;
      }
      buffer.splice(0, total);
      return { id: raw[2], status: raw[4], params: raw.slice(5, total - 1), raw };
    }
  }

  function sameBytes(a, b) {
    if (a.length !== b.length) return false;
    for (let i = 0; i < a.length; i++) if (a[i] !== b[i]) return false;
    return true;
  }

  // ---------------------------------------------------------------------------
  // Serial bus: one request at a time, half duplex
  // ---------------------------------------------------------------------------

  class FeetechBus {
    constructor() {
      this.port = null;
      this.reader = null;
      this.writer = null;
      this.rx = [];
      this.onData = null;
      this.queue = Promise.resolve();
      this.timeoutMs = 30;
    }

    get isOpen() {
      return !!this.writer;
    }

    async open(port, baudRate) {
      await port.open({ baudRate });
      this.port = port;
      this.writer = port.writable.getWriter();
      this.reader = port.readable.getReader();
      this._readLoop();
    }

    async _readLoop() {
      try {
        for (;;) {
          const { value, done } = await this.reader.read();
          if (done) break;
          for (const b of value) this.rx.push(b);
          if (this.onData) this.onData();
        }
      } catch (e) {
        // Port lost (unplugged) or cancelled by close().
      }
      this._release();
    }

    _release() {
      try { this.reader && this.reader.releaseLock(); } catch (e) { /* already released */ }
      try { this.writer && this.writer.releaseLock(); } catch (e) { /* already released */ }
      this.reader = null;
      this.writer = null;
    }

    async close() {
      const port = this.port;
      this.port = null;
      try { if (this.reader) await this.reader.cancel(); } catch (e) { /* ignore */ }
      this._release();
      try { if (port) await port.close(); } catch (e) { /* ignore */ }
    }

    _waitForData(ms) {
      return new Promise((resolve) => {
        const timer = setTimeout(() => {
          this.onData = null;
          resolve();
        }, ms);
        this.onData = () => {
          clearTimeout(timer);
          this.onData = null;
          resolve();
        };
      });
    }

    // Sends `packet` and collects up to `expected` status packets.
    // Bytes identical to the sent packet are treated as adapter echo and skipped.
    exchange(packet, expected) {
      const run = async () => {
        if (!this.isOpen) return [];
        this.rx.length = 0;
        await this.writer.write(packet);
        const replies = [];
        const deadline = Date.now() + this.timeoutMs * Math.max(1, expected);
        while (replies.length < expected) {
          const reply = takePacket(this.rx);
          if (reply) {
            if (!sameBytes(reply.raw, packet)) replies.push(reply);
            continue;
          }
          const left = deadline - Date.now();
          if (left <= 0 || !this.isOpen) break;
          await this._waitForData(left);
        }
        return replies;
      };
      const result = this.queue.then(run, run);
      this.queue = result.catch(() => {});
      return result;
    }

    async ping(id) {
      const [reply] = await this.exchange(packets.ping(id), 1);
      return !!reply && reply.id === id;
    }

    async read(id, address, length) {
      const [reply] = await this.exchange(packets.read(id, address, length), 1);
      if (!reply || reply.id !== id || reply.params.length !== length) return null;
      return reply.params;
    }

    async write(id, address, data) {
      if (id === BROADCAST_ID) {
        await this.exchange(packets.write(id, address, data), 0);
        return true;
      }
      const [reply] = await this.exchange(packets.write(id, address, data), 1);
      return !!reply && reply.id === id;
    }

    async syncWrite(address, length, entries) {
      if (entries.length) await this.exchange(packets.syncWrite(address, length, entries), 0);
    }

    // Returns a Map of id -> params for every servo that answered.
    async syncRead(address, length, ids) {
      const replies = await this.exchange(packets.syncRead(address, length, ids), ids.length);
      const result = new Map();
      for (const r of replies) if (r.params.length === length) result.set(r.id, r.params);
      return result;
    }
  }

  const protocol = {
    INST,
    REG,
    BROADCAST_ID,
    checksum,
    makePacket,
    packets,
    takePacket,
    encodeSignMagnitude,
    decodeSignMagnitude,
    FeetechBus
  };

  if (typeof module !== 'undefined' && module.exports) {
    module.exports = protocol;
  }
  if (!Scratch) return;

  // ---------------------------------------------------------------------------
  // Scratch extension
  // ---------------------------------------------------------------------------

  if (!Scratch.extensions.unsandboxed) {
    throw new Error('The SO-100 Arm extension must be loaded without the sandbox.');
  }

  const JOINTS = [
    { id: 1, name: 'shoulder pan' },
    { id: 2, name: 'shoulder lift' },
    { id: 3, name: 'elbow flex' },
    { id: 4, name: 'wrist flex' },
    { id: 5, name: 'wrist roll' },
    { id: 6, name: 'gripper' }
  ];
  const GRIPPER_ID = 6;
  const BAUD_RATE = 1000000;
  const POLL_INTERVAL_MS = 33;
  const MAX_SPEED_STEPS = 3000; // steps/s that "100 %" speed maps to
  const DEFAULT_SPEED_PERCENT = 30;
  const DEFAULT_ACCELERATION = 50;
  const CALIBRATION_KEY = 'so100-arm-calibration';

  const BlockType = Scratch.BlockType;
  const ArgumentType = Scratch.ArgumentType;

  class SO100Arm {
    constructor(runtime) {
      this.runtime = runtime;
      this.bus = new FeetechBus();
      this.found = new Set();
      this.state = new Map();
      this.limits = new Map();
      this.pendingGoals = new Map();
      this.torque = new Map();
      this.polling = false;
      this.centers = this._loadCalibration();
      this.poses = {};

      runtime.on('PROJECT_STOP_ALL', () => this._holdPosition());

      if (navigator.serial) {
        navigator.serial.addEventListener('disconnect', (event) => {
          if (event.target === this.bus.port) this._lost();
        });
      }
    }

    getInfo() {
      const joint = { type: ArgumentType.STRING, menu: 'joint', defaultValue: '3' };
      return {
        id: 'so100arm',
        name: 'SO-100 Arm',
        color1: '#e07a2f',
        color2: '#c4621a',
        blocks: [
          { opcode: 'connect', blockType: BlockType.COMMAND, text: 'connect to robot' },
          { opcode: 'disconnect', blockType: BlockType.COMMAND, text: 'disconnect robot' },
          { opcode: 'isConnected', blockType: BlockType.BOOLEAN, text: 'robot connected?' },
          { opcode: 'isFound', blockType: BlockType.BOOLEAN, text: 'motor [JOINT] found?', arguments: { JOINT: joint } },
          '---',
          {
            opcode: 'moveTo',
            blockType: BlockType.COMMAND,
            text: 'move [JOINT] to [ANGLE] degrees',
            arguments: { JOINT: joint, ANGLE: { type: ArgumentType.ANGLE, defaultValue: 0 } }
          },
          {
            opcode: 'moveToAndWait',
            blockType: BlockType.COMMAND,
            text: 'move [JOINT] to [ANGLE] degrees and wait',
            arguments: { JOINT: joint, ANGLE: { type: ArgumentType.ANGLE, defaultValue: 0 } }
          },
          {
            opcode: 'moveBy',
            blockType: BlockType.COMMAND,
            text: 'turn [JOINT] by [ANGLE] degrees',
            arguments: { JOINT: joint, ANGLE: { type: ArgumentType.NUMBER, defaultValue: 10 } }
          },
          {
            opcode: 'moveAll',
            blockType: BlockType.COMMAND,
            text: 'move arm to pan [A1] lift [A2] elbow [A3] wrist [A4] roll [A5] gripper [A6]',
            arguments: {
              A1: { type: ArgumentType.NUMBER, defaultValue: 0 },
              A2: { type: ArgumentType.NUMBER, defaultValue: 0 },
              A3: { type: ArgumentType.NUMBER, defaultValue: 0 },
              A4: { type: ArgumentType.NUMBER, defaultValue: 0 },
              A5: { type: ArgumentType.NUMBER, defaultValue: 0 },
              A6: { type: ArgumentType.NUMBER, defaultValue: 0 }
            }
          },
          {
            opcode: 'setGripper',
            blockType: BlockType.COMMAND,
            text: 'set gripper to [PERCENT] % open',
            arguments: { PERCENT: { type: ArgumentType.NUMBER, defaultValue: 100 } }
          },
          {
            opcode: 'setSpeed',
            blockType: BlockType.COMMAND,
            text: 'set [JOINT] speed to [PERCENT] %',
            arguments: { JOINT: { type: ArgumentType.STRING, menu: 'jointOrAll', defaultValue: 'all' }, PERCENT: { type: ArgumentType.NUMBER, defaultValue: DEFAULT_SPEED_PERCENT } }
          },
          {
            opcode: 'setTorque',
            blockType: BlockType.COMMAND,
            text: 'turn motor [JOINT] [STATE]',
            arguments: {
              JOINT: { type: ArgumentType.STRING, menu: 'jointOrAll', defaultValue: 'all' },
              STATE: { type: ArgumentType.STRING, menu: 'onOff', defaultValue: 'on' }
            }
          },
          { opcode: 'emergencyStop', blockType: BlockType.COMMAND, text: 'emergency stop' },
          '---',
          { opcode: 'getPosition', blockType: BlockType.REPORTER, text: '[JOINT] angle', arguments: { JOINT: joint } },
          {
            opcode: 'getSensor',
            blockType: BlockType.REPORTER,
            text: '[JOINT] [SENSOR]',
            arguments: { JOINT: joint, SENSOR: { type: ArgumentType.STRING, menu: 'sensor', defaultValue: 'load' } }
          },
          { opcode: 'isMoving', blockType: BlockType.BOOLEAN, text: '[JOINT] moving?', arguments: { JOINT: joint } },
          {
            opcode: 'whenLoad',
            blockType: BlockType.HAT,
            isEdgeActivated: true,
            text: 'when [JOINT] load > [LOAD] %',
            arguments: { JOINT: joint, LOAD: { type: ArgumentType.NUMBER, defaultValue: 50 } }
          },
          '---',
          {
            opcode: 'savePose',
            blockType: BlockType.COMMAND,
            text: 'remember pose as [NAME]',
            arguments: { NAME: { type: ArgumentType.STRING, defaultValue: 'home' } }
          },
          {
            opcode: 'goToPose',
            blockType: BlockType.COMMAND,
            text: 'go to pose [NAME]',
            arguments: { NAME: { type: ArgumentType.STRING, defaultValue: 'home' } }
          },
          {
            opcode: 'setZero',
            blockType: BlockType.COMMAND,
            text: 'set [JOINT] zero to current position',
            arguments: { JOINT: { type: ArgumentType.STRING, menu: 'jointOrAll', defaultValue: 'all' } }
          }
        ],
        menus: {
          joint: { acceptReporters: true, items: JOINTS.map((j) => ({ text: j.name, value: String(j.id) })) },
          jointOrAll: {
            acceptReporters: true,
            items: [{ text: 'all', value: 'all' }, ...JOINTS.map((j) => ({ text: j.name, value: String(j.id) }))]
          },
          onOff: { acceptReporters: false, items: ['on', 'off'] },
          sensor: {
            acceptReporters: false,
            items: [
              { text: 'load %', value: 'load' },
              { text: 'speed (degrees/s)', value: 'speed' },
              { text: 'temperature (°C)', value: 'temperature' },
              { text: 'voltage (V)', value: 'voltage' },
              { text: 'current (raw)', value: 'current' },
              { text: 'raw position', value: 'raw' }
            ]
          }
        }
      };
    }

    // ----- helpers -----------------------------------------------------------

    _jointId(value) {
      const text = String(value).trim().toLowerCase();
      const byName = JOINTS.find((j) => j.name === text);
      if (byName) return byName.id;
      const id = Math.round(Number(text));
      return JOINTS.some((j) => j.id === id) ? id : null;
    }

    _jointIds(value) {
      if (String(value).trim().toLowerCase() === 'all') return [...this.found];
      const id = this._jointId(value);
      return id === null ? [] : [id];
    }

    _center(id) {
      return this.centers[id] ?? CENTER;
    }

    _clampRaw(id, raw) {
      const [min, max] = this.limits.get(id) || [0, STEPS_PER_TURN - 1];
      return Math.max(min, Math.min(max, Math.round(raw)));
    }

    _degreesToRaw(id, degrees) {
      return this._clampRaw(id, this._center(id) + (Number(degrees) || 0) * STEPS_PER_TURN / 360);
    }

    _rawToDegrees(id, raw) {
      return Math.round((raw - this._center(id)) * 3600 / STEPS_PER_TURN) / 10;
    }

    _loadCalibration() {
      try {
        return JSON.parse(localStorage.getItem(CALIBRATION_KEY)) || {};
      } catch (e) {
        return {};
      }
    }

    _saveCalibration() {
      try {
        localStorage.setItem(CALIBRATION_KEY, JSON.stringify(this.centers));
      } catch (e) {
        // Storage unavailable; calibration lasts until the page reloads.
      }
    }

    // ----- connection --------------------------------------------------------

    async connect() {
      if (this.bus.isOpen) return;
      if (!navigator.serial) {
        alert('This browser has no Web Serial. Use TurboWarp in Chrome or Edge on a computer.');
        return;
      }
      let port;
      try {
        port = await navigator.serial.requestPort();
      } catch (e) {
        return; // user cancelled the picker
      }
      try {
        await this.bus.open(port, BAUD_RATE);
      } catch (e) {
        alert('Could not open the serial port. Close other programs using it (Feetech FD, LeRobot) and try again.');
        return;
      }

      this.found.clear();
      for (const { id } of JOINTS) {
        if (await this.bus.ping(id)) this.found.add(id);
      }
      for (const id of this.found) {
        const limits = await this.bus.read(id, REG.MIN_POSITION_LIMIT, 4);
        if (limits) {
          const min = readU16(limits, 0);
          const max = readU16(limits, 2);
          if (max > min) this.limits.set(id, [min, max]);
        }
        const torque = await this.bus.read(id, REG.TORQUE_ENABLE, 1);
        this.torque.set(id, !!(torque && torque[0]));
        await this.bus.write(id, REG.ACCELERATION, [DEFAULT_ACCELERATION]);
        await this.bus.write(id, REG.GOAL_VELOCITY, u16(this._speedSteps(DEFAULT_SPEED_PERCENT)));
      }
      await this._readState();
      this._startPolling();
    }

    async disconnect() {
      if (!this.bus.isOpen) return;
      await this._setTorque([...this.found], false);
      this.polling = false;
      await this.bus.close();
      this._lost();
    }

    _lost() {
      this.polling = false;
      this.found.clear();
      this.pendingGoals.clear();
      this.bus.close();
    }

    isConnected() {
      return this.bus.isOpen;
    }

    isFound(args) {
      return this.found.has(this._jointId(args.JOINT));
    }

    // ----- background loop ---------------------------------------------------

    _startPolling() {
      if (this.polling) return;
      this.polling = true;
      const tick = async () => {
        if (!this.polling || !this.bus.isOpen) {
          this.polling = false;
          return;
        }
        const started = Date.now();
        try {
          await this._flushGoals();
          await this._readState();
        } catch (e) {
          // A failed tick is retried on the next one.
        }
        setTimeout(tick, Math.max(0, POLL_INTERVAL_MS - (Date.now() - started)));
      };
      tick();
    }

    async _flushGoals() {
      if (!this.pendingGoals.size) return;
      const entries = [];
      for (const [id, raw] of this.pendingGoals) {
        entries.push({ id, data: u16(encodeSignMagnitude(raw, 15)) });
      }
      this.pendingGoals.clear();
      await this.bus.syncWrite(REG.GOAL_POSITION, 2, entries);
    }

    async _readState() {
      const ids = [...this.found];
      if (!ids.length) return;
      const replies = await this.bus.syncRead(STATE_START, STATE_LENGTH, ids);
      for (const [id, p] of replies) {
        const at = (reg) => reg - STATE_START;
        this.state.set(id, {
          position: decodeSignMagnitude(readU16(p, at(REG.PRESENT_POSITION)), 15),
          velocity: decodeSignMagnitude(readU16(p, at(REG.PRESENT_VELOCITY)), 15),
          load: decodeSignMagnitude(readU16(p, at(REG.PRESENT_LOAD)), 10),
          voltage: p[at(REG.PRESENT_VOLTAGE)],
          temperature: p[at(REG.PRESENT_TEMPERATURE)],
          moving: p[at(REG.MOVING)] !== 0,
          current: readU16(p, at(REG.PRESENT_CURRENT))
        });
      }
    }

    // ----- motion ------------------------------------------------------------

    async _ensureTorque(id) {
      if (this.torque.get(id)) return true;
      // Hold where the joint is now, so enabling torque cannot jump to a stale goal.
      const now = await this.bus.read(id, REG.PRESENT_POSITION, 2);
      if (!now) return false;
      await this.bus.write(id, REG.GOAL_POSITION, now);
      await this.bus.write(id, REG.TORQUE_ENABLE, [1]);
      this.torque.set(id, true);
      return true;
    }

    async _setGoal(id, raw) {
      if (!this.found.has(id)) return false;
      if (!(await this._ensureTorque(id))) return false;
      this.pendingGoals.set(id, this._clampRaw(id, raw));
      return true;
    }

    async moveTo(args) {
      const id = this._jointId(args.JOINT);
      if (id !== null) await this._setGoal(id, this._degreesToRaw(id, args.ANGLE));
    }

    async moveToAndWait(args) {
      const id = this._jointId(args.JOINT);
      if (id === null) return;
      const goal = this._degreesToRaw(id, args.ANGLE);
      if (!(await this._setGoal(id, goal))) return;
      const deadline = Date.now() + 5000;
      await new Promise((resolve) => setTimeout(resolve, 2 * POLL_INTERVAL_MS));
      while (Date.now() < deadline && this.bus.isOpen) {
        const s = this.state.get(id);
        if (s && !s.moving && !this.pendingGoals.has(id) && Math.abs(s.position - goal) < 30) break;
        await new Promise((resolve) => setTimeout(resolve, POLL_INTERVAL_MS));
      }
    }

    async moveBy(args) {
      const id = this._jointId(args.JOINT);
      if (id === null || !this.found.has(id)) return;
      const base = this.pendingGoals.get(id) ?? this.state.get(id)?.position;
      if (base === undefined) return;
      await this._setGoal(id, base + (Number(args.ANGLE) || 0) * STEPS_PER_TURN / 360);
    }

    async moveAll(args) {
      const angles = [args.A1, args.A2, args.A3, args.A4, args.A5, args.A6];
      for (let i = 0; i < JOINTS.length; i++) {
        const id = JOINTS[i].id;
        await this._setGoal(id, this._degreesToRaw(id, angles[i]));
      }
    }

    async setGripper(args) {
      const [min, max] = this.limits.get(GRIPPER_ID) || [0, STEPS_PER_TURN - 1];
      const percent = Math.max(0, Math.min(100, Number(args.PERCENT) || 0));
      await this._setGoal(GRIPPER_ID, min + (max - min) * percent / 100);
    }

    _speedSteps(percent) {
      // Goal velocity 0 means "no limit" on STS servos, so never send 0.
      const p = Math.max(1, Math.min(100, Number(percent) || 0));
      return Math.round(MAX_SPEED_STEPS * p / 100);
    }

    async setSpeed(args) {
      const steps = this._speedSteps(args.PERCENT);
      for (const id of this._jointIds(args.JOINT)) {
        await this.bus.write(id, REG.GOAL_VELOCITY, u16(steps));
      }
    }

    async _setTorque(ids, on) {
      for (const id of ids) {
        if (on) {
          if (this.found.has(id)) await this._ensureTorque(id);
        } else {
          this.pendingGoals.delete(id);
          await this.bus.write(id, REG.TORQUE_ENABLE, [0]);
          this.torque.set(id, false);
        }
      }
    }

    async setTorque(args) {
      await this._setTorque(this._jointIds(args.JOINT), args.STATE === 'on');
    }

    async emergencyStop() {
      this.pendingGoals.clear();
      if (!this.bus.isOpen) return;
      // Broadcast write: every servo on the bus drops torque, no replies.
      await this.bus.write(BROADCAST_ID, REG.TORQUE_ENABLE, [0]);
      for (const id of this.found) this.torque.set(id, false);
    }

    // Red stop button: cancel queued moves and hold every joint where it is.
    _holdPosition() {
      this.pendingGoals.clear();
      for (const id of this.found) {
        const s = this.state.get(id);
        if (s && this.torque.get(id)) this.pendingGoals.set(id, s.position);
      }
    }

    // ----- feedback ----------------------------------------------------------

    getPosition(args) {
      const id = this._jointId(args.JOINT);
      const s = this.state.get(id);
      return s ? this._rawToDegrees(id, s.position) : 0;
    }

    getSensor(args) {
      const s = this.state.get(this._jointId(args.JOINT));
      if (!s) return 0;
      switch (args.SENSOR) {
        case 'load': return s.load / 10;
        case 'speed': return Math.round(s.velocity * 3600 / STEPS_PER_TURN) / 10;
        case 'temperature': return s.temperature;
        case 'voltage': return s.voltage / 10;
        case 'current': return s.current;
        case 'raw': return s.position;
        default: return 0;
      }
    }

    isMoving(args) {
      const s = this.state.get(this._jointId(args.JOINT));
      return !!(s && s.moving);
    }

    whenLoad(args) {
      const s = this.state.get(this._jointId(args.JOINT));
      return !!s && Math.abs(s.load / 10) > (Number(args.LOAD) || 0);
    }

    // ----- poses and calibration ---------------------------------------------

    savePose(args) {
      const pose = {};
      for (const id of this.found) {
        const s = this.state.get(id);
        if (s) pose[id] = s.position;
      }
      this.poses[String(args.NAME)] = pose;
    }

    async goToPose(args) {
      const pose = this.poses[String(args.NAME)];
      if (!pose) return;
      for (const id of Object.keys(pose)) await this._setGoal(Number(id), pose[id]);
    }

    setZero(args) {
      for (const id of this._jointIds(args.JOINT)) {
        const s = this.state.get(id);
        if (s) this.centers[id] = s.position;
      }
      this._saveCalibration();
    }
  }

  Scratch.extensions.register(new SO100Arm(Scratch.vm.runtime));
})(typeof Scratch !== 'undefined' ? Scratch : null);
