const test = require('node:test');
const assert = require('node:assert');
const p = require('../so100.js');

const hex = (bytes) => Buffer.from(bytes).toString('hex').match(/../g).join(' ');

test('ping packet matches the Feetech reference bytes', () => {
  assert.strictEqual(hex(p.packets.ping(1)), 'ff ff 01 02 01 fb');
});

test('read present position packet', () => {
  // ID 1, READ, addr 56, length 2: checksum = ~(1+4+2+56+2) = 0xBE
  assert.strictEqual(hex(p.packets.read(1, 56, 2)), 'ff ff 01 04 02 38 02 be');
});

test('sync write builds one broadcast packet', () => {
  const pkt = p.packets.syncWrite(42, 2, [{ id: 1, data: [0x00, 0x08] }, { id: 2, data: [0xff, 0x0f] }]);
  assert.strictEqual(hex(pkt.slice(0, 5)), 'ff ff fe 0a 83');
  assert.strictEqual(pkt[pkt.length - 1], p.checksum(Array.from(pkt.slice(2, -1))));
});

test('sign-magnitude round trip', () => {
  assert.strictEqual(p.encodeSignMagnitude(-100, 15), 0x8064);
  assert.strictEqual(p.decodeSignMagnitude(0x8064, 15), -100);
  assert.strictEqual(p.decodeSignMagnitude(0x0400 | 250, 10), -250);
  assert.strictEqual(p.decodeSignMagnitude(2048, 15), 2048);
});

test('takePacket skips garbage and bad checksums, waits for partial data', () => {
  const good = [0xff, 0xff, 0x01, 0x04, 0x00, 0x00, 0x08];
  good.push(p.checksum(good.slice(2)));
  const bad = [0xff, 0xff, 0x02, 0x02, 0x00, 0x00];
  const buf = [0x12, 0x34, ...bad, ...good.slice(0, 5)];
  assert.strictEqual(p.takePacket(buf), null);
  buf.push(...good.slice(5));
  const pkt = p.takePacket(buf);
  assert.deepStrictEqual(pkt.params, [0x00, 0x08]);
  assert.strictEqual(pkt.id, 1);
  assert.strictEqual(buf.length, 0);
});

function fakePort(respond) {
  let push;
  const queue = [];
  const reader = {
    read: () => queue.length ? Promise.resolve(queue.shift()) : new Promise((r) => { push = r; }),
    cancel: async () => { if (push) push({ done: true }); },
    releaseLock: () => {}
  };
  const deliver = (bytes) => {
    const chunk = { value: Uint8Array.from(bytes), done: false };
    if (push) { const r = push; push = null; r(chunk); } else queue.push(chunk);
  };
  return {
    open: async () => {},
    close: async () => {},
    readable: { getReader: () => reader },
    writable: { getWriter: () => ({ write: async (pkt) => setTimeout(() => respond(Array.from(pkt), deliver), 1), releaseLock: () => {} }) }
  };
}

function status(id, params) {
  const body = [id, params.length + 2, 0, ...params];
  return [0xff, 0xff, ...body, p.checksum(body)];
}

test('bus ignores the echo of its own packet and reads the reply', async () => {
  const bus = new p.FeetechBus();
  await bus.open(fakePort((sent, deliver) => {
    deliver(sent); // adapter echo
    deliver(status(sent[2], [0x00, 0x08]));
  }), 1000000);
  const data = await bus.read(3, 56, 2);
  assert.deepStrictEqual(data, [0x00, 0x08]);
  await bus.close();
});

test('sync read collects one reply per servo and times out on missing ones', async () => {
  const bus = new p.FeetechBus();
  await bus.open(fakePort((sent, deliver) => {
    const ids = sent.slice(7, -1);
    for (const id of ids) if (id !== 2) deliver(status(id, [id, 0]));
  }), 1000000);
  const result = await bus.syncRead(56, 2, [1, 2, 3]);
  assert.deepStrictEqual([...result.keys()].sort(), [1, 3]);
  assert.deepStrictEqual(result.get(3), [3, 0]);
  assert.strictEqual(await bus.ping(2), false);
  await bus.close();
});
