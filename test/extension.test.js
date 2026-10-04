const test = require('node:test');
const assert = require('node:assert');
const fs = require('node:fs');
const path = require('node:path');

test('extension registers with TurboWarp-style Scratch and defines its blocks', () => {
  let registered;
  const Scratch = {
    extensions: { unsandboxed: true, register: (ext) => { registered = ext; } },
    BlockType: { COMMAND: 'command', REPORTER: 'reporter', BOOLEAN: 'Boolean', HAT: 'hat' },
    ArgumentType: { STRING: 'string', NUMBER: 'number', ANGLE: 'angle' },
    vm: { runtime: { on: () => {} } }
  };
  const source = fs.readFileSync(path.join(__dirname, '..', 'so100.js'), 'utf8');
  new Function('Scratch', 'module', source)(Scratch, undefined);
  const info = registered.getInfo();
  assert.strictEqual(info.id, 'so100arm');
  for (const block of info.blocks) {
    if (typeof block === 'string') continue;
    assert.strictEqual(typeof registered[block.opcode], 'function', block.opcode);
    for (const name of Object.keys(block.arguments || {})) {
      assert.ok(block.text.includes(`[${name}]`), `${block.opcode} uses [${name}]`);
      const menu = block.arguments[name].menu;
      if (menu) assert.ok(info.menus[menu], `menu ${menu}`);
    }
  }
  // Reporters are safe to use before connecting.
  assert.strictEqual(registered.getPosition({ JOINT: '3' }), 0);
  assert.strictEqual(registered.isConnected(), false);
  assert.strictEqual(registered._jointId('elbow flex'), 3);
});
