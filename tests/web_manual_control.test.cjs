// Offline regression: execute the actual TypeScript with synthetic HTTP/UI ports.
// No server, browser, ROS, control lease, or physical device is contacted.
const assert = require('node:assert/strict')
const { test } = require('node:test')
const fs = require('node:fs')
const path = require('node:path')
const root = path.resolve(__dirname, '..')
const sourceRoot = process.env.AGRI_MANUAL_TEST_ROOT || root
const ts = require(require.resolve('typescript', { paths: [path.join(root, 'agri_web')] }))
const { parse } = require(require.resolve('@vue/compiler-sfc', { paths: [path.join(root, 'agri_web')] }))

function execute(source, modules, globals = {}) {
  const output = ts.transpileModule(source, {
    compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022 },
  }).outputText
  const requireMock = name => {
    assert.ok(Object.hasOwn(modules, name), `Unexpected real dependency: ${name}`)
    return modules[name]
  }
  return new Function('require', 'exports', ...Object.keys(globals), `${output}; return exports`)(
    requireMock, {}, ...Object.values(globals))
}
const deferred = () => {
  let resolve, reject
  const promise = new Promise((yes, no) => { resolve = yes; reject = no })
  return { promise, resolve, reject }
}
const flush = () => new Promise(setImmediate)
const response = () => ({ token: 'synthetic-session', server_time: Date.now() / 1000, heartbeat_protocol: 4, limits: { linear: 1, angular: 5 } })

function control(overrides = {}) {
  const calls = { commands: [], stops: [], errors: [], intervals: [], arms: 0, closes: 0, warnings: [], channels: [] }
  let clock = 0, wallSkew = 0
  const api = {
    armManual: async () => { calls.arms++; return response() },
    manualCommand: async data => { calls.commands.push(data); return { success: true, seq: data.seq, applied: true } },
    stopManual: async token => { calls.stops.push(token) }, ...overrides,
  }
  const device = { value: { safety: { mode: 'base', motion_inhibited: false } } }
  const document = { visibilityState: 'visible' }
  class Channel {
    constructor() { calls.channels.push(this) }
    open() { return overrides.openChannel ? overrides.openChannel() : Promise.resolve() }
    send(data) { return api.manualCommand(data) }
    close() { calls.closes++ }
  }
  const source = parse(fs.readFileSync(path.join(sourceRoot, 'agri_web/src/components/ManualControl.vue'), 'utf8')).descriptor.scriptSetup.content
  const ui = execute(`${source}\nexport { arm, token, busy, stop, blurred, hidden, heartbeat, limits, direction, hold, neutral, keyDown, keyUp };`, {
    vue: { ref: value => ({ value }), computed: fn => ({ get value() { return fn() } }),
      onMounted() {}, onUnmounted() {}, watch() {} },
    'element-plus': { ElMessage: { error: value => calls.errors.push(value), warning: value => calls.warnings.push(value) } },
    '../composables/useRealtime': { useRealtime: () => ({ devices: device, safetyFresh: { value: true } }) },
    '../api/workspace': api, '../api/http': { apiUrl: p => p },
    '../control/manualChannel': { ManualChannel: Channel, manualTiming: { heartbeat: 80, lifetime: .2, maxGap: 200 } },
  }, { document, Date: { now: () => 1700000000000 + clock + wallSkew }, performance: { now: () => clock }, setInterval: (fn, ms) => { calls.intervals.push({ fn, ms }); return calls.intervals.length },
    clearInterval() {}, navigator: { sendBeacon: () => assert.fail('Unexpected beacon') } })
  return { ui, calls, device, document, advance: ms => { clock += ms }, shiftWall: ms => { wallSkew += ms } }
}

test('enable waits for a zero-velocity acknowledgement before enabling controls', async () => {
  const acknowledgement = deferred(), commands = []
  const { ui, calls } = control({ manualCommand: data => { commands.push(data); return acknowledgement.promise } })
  const arm = ui.arm()
  await flush()
  assert.equal(ui.token.value, null)
  assert.equal(ui.busy.value, true)
  assert.equal(calls.intervals.length, 0)
  assert.equal(commands.length, 1)
  assert.equal(commands[0].linear, 0)
  assert.equal(commands[0].angular, 0)
  assert.equal(commands[0].seq, 1)
  acknowledgement.resolve()
  await arm
  assert.equal(ui.token.value, 'synthetic-session')
  assert.equal(ui.limits.value.angular, 2)
  assert.equal(calls.intervals[0].ms, 80)
  assert.equal(calls.stops.length, 0)
  await ui.heartbeat()
  assert.equal(commands[1].seq, 2)
  ui.stop()
})

test('failed zero probe releases the acquired session without retry or movement', async () => {
  const commands = []
  const { ui, calls } = control({ manualCommand: async data => {
    commands.push(data); throw new Error('遥控通信应答超过 500 毫秒')
  } })
  await ui.arm()
  assert.equal(ui.token.value, null)
  assert.deepEqual(calls.stops, ['synthetic-session'])
  assert.equal(calls.intervals.length, 0)
  assert.equal(calls.arms, 1)
  assert.equal(commands.length, 1)
  assert.equal(commands[0].linear, 0)
  assert.match(calls.errors[0], /500 毫秒/)
})

test('losing focus while arm is pending cannot activate a late response', async () => {
  const pending = deferred()
  const { ui, calls } = control({ armManual: () => pending.promise })
  const arm = ui.arm()
  await flush()
  ui.blurred()
  pending.resolve(response())
  await arm
  assert.equal(ui.token.value, null)
  assert.equal(calls.commands.length, 0)
  assert.deepEqual(calls.stops, ['synthetic-session'])
})

test('hiding the page or losing eligibility during zero probe cancels activation', async () => {
  for (const invalidate of [s => { s.document.visibilityState = 'hidden' },
    s => { s.device.value.safety.motion_inhibited = true }, s => s.ui.stop()]) {
    const pending = deferred()
    const state = control({ manualCommand: () => pending.promise })
    const arm = state.ui.arm()
    await flush()
    invalidate(state)
    pending.resolve()
    await arm
    assert.equal(state.ui.token.value, null)
    assert.equal(state.calls.intervals.length, 0)
    assert.deepEqual(state.calls.stops, ['synthetic-session'])
  }
})

test('a later heartbeat failure still releases control without rearming', async () => {
  let count = 0
  const { ui, calls } = control({ manualCommand: async () => {
    if (++count > 1) throw new Error('遥控通信应答超过 500 毫秒')
  } })
  await ui.arm()
  await ui.heartbeat()
  assert.equal(ui.token.value, null)
  assert.deepEqual(calls.stops, ['synthetic-session'])
  assert.equal(calls.arms, 1)
})


test('heartbeats continue while an earlier acknowledgement is delayed', async () => {
  const pending = deferred(), commands = []
  const state = control({ manualCommand: data => {
    commands.push(data); return data.seq === 2 ? pending.promise : Promise.resolve()
  } })
  await state.ui.arm()
  const second = state.ui.heartbeat()
  state.advance(80)
  await state.ui.heartbeat()
  assert.deepEqual(commands.map(c => c.seq), [1, 2, 3])
  assert.equal(state.ui.token.value, 'synthetic-session')
  pending.resolve(); await second
  state.ui.stop()
})

test('a late acknowledgement after stop cannot re-enable control', async () => {
  const pending = deferred()
  const state = control({ manualCommand: data => data.seq === 1 ? Promise.resolve() : pending.promise })
  await state.ui.arm(); const pulse = state.ui.heartbeat()
  state.ui.stop(); pending.resolve(); await pulse
  assert.equal(state.ui.token.value, null)
  assert.equal(state.calls.arms, 1)
})

test('a stalled browser scheduler stops instead of resuming movement', async () => {
  const state = control(); await state.ui.arm()
  state.advance(201); await state.ui.heartbeat()
  assert.equal(state.ui.token.value, null)
  assert.equal(state.calls.commands.length, 1)
  assert.match(state.calls.errors[0], /调度中断/)
})

test('direction release sends a newer zero while a moving acknowledgement is pending', async () => {
  const pending = deferred()
  const state = control({ manualCommand: data => {
    state.calls.commands.push(data)
    return data.seq === 2 ? pending.promise : Promise.resolve()
  } })
  await state.ui.arm()
  state.ui.hold({ currentTarget: { setPointerCapture() {} }, pointerId: 1 }, 1, 0)
  state.ui.neutral(); await flush()
  const commands = state.calls.commands
  assert.equal(commands[1].linear, .15)
  assert.equal(commands[2].linear, 0)
  assert.ok(commands[2].seq > commands[1].seq)
  pending.resolve(); await flush(); state.ui.stop()
})

test('stopping during connection establishment does not acquire a lease', async () => {
  const pending = deferred()
  const state = control({ openChannel: () => pending.promise })
  const arm = state.ui.arm(); state.ui.blurred(); pending.resolve(); await arm
  assert.equal(state.calls.arms, 0)
  assert.equal(state.ui.token.value, null)
})

test('old backend capability is rejected before any direction control is enabled', async () => {
  const state = control({ armManual: async () => ({ ...response(), heartbeat_protocol: undefined }) })
  await state.ui.arm()
  assert.equal(state.ui.token.value, null)
  assert.equal(state.calls.commands.length, 0)
  assert.deepEqual(state.calls.stops, ['synthetic-session'])
  assert.match(state.calls.errors[0], /协议尚未更新/)
})

test('arm still rejects a response slower than 300ms', async () => {
  const state = control({ armManual: async () => { state.advance(301); return response() } })
  await state.ui.arm()
  assert.equal(state.ui.token.value, null)
  assert.equal(state.calls.commands.length, 0)
  assert.deepEqual(state.calls.stops, ['synthetic-session'])
  assert.match(state.calls.errors[0], /响应延迟过高/)
})

test('a dropped-frame ACK silently preserves a held direction and uses new deadlines', async () => {
  const s = control({ manualCommand: async f => {
    s.calls.commands.push(f)
    return { success: true, seq: f.seq, applied: f.seq !== 2, reason: f.seq === 2 ? 'expired' : undefined }
  } })
  await s.ui.arm()
  s.ui.hold({ currentTarget: { setPointerCapture() {} }, pointerId: 1 }, 1, 0)
  await flush()
  s.advance(80); await s.ui.heartbeat()
  assert.equal(s.ui.token.value, 'synthetic-session')
  assert.equal(s.calls.commands.at(-1).linear, .15)
  assert.ok(s.calls.commands.at(-1).expires_at > s.calls.commands[1].expires_at)
  assert.equal(s.calls.arms, 1)
  assert.deepEqual(s.calls.errors, []); assert.deepEqual(s.calls.warnings, [])
  s.ui.neutral(); await flush()
  assert.equal(s.calls.commands.at(-1).linear, 0)
  s.ui.stop()
})

test('keyboard hold continues across a dropped frame and release still zeros', async () => {
  const s = control({ manualCommand: async f => {
    s.calls.commands.push(f)
    return { success: true, seq: f.seq, applied: f.seq < 3, reason: 'expired' }
  } })
  await s.ui.arm()
  const event = code => ({ code, repeat: false, target: {}, preventDefault() {} })
  s.ui.keyDown(event('Space')); s.ui.keyDown(event('KeyW')); await flush()
  s.advance(80); await s.ui.heartbeat()
  assert.equal(s.calls.commands.at(-1).linear, .15)
  s.ui.keyUp(event('KeyW')); await flush()
  assert.equal(s.calls.commands.at(-1).linear, 0)
  assert.deepEqual(s.calls.errors, []); assert.deepEqual(s.calls.warnings, [])
  s.ui.stop()
})

test('late dropped ACK after release cannot restore the old direction', async () => {
  const pending = deferred()
  const s = control({ manualCommand: f => {
    s.calls.commands.push(f)
    return f.seq === 2 ? pending.promise : Promise.resolve({ success: true, seq: f.seq, applied: true })
  } })
  await s.ui.arm()
  s.ui.hold({ currentTarget: { setPointerCapture() {} }, pointerId: 1 }, 1, 0)
  s.ui.neutral()
  pending.resolve({ success: true, seq: 2, applied: false, reason: 'expired' }); await flush()
  s.advance(80); await s.ui.heartbeat()
  assert.equal(s.calls.commands.at(-1).linear, 0)
  assert.deepEqual(s.calls.warnings, [])
  s.ui.stop()
})

test('system wall-clock changes do not change the command lifetime estimate', async () => {
  const s = control(); await s.ui.arm()
  const first = s.calls.commands[0].expires_at
  s.shiftWall(3600000); s.advance(80); await s.ui.heartbeat()
  assert.ok(Math.abs(s.calls.commands.at(-1).expires_at - first - .08) < .0001)
  s.ui.stop()
})

test('old expiry warning and recovery prompt are absent from the rendered component', () => {
  const source = fs.readFileSync(path.join(sourceRoot, 'agri_web/src/components/ManualControl.vue'), 'utf8')
  assert.doesNotMatch(source, /通信波动|旧指令已丢弃|松开后重新按方向|neutralRequired|recoverForIntent/)
})
