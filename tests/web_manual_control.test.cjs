// Offline regression: execute the actual TypeScript with synthetic HTTP/UI ports.
// No server, browser, ROS, control lease, or physical device is contacted.
const assert = require('node:assert/strict')
const { test } = require('node:test')
const fs = require('node:fs')
const path = require('node:path')
const root = path.resolve(__dirname, '..')
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
const response = () => ({ token: 'synthetic-session', server_time: Date.now() / 1000, limits: { linear: 1, angular: 5 } })

function manualApi(fetch) {
  const timers = []
  const api = execute(fs.readFileSync(path.join(root, 'agri_web/src/api/workspace.ts'), 'utf8'), {
    './http': { apiUrl: p => p, apiRequest: () => assert.fail('Unexpected HTTP request') },
  }, {
    fetch, setTimeout: (callback, ms) => { timers.push({ callback, ms }); return timers.length },
    clearTimeout: id => { timers[id - 1].cleared = true },
  })
  return { api, timers }
}

function control(overrides = {}) {
  const calls = { commands: [], stops: [], errors: [], intervals: [], arms: 0 }
  const api = {
    armManual: async () => { calls.arms++; return response() },
    manualCommand: async data => { calls.commands.push(data) },
    stopManual: async token => { calls.stops.push(token) }, ...overrides,
  }
  const device = { value: { safety: { mode: 'base', motion_inhibited: false } } }
  const document = { visibilityState: 'visible' }
  const source = parse(fs.readFileSync(path.join(root, 'agri_web/src/components/ManualControl.vue'), 'utf8')).descriptor.scriptSetup.content
  const ui = execute(`${source}\nexport { arm, token, busy, stop, blurred, hidden, heartbeat, limits };`, {
    vue: { ref: value => ({ value }), computed: fn => ({ get value() { return fn() } }),
      onMounted() {}, onUnmounted() {}, watch() {} },
    'element-plus': { ElMessage: { error: value => calls.errors.push(value) } },
    '../composables/useRealtime': { useRealtime: () => ({ devices: device, safetyFresh: { value: true } }) },
    '../api/workspace': api, '../api/http': { apiUrl: p => p },
  }, { document, setInterval: (fn, ms) => { calls.intervals.push({ fn, ms }); return calls.intervals.length },
    clearInterval() {}, navigator: { sendBeacon: () => assert.fail('Unexpected beacon') } })
  return { ui, calls, device, document }
}

test('180 ms abort becomes a useful error and retains the original deadline', async () => {
  const { api, timers } = manualApi((_url, options) => new Promise((_, reject) => {
    options.signal.addEventListener('abort', () => reject(new Error('signal is aborted without reason')), { once: true })
  }))
  const command = api.manualCommand({ token: 'synthetic', seq: 1, linear: 0, angular: 0, expires_at: 1 })
  assert.equal(timers[0].ms, 180)
  timers[0].callback()
  await assert.rejects(command, /遥控通信超过 180 毫秒/)
  assert.equal(timers[0].cleared, true)
})

test('ROS rejection and genuine network failures preserve their explanations', async () => {
  const rejected = manualApi(async () => ({ ok: false, json: async () => ({ detail: 'Web 没有底盘控制权' }) }))
  await assert.rejects(rejected.api.manualCommand({}), /Web 没有底盘控制权/)
  const network = manualApi(async () => { throw new TypeError('Failed to fetch') })
  await assert.rejects(network.api.manualCommand({}), /Failed to fetch/)
})

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
    commands.push(data); throw new Error('遥控通信超过 180 毫秒')
  } })
  await ui.arm()
  assert.equal(ui.token.value, null)
  assert.deepEqual(calls.stops, ['synthetic-session'])
  assert.equal(calls.intervals.length, 0)
  assert.equal(calls.arms, 1)
  assert.equal(commands.length, 1)
  assert.equal(commands[0].linear, 0)
  assert.match(calls.errors[0], /180 毫秒/)
})

test('losing focus while arm is pending cannot activate a late response', async () => {
  const pending = deferred()
  const { ui, calls } = control({ armManual: () => pending.promise })
  const arm = ui.arm()
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
    if (++count > 1) throw new Error('遥控通信超过 180 毫秒')
  } })
  await ui.arm()
  await ui.heartbeat()
  assert.equal(ui.token.value, null)
  assert.deepEqual(calls.stops, ['synthetic-session'])
  assert.equal(calls.arms, 1)
})
