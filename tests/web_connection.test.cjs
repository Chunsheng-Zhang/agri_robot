// Offline regression using the real TypeScript and Vue reactivity; no network.
const assert = require('node:assert/strict')
const { test } = require('node:test')
const fs = require('node:fs')
const path = require('node:path')
const root = path.resolve(__dirname, '..')
const deps = [path.join(root, 'agri_web')]
const ts = require(require.resolve('typescript', { paths: deps }))
const vue = require(require.resolve('vue', { paths: deps }))
const read = p => fs.readFileSync(path.join(root, 'agri_web', p), 'utf8')
const flush = () => new Promise(setImmediate)
const deferred = () => {
  let resolve, reject
  const promise = new Promise((yes, no) => { resolve = yes; reject = no })
  return { promise, resolve, reject }
}
function execute(source, modules, globals) {
  const code = ts.transpileModule(source, {
    compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022 },
  }).outputText
  return new Function('require', 'exports', ...Object.keys(globals), `${code}; return exports`)(name => {
    assert.ok(Object.hasOwn(modules, name), `Unexpected real dependency: ${name}`)
    return modules[name]
  }, {}, ...Object.values(globals))
}

function http(fetch) {
  const timers = []
  const api = execute(read('src/api/http.ts').replace('import.meta.env.VITE_API_BASE_URL', 'undefined'), {}, {
    fetch, window: { setTimeout: (fn, ms) => { timers.push({ fn, ms }); return timers.length },
      clearTimeout: id => { timers[id - 1].cleared = true } },
  })
  return { api, timers }
}
test('query timeout names the failed endpoint and does not assert ROS disconnected', async () => {
  const { api, timers } = http((_url, options) => new Promise((_, reject) => {
    options.signal.addEventListener('abort', () => reject(new Error('signal is aborted without reason')))
  }))
  const request = api.apiRequest('/api/devices')
  assert.equal(timers[0].ms, 2000)
  timers[0].fn()
  await assert.rejects(request, /后端响应超时.*\/api\/devices.*无法确认 ROS/)
  assert.equal(timers[0].cleared, true)
})
test('abort during response body is not misreported as invalid JSON', async () => {
  const { api, timers } = http(async (_url, options) => ({ ok: true,
    json: () => new Promise((_, reject) => options.signal.addEventListener('abort', () => reject(new Error('aborted')))),
  }))
  const request = api.apiRequest('/api/robot/status')
  await flush(); timers[0].fn()
  await assert.rejects(request, /后端响应超时.*\/api\/robot\/status/)
})
test('mutation timeout keeps its original deadline and requests state confirmation', async () => {
  const { api, timers } = http((_url, options) => new Promise((_, reject) => {
    options.signal.addEventListener('abort', () => reject(new Error('aborted')))
  }))
  const request = api.apiRequest('/api/missions/id/start', { method: 'POST' })
  assert.equal(timers[0].ms, 20000)
  timers[0].fn()
  await assert.rejects(request, /请求超时.*核对实际状态/)
})

function realtime() {
  const mounted = [], unmounted = [], timers = [], events = {}, statuses = [], devices = []
  let time = 100000
  const api = execute(read('src/composables/useRealtime.ts'), {
    vue: { ...vue, onMounted: fn => mounted.push(fn), onUnmounted: fn => unmounted.push(fn) },
    '../api/robot': {
      getRobotStatus: () => { const p = deferred(); statuses.push(p); return p.promise },
      getDevices: () => { const p = deferred(); devices.push(p); return p.promise },
    },
  }, { Date: { now: () => time }, setInterval: (fn, ms) => { timers.push({ fn, ms }); return timers.length },
    clearInterval() {}, document: { visibilityState: 'visible',
      addEventListener: (name, fn) => { events[name] = fn }, removeEventListener: name => { delete events[name] } } })
  const state = api.useRealtime()
  return { state, statuses, devices, mount: () => mounted.forEach(fn => fn()), unmount: () => unmounted.forEach(fn => fn()),
    advance: ms => { time += ms; timers.find(t => t.ms === 250).fn() },
    poll: () => timers.find(t => t.ms === 500).fn() }
}
const robot = () => ({ ros_connected: true, online: true, odom_age_seconds: .02,
  speed_source: 'mgx23_ned', velocity_age_seconds: .02 })
const device = () => ({ safety: { fresh: true, age_seconds: .02, mode: 'base', emergency_stop: false },
  voltage: { fresh: true, age_seconds: 1.5 } })
test('slow device query does not block robot results or the next robot refresh', async () => {
  const r = realtime(); r.mount()
  r.statuses[0].resolve(robot()); await flush()
  assert.equal(r.state.robot.value.ros_connected, true)
  assert.equal(r.state.devices.value, null)
  r.advance(500); r.poll(); await flush()
  assert.equal(r.statuses.length, 2)
  assert.equal(r.devices.length, 1) // Each resource remains single-flight.
  r.statuses[1].resolve(robot()); r.devices[0].reject(new Error('devices timeout')); await flush()
  assert.equal(r.state.robotFresh.value, true)
  assert.equal(r.state.safetyFresh.value, false)
  assert.match(r.state.error.value, /devices timeout/)
  r.unmount()
})
test('network delay counts toward freshness instead of making old data look new', async () => {
  const r = realtime(); r.mount(); r.advance(1600)
  r.statuses[0].resolve(robot()); r.devices[0].resolve(device()); await flush()
  assert.equal(r.state.robotFresh.value, true)
  assert.equal(r.state.poseFresh.value, false)
  assert.equal(r.state.velocityFresh.value, false)
  assert.equal(r.state.safetyFresh.value, false)
  r.advance(1000)
  assert.equal(r.state.robotFresh.value, false)
  r.unmount()
})
test('timely safety remains usable and voltage retains its separate 3-second budget', async () => {
  const r = realtime(); r.mount(); r.advance(100)
  r.statuses[0].resolve(robot()); r.devices[0].resolve(device()); await flush()
  assert.equal(r.state.poseFresh.value, true)
  assert.equal(r.state.safetyFresh.value, true)
  assert.equal(r.state.isFresh(r.state.devices.value.voltage, 3000), true)
  r.advance(900)
  assert.equal(r.state.safetyFresh.value, false)
  r.unmount()
})
test('late responses after unmount cannot repopulate state', async () => {
  const r = realtime(); r.mount(); r.unmount()
  r.statuses[0].resolve(robot()); r.devices[0].resolve(device()); await flush()
  assert.equal(r.state.robot.value, null)
  assert.equal(r.state.devices.value, null)
})
