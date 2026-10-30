// Offline tests of the real preview state machine; no network or real camera.
const assert = require('node:assert/strict')
const { test } = require('node:test')
const fs = require('node:fs')
const path = require('node:path')
const root = path.resolve(__dirname, '..')
const deps = [path.join(root, 'agri_web')]
const ts = require(require.resolve('typescript', { paths: deps }))
const vue = require(require.resolve('vue', { paths: deps }))
const flush = () => new Promise(setImmediate)
function deferred() {
  let resolve, reject
  const promise = new Promise((yes, no) => { resolve = yes; reject = no })
  return { promise, resolve, reject }
}
const jpeg = () => ({ ok: true, blob: async () => new Blob(['synthetic JPEG'], { type: 'image/jpeg' }) })

function preview() {
  const mounted = [], before = [], unmounted = [], events = {}, timers = new Map()
  const frames = [], revoked = [], metadata = [], blobs = []
  let nextTimer = 0
  const doc = { visibilityState: 'visible', addEventListener: (name, fn) => { events[name] = fn },
    removeEventListener: name => { delete events[name] } }
  const devices = vue.ref({ camera: { connected: true } })
  class PreviewURL extends URL {
    static createObjectURL(blob) { blobs.push(blob); return `blob:preview-${blobs.length}` }
    static revokeObjectURL(url) { revoked.push(url) }
  }
  const source = fs.readFileSync(path.join(root, 'agri_web/src/composables/useCameraPreview.ts'), 'utf8')
  const code = ts.transpileModule(source, {
    compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022 },
  }).outputText
  const modules = {
    vue: { ...vue, onMounted: fn => mounted.push(fn), onBeforeUnmount: fn => before.push(fn), onUnmounted: fn => unmounted.push(fn) },
    '../api/robot': { getCameraStream: async () => {
      metadata.push(1)
      return { stream_url: '/video/stream?topic=/camera/color/image_raw&type=mjpeg&quality=70' }
    } },
    './useRealtime': { useRealtime: () => ({ devices, isFresh: value => value !== null && value !== undefined }) },
  }
  const api = new Function('require', 'exports', 'document', 'window', 'URL', 'fetch', 'setTimeout', 'clearTimeout', `${code}; return exports`)(
    name => { assert.ok(Object.hasOwn(modules, name)); return modules[name] }, {}, doc,
    { location: { origin: 'http://synthetic.invalid' } }, PreviewURL,
    (url, options) => { const p = deferred(); frames.push({ ...p, url, options }); return p.promise },
    (fn, ms) => { timers.set(++nextTimer, { fn, ms }); return nextTimer }, id => timers.delete(id))
  const state = api.useCameraPreview()
  return { state, frames, revoked, metadata, doc, events, devices, timers,
    mount: () => mounted.forEach(fn => fn()),
    unmount: () => { before.forEach(fn => fn()); unmounted.forEach(fn => fn()) },
    run: ms => {
      const entry = [...timers].find(([, t]) => t.ms === ms)
      assert.ok(entry, `No ${ms} ms timer`)
      timers.delete(entry[0]); entry[1].fn()
    } }
}

test('preview is opt-in, including after mounting and visibility changes', async () => {
  const p = preview(); p.mount()
  p.doc.visibilityState = 'hidden'; p.events.visibilitychange()
  p.doc.visibilityState = 'visible'; p.events.visibilitychange()
  await flush()
  assert.equal(p.state.enabled.value, false)
  assert.equal(p.frames.length, 0)
  assert.equal(p.metadata.length, 0)
  p.unmount()
})
test('uses a small single JPEG request, preserves ROS slashes, and schedules after completion', async () => {
  const p = preview(); p.mount(); p.state.toggle(); await flush()
  assert.equal(p.frames.length, 1)
  const f = p.frames[0], url = new URL(f.url)
  assert.equal(url.pathname, '/video/snapshot')
  assert.equal(url.searchParams.get('width'), '320')
  assert.equal(url.searchParams.get('height'), '240')
  assert.equal(url.searchParams.get('quality'), '40')
  assert.ok(f.url.includes('topic=/camera/color/image_raw'))
  assert.equal(f.options.cache, 'no-store')
  assert.ok(![...p.timers.values()].some(t => t.ms === 500))
  f.resolve(jpeg()); await flush()
  assert.equal(p.state.imageUrl.value, 'blob:preview-1')
  p.run(500); await flush()
  assert.equal(p.frames.length, 2)
  assert.ok(![...p.timers.values()].some(t => t.ms === 500))
  p.frames[1].resolve(jpeg()); await flush()
  assert.deepEqual(p.revoked, ['blob:preview-1'])
  p.state.toggle()
  assert.equal(p.state.imageUrl.value, '')
  assert.deepEqual(p.revoked, ['blob:preview-1', 'blob:preview-2'])
  assert.equal(p.timers.size, 0)
  p.unmount()
})
test('pause aborts pending frame; a late response cannot replace a new session image', async () => {
  const p = preview(); p.mount(); p.state.toggle(); await flush()
  const old = p.frames[0]
  p.state.toggle(); assert.equal(old.options.signal.aborted, true)
  p.state.toggle(); await flush()
  p.frames[1].resolve(jpeg()); await flush()
  const image = p.state.imageUrl.value
  old.resolve(jpeg()); await flush()
  assert.equal(p.state.imageUrl.value, image)
  assert.equal(p.frames.length, 2)
  p.unmount()
})
test('hidden/offline panels cancel images and only an enabled visible panel resumes', async () => {
  const p = preview(); p.mount(); p.state.toggle(); await flush()
  p.doc.visibilityState = 'hidden'; p.events.visibilitychange()
  assert.equal(p.frames[0].options.signal.aborted, true)
  assert.equal(p.timers.size, 0)
  p.doc.visibilityState = 'visible'; p.events.visibilitychange(); await flush()
  assert.equal(p.frames.length, 2)
  p.devices.value.camera.connected = false
  assert.equal(p.frames[1].options.signal.aborted, true)
  p.state.toggle() // Pause while disconnected.
  p.devices.value.camera.connected = true; await flush()
  assert.equal(p.frames.length, 2)
  p.unmount()
})
test('unmount aborts downloads and drops late results without rescheduling', async () => {
  const p = preview(); p.mount(); p.state.toggle(); await flush()
  p.unmount(); assert.equal(p.frames[0].options.signal.aborted, true)
  p.frames[0].resolve(jpeg()); await flush()
  assert.equal(p.state.imageUrl.value, '')
  assert.equal(p.timers.size, 0)
  assert.equal(p.events.visibilitychange, undefined)
})
test('HTTP and invalid-image failures clear old frames and back off for two seconds', async () => {
  const p = preview(); p.mount(); p.state.toggle(); await flush()
  p.frames[0].resolve({ ok: false, status: 503 }); await flush()
  assert.match(p.state.error.value, /503/)
  assert.equal(p.frames.length, 1)
  p.run(2000); await flush()
  p.frames[1].resolve({ ok: true, blob: async () => new Blob(['html'], { type: 'text/html' }) }); await flush()
  assert.match(p.state.error.value, /JPEG/)
  assert.equal(p.state.imageUrl.value, '')
  p.unmount()
})
test('stalled snapshot aborts at 1.5 seconds instead of accumulating more requests', async () => {
  const p = preview(); p.mount(); p.state.toggle(); await flush()
  p.frames[0].options.signal.addEventListener('abort', () => p.frames[0].reject(new Error('aborted')))
  p.run(1500); await flush()
  assert.match(p.state.error.value, /画面请求超时/)
  assert.equal(p.frames.length, 1)
  assert.ok([...p.timers.values()].some(t => t.ms === 2000))
  p.unmount()
})
