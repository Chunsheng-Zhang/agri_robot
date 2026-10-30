// Actual Vue/TypeScript state machine with a fake Leaflet renderer. No real requests.
const assert = require('node:assert/strict')
const { test } = require('node:test')
const fs = require('node:fs')
const path = require('node:path')
const root = path.resolve(__dirname, '..'), deps = [path.join(root, 'agri_web')]
const ts = require(require.resolve('typescript', { paths: deps }))
const vue = require(require.resolve('vue', { paths: deps }))
const flush = () => vue.nextTick()
const key = 'a'.repeat(32)
const position = (longitude = 117.02) => ({ latitude: 36.64, longitude, bearing: 90, reference: 'base_link', crs: 'EPSG:4326', source: 'mgx23_localization' })
function fixture(storedKey = '') {
  const mounted = [], unmounted = [], events = {}, timers = new Map(), storage = new Map()
  if (storedKey) storage.set('agri-robot.tianditu.web-key', storedKey)
  const tiles = [], moves = [], icons = [], markers = [], mapEvents = {}
  let observer, intersection, timerId = 0, mapCount = 0, removed = false
  const robot = vue.ref(null), poseFresh = vue.ref(false), context = vue.ref({ origin: { latitude: 36.64, longitude: 117.02 } })
  const doc = { visibilityState: 'visible', addEventListener: (event, fn) => { events[event] = fn }, removeEventListener: event => { delete events[event] } }
  const map = { setView: (...args) => moves.push(['view', ...args]), panTo: (...args) => moves.push(['pan', ...args]),
    getZoom: () => 18, getSize: () => ({ x: 800, y: 360 }), latLngToContainerPoint: () => ({ x: 400, y: 180 }),
    on: (event, fn) => { mapEvents[event] = fn }, invalidateSize() {}, remove: () => { removed = true } }
  const leaflet = {
    map: () => { mapCount++; return map }, control: { scale: () => ({ addTo() {} }) },
    divIcon: icon => { icons.push(icon); return icon },
    marker: (latlng, options) => { const value = { latlng, icon: options.icon, tooltip: '', addTo() { return this },
      bindTooltip() { return this }, setLatLng(p) { this.latlng = p; return this }, setIcon(icon) { this.icon = icon; return this }, setTooltipContent(text) { this.tooltip = text; return this } }; markers.push(value); return value },
    tileLayer: (url, options) => { const layer = { url, options, events: {}, added: false,
      on(name, fn) { this.events[name] = fn; return this }, off() { this.events = {} },
      remove() { this.added = false }, addTo() { this.added = true; this.events.loading?.(); return this } }; tiles.push(layer); return layer },
  }
  const source = fs.readFileSync(path.join(root, 'agri_web/src/composables/useLocationMap.ts'), 'utf8')
  const code = ts.transpileModule(source, { compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022 } }).outputText
  const modules = { vue: { ...vue, onMounted: fn => mounted.push(fn), onUnmounted: fn => unmounted.push(fn) }, leaflet,
    './useRealtime': { useRealtime: () => ({ robot, poseFresh, refresh: async () => {} }) },
    './useWorkspace': { useWorkspace: () => ({ context }) } }
  const api = new Function('require', 'exports', 'document', 'localStorage', 'ResizeObserver', 'IntersectionObserver', 'setTimeout', 'clearTimeout', `${code}; return exports`)(
    name => { assert.ok(modules[name], name); return modules[name] }, {}, doc,
    { getItem: name => storage.get(name) || null, setItem: (name, value) => storage.set(name, value), removeItem: name => storage.delete(name) },
    class { constructor(fn) { observer = this; this.fn = fn } observe() {} disconnect() { this.disconnected = true } },
    class { constructor(fn) { intersection = this; this.fn = fn } observe() {} disconnect() { this.disconnected = true } },
    (fn, ms) => { timers.set(++timerId, { fn, ms }); return timerId }, id => timers.delete(id))
  const state = api.useLocationMap(vue.ref({}))
  return { state, api, robot, poseFresh, context, tiles, moves, icons, markers, doc, events, timers, storage, map, mapEvents,
    mount: () => mounted.forEach(fn => fn()), unmount: () => unmounted.forEach(fn => fn()),
    get mapCount() { return mapCount }, get removed() { return removed },
    get observersDisconnected() { return observer.disconnected && intersection.disconnected },
    viewport: visible => intersection.fn([{ isIntersecting: visible }]),
    update: (p = position()) => { robot.value = { map_position: p, last_odom_time: '2026-10-30T12:00:00+08:00' }; poseFresh.value = true },
  }
}
test('no key creates no tile requests or fabricated vehicle marker', () => {
  const f = fixture(); f.mount()
  assert.equal(f.tiles.length, 0); assert.equal(f.markers.length, 0)
  assert.equal(f.state.lastPosition.value, null)
  f.unmount()
})
test('fresh positions only move one marker and never recreate map or tiles', async () => {
  const f = fixture(key); f.mount(); f.update(); await flush()
  assert.equal(f.tiles.length, 2)
  const loadingTimers = f.timers.size
  f.update(position(117.020001)); await flush()
  assert.equal(f.mapCount, 1); assert.equal(f.tiles.length, 2); assert.equal(f.markers.length, 1)
  assert.equal(f.moves.length, 1, 'Small motion inside center does not pan map')
  assert.equal(f.markers[0].latlng[1], 117.020001)
  assert.equal(f.timers.size, loadingTimers)
  assert.equal(f.tiles[0].options.updateWhenIdle, false)
  assert.equal(f.tiles[0].options.updateWhenZooming, true)
  assert.equal(f.tiles[0].options.updateInterval, 200)
  assert.equal(f.icons.length, 1, 'Position changes alone must not rebuild the marker SVG')
  assert.equal(f.tiles[0].options.detectRetina, false)
  for (const subdomain of f.tiles[0].options.subdomains) {
    const host = new URL(f.tiles[0].url.replace('{s}', subdomain)).hostname
    assert.match(host, /^t[0-7]\.tianditu\.gov\.cn$/, 'Use the provider numeric hosts, not Leaflet a/b/c defaults')
  }
  f.unmount()
})
test('stale or invalid data keeps a grey last position instead of marking old data live', async () => {
  const f = fixture(); f.mount(); f.update(); await flush()
  f.poseFresh.value = false; await flush()
  assert.equal(f.state.fresh.value, false)
  assert.match(f.markers[0].icon.html, /stale/)
  assert.match(f.markers[0].tooltip, /最后位置/)
  f.update({ ...position(), longitude: NaN }); await flush()
  assert.equal(f.state.lastPosition.value.longitude, 117.02)
  assert.match(f.markers[0].icon.html, /stale/)
  f.unmount()
})
test('drag pauses follow, leaving center pans only with follow enabled, recenter is read-only', async () => {
  const f = fixture(); f.mount(); f.update(); await flush()
  f.mapEvents['dragstart boxzoomstart']()
  f.map.latLngToContainerPoint = () => ({ x: 900, y: 180 })
  f.update(position(117.021)); await flush()
  assert.equal(f.moves.length, 1)
  f.state.toggleFollow()
  assert.equal(f.state.following.value, true)
  assert.equal(f.moves.length, 2)
  f.update(position(117.022)); await flush()
  assert.equal(f.moves[2][0], 'pan')
  f.unmount()
})
test('key persists only in browser and switching basemaps replaces rather than stacks layers', async () => {
  const f = fixture(); f.mount(); f.state.setKey(key)
  assert.ok(f.tiles[0].url.includes('img_w/wmts'))
  assert.equal(f.storage.get('agri-robot.tianditu.web-key'), key)
  f.state.layerKind.value = 'street'; await flush()
  assert.equal(f.tiles.filter(tile => tile.added).length, 2)
  assert.ok(f.tiles[2].url.includes('vec_w/wmts'))
  assert.throws(() => f.state.setKey('invalid'), /有效/)
  f.state.setKey('')
  assert.equal(f.tiles.filter(tile => tile.added).length, 0)
  assert.equal(f.storage.size, 0)
  f.unmount()
})
test('tile failure stops all requests and ignores late events until explicit retry', () => {
  const f = fixture(key); f.mount()
  const late = f.tiles[1].events.tileerror
  f.tiles[0].events.tileerror()
  assert.match(f.state.tileError.value, /底图加载失败/)
  assert.equal(f.tiles.filter(tile => tile.added).length, 0)
  assert.equal(f.timers.size, 0)
  f.state.retryTiles()
  assert.equal(f.state.tileError.value, '')
  late()
  assert.equal(f.tiles.filter(tile => tile.added).length, 2)
  f.unmount()
})
test('stalled tile batch times out at eight seconds without an automatic retry loop', () => {
  const f = fixture(key); f.mount()
  assert.equal(f.timers.size, 1)
  const timer = [...f.timers.values()][0]
  assert.equal(timer.ms, 8000)
  timer.fn()
  assert.match(f.state.tileError.value, /超时/)
  assert.equal(f.tiles.length, 2)
  assert.equal(f.tiles.filter(tile => tile.added).length, 0)
  f.unmount()
})
test('hidden and out-of-view panels stop tiles; unmount releases map, observers, events and timers', async () => {
  const f = fixture(key); f.mount(); f.update(); await flush()
  f.doc.visibilityState = 'hidden'; f.events.visibilitychange(); await flush()
  assert.equal(f.tiles.filter(tile => tile.added).length, 0)
  f.doc.visibilityState = 'visible'; f.events.visibilitychange(); await flush()
  assert.equal(f.tiles.filter(tile => tile.added).length, 2)
  f.viewport(false); await flush()
  assert.equal(f.tiles.filter(tile => tile.added).length, 0)
  f.unmount()
  assert.equal(f.removed, true); assert.equal(f.observersDisconnected, true)
  assert.equal(f.timers.size, 0); assert.equal(f.events.visibilitychange, undefined)
})
test('workspace polling without GNSS never resets a manually chosen zoom or center', async () => {
  const f = fixture(); f.mount()
  f.mapEvents.zoomstart()
  for (let i = 0; i < 3; i++) {
    f.context.value = { origin: { latitude: 36.64, longitude: 117.02 } }
    await flush()
  }
  assert.equal(f.moves.length, 0)
  assert.equal(f.mapCount, 1)
  f.map.getZoom = () => 15
  f.update(); await flush()
  assert.equal(f.moves[0][2], 15, 'The first vehicle fix also preserves a user-selected zoom')
  f.unmount()
})
test('a late fixed origin sets the initial view only once, while a first vehicle fix can still be centered', async () => {
  const f = fixture(); f.context.value = null; f.mount()
  f.context.value = { origin: { latitude: 36.64, longitude: 117.02 } }; await flush()
  assert.equal(f.moves.length, 1)
  f.context.value = { origin: { latitude: 36.64, longitude: 117.02 } }; await flush()
  assert.equal(f.moves.length, 1)
  f.update(position(117.03)); await flush()
  assert.equal(f.moves.length, 2)
  assert.equal(f.moves[1][1][1], 117.03)
  f.unmount()
})
test('a late origin cannot override a view the user already dragged', async () => {
  const f = fixture(); f.context.value = null; f.mount()
  f.mapEvents['dragstart boxzoomstart']()
  f.context.value = { origin: { latitude: 36.64, longitude: 117.02 } }; await flush()
  assert.equal(f.moves.length, 0)
  assert.equal(f.state.following.value, false)
  f.unmount()
})
