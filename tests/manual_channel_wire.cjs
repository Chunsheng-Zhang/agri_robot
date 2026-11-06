// Invoked only by test_manual_channel.py with a loopback fake backend.
const assert = require('node:assert/strict')
const fs = require('node:fs')
const path = require('node:path')
const http = require('node:http')
const { pathToFileURL } = require('node:url')
const root = path.resolve(__dirname, '..'), web = path.join(root, 'agri_web')
const sourceRoot = process.env.AGRI_MANUAL_TEST_ROOT || root
const ts = require(require.resolve('typescript', { paths: [web] }))
const target = process.argv[2]
assert.match(target, /^http:\/\/127\.0\.0\.1:\d+$/)
function execute(source, requireMock, globals = {}) {
  const code = ts.transpileModule(source, {
    compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022 },
  }).outputText
  return new Function('require', 'exports', ...Object.keys(globals), code + '; return exports')(
    requireMock, {}, ...Object.values(globals))
}

async function main() {
  const configFile = path.join(sourceRoot, 'agri_web/vite.config.ts')
  const ports = {
    vite: { defineConfig: fn => fn, loadEnv: () => ({ AGRI_BACKEND_URL: target, AGRI_VIDEO_URL: target }) },
    '@vitejs/plugin-vue': { __esModule: true, default: () => ({ name: 'synthetic-vue-port' }) },
    'node:url': require('node:url'), 'node:http': http, 'node:https': require('node:https'),
  }
  const config = execute(fs.readFileSync(configFile, 'utf8')
    .replaceAll('import.meta.url', JSON.stringify(pathToFileURL(path.join(web, 'vite.config.ts')).href)),
    name => { assert.ok(ports[name], name); return ports[name] }).default({ mode: 'test', command: 'serve' })
  assert.equal(config.server.proxy['/api'].ws, true)
  const cache = fs.mkdtempSync(path.join(root, '.diagnostics/manual-wire-'))
  let vite, channel, pulse
  try {
    const { createServer } = await import(pathToFileURL(require.resolve('vite', { paths: [web] })).href)
    vite = await createServer({ ...config, configFile: false, root: web, cacheDir: cache,
      logLevel: 'silent', optimizeDeps: { noDiscovery: true, include: [] },
      server: { ...config.server, host: '127.0.0.1', port: 0, watch: null, hmr: false },
    })
    await vite.listen()
    const origin = `http://127.0.0.1:${vite.httpServer.address().port}`
    // Delay only ACK delivery to the frontend; keep sending/receiving frames.
    // This models slow return traffic without delaying the command receiver.
    class DelayedSocket {
      static OPEN = 1
      constructor(url) { this.ws = new WebSocket(url) }
      get readyState() { return this.ws.readyState }
      get bufferedAmount() { return this.ws.bufferedAmount }
      set onopen(fn) { this.ws.onopen = fn }
      set onclose(fn) { this.ws.onclose = fn }
      set onerror(fn) { this.ws.onerror = fn }
      set onmessage(fn) { this.ws.onmessage = fn ? event => setTimeout(() => fn(event), 250) : null }
      send(data) { this.ws.send(data) }
      close() { this.ws.close() }
    }
    const { ManualChannel } = execute(fs.readFileSync(path.join(sourceRoot,
      'agri_web/src/control/manualChannel.ts'), 'utf8'), name => assert.fail(name),
      { WebSocket: DelayedSocket, window: { location: { href: origin } } })
    channel = new ManualChannel(origin + '/api/control/channel')
    const failures = []; channel.onFailure = error => failures.push(error.message)
    await channel.open()
    const response = await fetch(origin + '/api/control/arm', { method: 'POST' })
    assert.equal(response.status, 200)
    const session = await response.json(); assert.equal(session.heartbeat_protocol, 4)
    const offset = session.server_time - Date.now()/1000
    let seq = 0
    const send = linear => channel.send({ token: session.token, seq: ++seq, linear, angular: 0,
      expires_at: Date.now()/1000 + offset + .2 })
    await send(0)
    const pending = []
    pulse = setInterval(() => { const p = send(.15); p.catch(() => {}); pending.push(p) }, 80)
    const snapshots = []
    for (let i = 0; i < 2; i++) {
      const r = await fetch(origin + '/video/snapshot?topic=/camera/color/image_raw&width=320&height=240&quality=40')
      assert.equal(r.status, 200); assert.equal(r.headers.get('content-type'), 'image/jpeg')
      snapshots.push((await r.arrayBuffer()).byteLength)
      await new Promise(resolve => setTimeout(resolve, 300))
    }
    clearInterval(pulse); pulse = null
    await send(0); await Promise.all(pending)
    assert.ok(pending.length >= 5)
    assert.ok(snapshots.every(n => n > 0))
    assert.deepEqual(failures, [])
    channel.close()
    console.log(JSON.stringify({ synthetic: true, acknowledgement_delay_ms: 250,
      frames: seq, camera_frames: snapshots.length, failures: failures.length }))
  } finally {
    clearInterval(pulse); channel?.close()
    config.server.proxy['/api'].agent.destroy()
    if (vite) await vite.close()
    fs.rmSync(cache, { recursive: true, force: true })
  }
}
main().catch(error => { console.error(error); process.exitCode = 1 })
