// Real loopback HTTP/Vite proxy regression with a synthetic upstream only.
// Never connects to ROS, the project backend, a camera, or a control lease.
const assert = require('node:assert/strict')
const { test } = require('node:test')
const fs = require('node:fs')
const path = require('node:path')
const http = require('node:http')
const https = require('node:https')
const { pathToFileURL } = require('node:url')
const root = path.resolve(__dirname, '..')
const web = path.join(root, 'agri_web')
const ts = require(require.resolve('typescript', { paths: [web] }))

function configuration(env = {}) {
  const file = path.join(web, 'vite.config.ts')
  const source = fs.readFileSync(file, 'utf8')
    .replaceAll('import.meta.url', JSON.stringify(pathToFileURL(path.join(web, 'vite.config.ts')).href))
  const code = ts.transpileModule(source, {
    compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022 },
  }).outputText
  const ports = {
    vite: { defineConfig: value => value, loadEnv: () => env },
    '@vitejs/plugin-vue': { __esModule: true, default: () => ({ name: 'synthetic-vue-port' }) },
    'node:url': require('node:url'), 'node:http': http, 'node:https': https,
  }
  const requireMock = name => {
    assert.ok(Object.hasOwn(ports, name), `Unexpected config dependency ${name}`)
    return ports[name]
  }
  return new Function('require', 'exports', code + '; return exports.default')(
    requireMock, {})({ mode: 'test', command: 'serve' })
}

function closeAgents(config) {
  const agents = new Set(Object.values(config.server.proxy).map(p => p.agent).filter(Boolean))
  for (const agent of agents) agent.destroy()
}

test('API uses keep-alive for dev and preview, independently of the camera proxy', () => {
  const config = configuration()
  try {
    const api = config.server.proxy['/api'], video = config.server.proxy['/video']
    assert.ok(api.agent instanceof http.Agent)
    assert.equal(api.agent.options.keepAlive, true)
    assert.equal(api.target, 'http://127.0.0.1:8000')
    assert.equal(config.preview.proxy['/api'].agent, api.agent)
    assert.notEqual(video.agent, api.agent)
    assert.equal(video.target, 'http://127.0.0.1:8080')
    assert.equal(video.rewrite('/video/snapshot?topic=/camera/color/image_raw'),
      '/snapshot?topic=/camera/color/image_raw')
  } finally { closeAgents(config) }
})

test('HTTPS backend keeps TLS certificate validation and uses an HTTPS agent', () => {
  const config = configuration({ AGRI_BACKEND_URL: 'https://backend.invalid:8443' })
  try {
    const api = config.server.proxy['/api']
    assert.ok(api.agent instanceof https.Agent)
    assert.equal(api.agent.options.keepAlive, true)
    assert.notEqual(api.secure, false)
    assert.notEqual(api.agent.options.rejectUnauthorized, false)
  } finally { closeAgents(config) }
})

test('real Vite API proxy reuses both TCP connections while a synthetic camera responds', async () => {
  const upstreamConnections = new Set(), requests = []
  const upstream = http.createServer((req, res) => {
    requests.push(req.url)
    if (req.url.startsWith('/api/')) upstreamConnections.add(req.socket)
    req.resume()
    req.on('end', () => {
      if (req.url.startsWith('/snapshot')) {
        res.setHeader('Content-Type', 'image/jpeg')
        return setTimeout(() => res.end(fs.readFileSync(path.join(__dirname, 'fixtures/camera_preview.jpg'))), 100)
      }
      res.setHeader('Content-Type', 'application/json')
      res.end('{"success":true}')
    })
  })
  await new Promise(resolve => upstream.listen(0, '127.0.0.1', resolve))
  let config
  const agent = new http.Agent({ keepAlive: true, maxSockets: 1 })
  const directory = path.join(root, '.diagnostics')
  fs.mkdirSync(directory, { recursive: true })
  const cache = fs.mkdtempSync(path.join(directory, 'proxy-keepalive-test-'))
  let vite
  try {
    const target = `http://127.0.0.1:${upstream.address().port}`
    config = configuration({ AGRI_BACKEND_URL: target, AGRI_VIDEO_URL: target })
    const { createServer } = await import(pathToFileURL(require.resolve('vite', { paths: [web] })).href)
    vite = await createServer({ ...config, configFile: false, root: web, cacheDir: cache,
      logLevel: 'silent', clearScreen: false,
      optimizeDeps: { noDiscovery: true, include: [] },
      server: { ...config.server, host: '127.0.0.1', port: 0, strictPort: false, watch: null, hmr: false },
    })
    await vite.listen()
    const snapshot = new Promise((resolve, reject) => {
      const req = http.get({ host: '127.0.0.1', port: vite.httpServer.address().port,
        path: '/video/snapshot?topic=/camera/color/image_raw&width=320&height=240&quality=40',
        agent: false, timeout: 2000,
      }, res => {
        let bytes = 0
        res.on('data', chunk => { bytes += chunk.length })
        res.on('end', () => resolve({ status: res.statusCode, type: res.headers['content-type'], bytes }))
      })
      req.on('timeout', () => req.destroy(new Error('Synthetic camera timed out')))
      req.on('error', reject)
    })
    // Observe a failure immediately, but rethrow it when checking the camera below.
    snapshot.catch(() => {})
    const downstreamPorts = new Set()
    for (let i = 0; i < 8; i++) {
      const result = await new Promise((resolve, reject) => {
        const req = http.request({ host: '127.0.0.1', port: vite.httpServer.address().port,
          path: '/api/control/command', method: 'POST', agent,
          headers: { 'Content-Type': 'application/json' }, timeout: 2000,
        }, res => {
          const connection = res.headers.connection
          downstreamPorts.add(res.socket.localPort)
          let body = ''
          res.setEncoding('utf8'); res.on('data', chunk => { body += chunk })
          res.on('end', () => resolve({ status: res.statusCode, connection, body }))
        })
        req.on('timeout', () => req.destroy(new Error('Synthetic proxy request timed out')))
        req.on('error', reject)
        req.end('{}')
      })
      assert.equal(result.status, 200)
      assert.notEqual(result.connection, 'close')
      assert.deepEqual(JSON.parse(result.body), { success: true })
    }
    assert.equal(downstreamPorts.size, 1, 'Browser-to-Vite connection must be reused')
    assert.equal(upstreamConnections.size, 1, 'Vite-to-backend connection must be reused')
    assert.equal(requests.filter(p => p === '/api/control/command').length, 8)
    const image = await snapshot
    assert.equal(image.status, 200)
    assert.equal(image.type, 'image/jpeg')
    assert.ok(image.bytes > 0)
    assert.ok(requests.includes('/snapshot?topic=/camera/color/image_raw&width=320&height=240&quality=40'))
  } finally {
    agent.destroy(); if (config) closeAgents(config)
    if (vite) await vite.close()
    upstream.closeAllConnections()
    await new Promise(resolve => upstream.close(resolve))
    fs.rmSync(cache, { recursive: true, force: true })
  }
})
