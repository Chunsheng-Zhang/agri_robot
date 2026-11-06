// Execute the real channel against a deterministic fake WebSocket and clock.
const assert = require('node:assert/strict')
const { test } = require('node:test')
const fs = require('node:fs')
const path = require('node:path')
const root = path.resolve(__dirname, '..')
const sourceRoot = process.env.AGRI_MANUAL_TEST_ROOT || root
const ts = require(require.resolve('typescript', { paths: [path.join(root, 'agri_web')] }))

function channel(address = '/api/control/channel') {
  let now = 0
  const timers = [], sockets = [], failures = []
  class Socket {
    static OPEN = 1
    readyState = 0
    bufferedAmount = 0
    frames = []
    constructor(url) { this.url = String(url); sockets.push(this) }
    send(text) { this.frames.push(JSON.parse(text)) }
    close() { this.readyState = 3 }
    connect() { this.readyState = 1; this.onopen?.() }
    ack(seq) { this.onmessage?.({ data: JSON.stringify({ success: true, seq }) }) }
  }
  const code = ts.transpileModule(fs.readFileSync(path.join(sourceRoot,
    'agri_web/src/control/manualChannel.ts'), 'utf8'), {
    compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022 },
  }).outputText
  const api = new Function('exports', 'WebSocket', 'window', 'URL', 'setTimeout', 'clearTimeout',
    code + '; return exports')({}, Socket, { location: { href: 'http://robot.invalid:5173/monitor' } }, URL,
    (fn, ms) => { timers.push({ fn, at: now + ms, cleared: false }); return timers.length },
    id => { timers[id - 1].cleared = true })
  const connection = new api.ManualChannel(address)
  connection.onFailure = error => failures.push(error.message)
  const opened = connection.open(); opened.catch(() => {})
  return { connection, opened, socket: sockets[0], sockets, failures,
    advance(ms) {
      now += ms
      for (const t of timers) if (!t.cleared && t.at <= now) { t.cleared = true; t.fn() }
    } }
}
const frame = seq => ({ token: 'synthetic', seq, linear: .1, angular: 0, expires_at: 123.2 })

test('a 250ms acknowledgement is accepted and later frames do not wait for it', async () => {
  const s = channel(); s.socket.connect(); await s.opened
  const first = s.connection.send(frame(1)); s.advance(80)
  const second = s.connection.send(frame(2)); s.advance(80)
  const third = s.connection.send(frame(3)); s.advance(90)
  assert.equal(s.socket.frames.length, 3)
  s.socket.ack(3); await Promise.all([first, second, third])
  assert.deepEqual(s.failures, [])
  assert.ok(s.socket.frames.every(f => f.expires_at === 123.2), 'ACK must not refresh expiry')
  s.connection.close()
})

test('500ms without an acknowledgement closes once and never reconnects', async () => {
  const s = channel(); s.socket.connect(); await s.opened
  const pulse = s.connection.send(frame(1)); pulse.catch(() => {})
  s.advance(499); assert.equal(s.socket.readyState, 1)
  s.advance(1); await assert.rejects(pulse, /500 毫秒/)
  assert.equal(s.socket.readyState, 3)
  assert.equal(s.failures.length, 1)
  assert.equal(s.sockets.length, 1)
})

test('connection establishment has a 500ms bound without acquiring a lease', async () => {
  const s = channel(); s.advance(500)
  await assert.rejects(s.opened, /连接建立超过 500/)
  assert.equal(s.socket.frames.length, 0)
})

test('close rejects outstanding work and a late message cannot revive the channel', async () => {
  const s = channel(); s.socket.connect(); await s.opened
  const late = s.socket.onmessage
  const pulse = s.connection.send(frame(1)); pulse.catch(() => {})
  s.connection.close(); await assert.rejects(pulse, /会话已结束/)
  late({ data: '{"success":true,"seq":1}' })
  await assert.rejects(s.connection.send(frame(2)), /连接不可用/)
  assert.equal(s.sockets.length, 1)
})

test('buffer accumulation, impossible acknowledgements and server rejection all close', async () => {
  for (const action of [s => { s.socket.bufferedAmount = 2049; return s.connection.send(frame(1)) },
    s => { s.socket.onmessage({ data: '{"success":true,"seq":9}' }); return Promise.resolve() },
    s => { s.socket.onmessage({ data: '{"success":false,"detail":"遥控指令过期"}' }); return Promise.resolve() }]) {
    const s = channel(); s.socket.connect(); await s.opened
    await action(s).catch(() => {})
    assert.equal(s.socket.readyState, 3)
    assert.equal(s.failures.length, 1)
  }
})

test('HTTPS page uses WSS and tokens never enter the URL', async () => {
  const s = channel('https://robot.invalid:5173/api/control/channel')
  assert.equal(s.socket.url, 'wss://robot.invalid:5173/api/control/channel')
  s.socket.connect(); await s.opened; s.connection.close()
})

test('an expired-frame ACK keeps the channel open without requiring recovery', async () => {
  const s = channel(); s.socket.connect(); await s.opened
  const pulse = s.connection.send(frame(1))
  s.socket.onmessage({ data: JSON.stringify({ success: true, seq: 1, applied: false,
    reason: 'expired', overdue_ms: 15 }) })
  const reply = await pulse
  assert.equal(reply.applied, false)
  assert.equal(reply.reason, 'expired')
  assert.equal(reply.neutral_required, undefined)
  assert.equal(s.socket.readyState, 1)
  assert.deepEqual(s.failures, [])
  s.connection.close()
})
