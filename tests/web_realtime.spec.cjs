// Isolated browser validation: ALL /api and /video requests are intercepted.
// Run with a local Vite server and Playwright installed outside the robot runtime.
const assert = require('node:assert/strict')
const fs = require('node:fs')
const path = require('node:path')
const { chromium } = require(process.env.PLAYWRIGHT_MODULE_PATH || 'playwright')
const base = process.env.AGRI_WEB_TEST_URL || 'http://127.0.0.1:15173'
if (!['127.0.0.1', 'localhost'].includes(new URL(base).hostname)) throw Error('Use a local test server')

;(async () => {
  const browser = await chromium.launch({ headless: true,
    executablePath: process.env.AGRI_BROWSER_EXECUTABLE || undefined,
    args: process.env.AGRI_BROWSER_EXECUTABLE ? ['--disable-gpu'] : [],
  })
  try {
    const page = await browser.newPage({ viewport: { width: 1440, height: 1100 } })
    const artifacts = path.resolve(__dirname, '../.diagnostics/location-map-browser')
    fs.mkdirSync(artifacts, { recursive: true })
    const errors = []
    page.on('pageerror', error => errors.push(error.message))
    let live = false, velocityLive = true, disconnected = false, locked = false, camera = true, badVideo = false, frame = 'map', x = 12.34
    let lidarEnabled = false, rejectLidarChange = false
    const fresh = () => ({ fresh: live, age_seconds: live ? 0.05 : null })
    await page.route(url => url.pathname.startsWith('/api/'), async route => {
      const path = new URL(route.request().url()).pathname
      if (!path.startsWith('/api/')) return route.continue()
      const json = data => route.fulfill({ json: data })
      if (disconnected) return route.fulfill({ status: 503, json: { detail: '后端测试断连' } })
      if (path === '/api/workspace') return json({ origin: { latitude: 31, longitude: 118, epsg: 32650 },
        origin_source: 'active_ros', defaults: { waypoint: {}, coverage: {} }, missions: [], runs: [], active_run: null, events: [] })
      if (path === '/api/robot/status') return json({ robot_id: 'test_robot', ros_connected: live, online: live,
        speed: velocityLive ? 0.37 : null, speed_source: velocityLive ? 'mgx23_ned' : null,
        velocity_age_seconds: velocityLive ? 0.05 : 2,
        angular_speed: -0.12, heading: 45, battery: null, position_x: x, position_y: 3.21,
        map_position: { latitude: 31.2345, longitude: 118.987 + x / 100000, bearing: 45,
          reference: 'base_link', source: 'mgx23_localization', crs: 'EPSG:4326' },
        odom_age_seconds: live ? 0.05 : null, last_odom_time: new Date().toISOString(),
        reference_path_points: 3, tracked_path_points: 2 })
      if (path === '/api/robot/lidar-protection') {
        if (rejectLidarChange) return route.fulfill({ status: 409, json: { detail: '测试雷达切换被拒绝' } })
        lidarEnabled = route.request().postDataJSON().enabled
        return json({ success: true, enabled: lidarEnabled, message: lidarEnabled ? '雷达障碍停车已启用' : '雷达障碍停车已关闭' })
      }
      if (path === '/api/devices') return json({
        safety: { ...fresh(), mode: 'coverage', mode_revision: 1, owner: 'tracking', emergency_stop: locked, obstacle_stop: false,
          motion_inhibited: locked, localization_valid: true, joy_connected: true, lidar_connected: true,
          lidar_required: lidarEnabled, reason: 'test feedback' },
        obstacles: { ...fresh(), valid: true, global_range: 1.45, front_distance: 2.34, rear_distance: 3.45, front_valid: true, rear_valid: true },
        manual: { ...fresh(), connected: true },
        camera: { ...fresh(), connected: camera, width: 640, height: 480, encoding: 'rgb8' },
        waypoint: { ...fresh(), state: 'IDLE', count: 3, index: 0 },
        tracking: { ...fresh(), state: 'RUNNING', progress: 2, length: 10 }, planning: { ...fresh(), state: 'SUCCEEDED' },
        gnss: { ...fresh(), latitude: 31.23456789, longitude: 118.98765432 },
      })
      if (path.endsWith('/path/reference')) return json({ count: 3, frame_id: frame, points: [{ x: 0, y: 0 }, { x: 5, y: 0 }, { x: 5, y: 5 }] })
      if (path.endsWith('/path/tracked')) return json({ count: 2, frame_id: 'map', points: [{ x: 0, y: 0 }, { x: 1, y: 1 }] })
      if (path === '/api/robot/waypoints') return json({ count: 0, frame_id: 'map', points: [] })
      if (path === '/api/camera/stream-info') return json({ stream_url: '/video/stream?topic=test' })
      if (path.endsWith('/emergency-stop/reset')) return route.fulfill({ status: 409, json: { detail: '测试拒绝复位：按钮仍按下' } })
      throw Error(`Unexpected API request: ${path}`)
    })
    await page.route('**/video/**', route => badVideo
      ? route.fulfill({ status: 503 })
      : route.fulfill({ contentType: 'image/jpeg', body: fs.readFileSync(path.join(__dirname, 'fixtures/camera_preview.jpg')) }))
    await page.route(url => url.hostname.endsWith('.tianditu.gov.cn'), route => {
      assert.match(new URL(route.request().url()).hostname, /^t[0-7]\.tianditu\.gov\.cn$/)
      return route.fulfill({ contentType: 'image/jpeg', body: fs.readFileSync(path.join(__dirname, 'fixtures/camera_preview.jpg')) })
    })
    const wait = predicate => page.waitForFunction(predicate, null, { timeout: 12000 })
    await page.goto(base, { waitUntil: 'commit' })
    await wait(() => document.body.innerText.includes('安全状态未知'))
    const lidarControl = page.getByRole('checkbox', { name: '启用雷达障碍停车', exact: true })
    assert(await lidarControl.isDisabled())
    assert(await page.getByRole('button', { name: '解除急停', exact: true }).first().isDisabled())
    assert.equal(await page.locator('.robot-map-pin').count(), 0)
    await page.locator('.location-card').scrollIntoViewIfNeeded()
    const initialScale = await page.locator('.leaflet-control-scale-line').innerText()
    await page.locator('.leaflet-control-zoom-out').click()
    await page.waitForTimeout(500)
    const chosenScale = await page.locator('.leaflet-control-scale-line').innerText()
    assert.notEqual(chosenScale, initialScale)
    await page.waitForTimeout(4500)
    assert.equal(await page.locator('.leaflet-control-scale-line').innerText(), chosenScale,
      'Workspace polling without GNSS must preserve the user zoom')
    live = true
    await wait(() => document.body.innerText.includes('118.98765432'))
    await wait(() => document.querySelector('.robot-map-pin.live') !== null)
    await page.locator('.map-buttons').getByRole('button', { name: '配置底图', exact: true }).click()
    await page.getByLabel('天地图服务 Key', { exact: true }).fill('a'.repeat(32))
    await page.getByRole('button', { name: '保存并加载', exact: true }).click()
    await wait(() => document.querySelector('.location-canvas img.leaflet-tile-loaded') !== null)
    await page.locator('.location-card').screenshot({ path: path.join(artifacts, 'desktop-synthetic.png') })
    await page.getByRole('button', { name: '展开地图', exact: true }).click()
    await wait(() => document.querySelector('.location-card.expanded') !== null)
    assert(await page.locator('.topbar .emergency-controls').isVisible(), 'Expanded map must keep emergency controls visible')
    assert(await page.locator('.topbar .emergency-controls').evaluate(element => {
      const rect = element.getBoundingClientRect(), top = document.elementFromPoint(rect.x + rect.width / 2, rect.y + rect.height / 2)
      return top && element.contains(top)
    }), 'Expanded map must not cover emergency controls')
    await page.screenshot({ path: path.join(artifacts, 'expanded-synthetic.png') })
    await page.keyboard.press('Escape')
    await page.locator('.map-toolbar').getByText('普通地图', { exact: true }).click()
    await wait(() => [...document.querySelectorAll('.location-canvas img.leaflet-tile-loaded')].some(image => image.src.includes('vec_w')))
    await page.setViewportSize({ width: 390, height: 844 })
    await page.locator('.location-card').scrollIntoViewIfNeeded()
    assert(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth + 2), 'Mobile layout must not overflow horizontally')
    const canvasSize = await page.locator('.location-canvas').boundingBox()
    assert(canvasSize.height >= 240 && canvasSize.height <= 360)
    await page.locator('.location-card').screenshot({ path: path.join(artifacts, 'mobile-synthetic.png') })
    await page.setViewportSize({ width: 1440, height: 1100 })
    assert(!(await lidarControl.isChecked()))
    await page.locator('.lidar-control .el-checkbox').click()
    await wait(() => document.querySelector('.lidar-control input')?.checked === true)
    rejectLidarChange = true
    await page.locator('.lidar-control .el-checkbox').click()
    await wait(() => document.body.innerText.includes('测试雷达切换被拒绝'))
    await wait(() => document.querySelector('.lidar-control input')?.checked === true)
    rejectLidarChange = false
    await page.locator('.lidar-control .el-checkbox').click()
    await wait(() => document.querySelector('.lidar-control input')?.checked === false)
    assert((await page.locator('.robot-card').innerText()).includes('0.37'))
    assert((await page.locator('.robot-card').innerText()).includes('MGX23 水平速度'))
    velocityLive = false
    await wait(() => !document.querySelector('.robot-card').innerText.includes('0.37'))
    assert((await page.locator('.robot-card').innerText()).includes('未更新'))
    velocityLive = true
    await wait(() => document.querySelector('.robot-card').innerText.includes('0.37'))
    assert((await page.locator('.safety-panel').innerText()).includes('2.34'))
    assert((await page.locator('.task-section').innerText()).includes('运行中'))
    assert.equal(await page.locator('.video-frame img').count(), 0)
    await page.getByRole('button', { name: '开启画面', exact: true }).click()
    await wait(() => document.querySelector('.video-frame img')?.naturalWidth > 0)
    x = 15.67
    await wait(() => document.querySelector('.robot-card').innerText.includes('15.67'))
    locked = true
    await wait(() => document.body.innerText.includes('急停锁定'))
    await page.getByRole('button', { name: '解除急停', exact: true }).first().click()
    await page.locator('.el-message-box').getByRole('button', { name: '确认复位', exact: true }).click()
    await wait(() => document.body.innerText.includes('测试拒绝复位'))
    assert((await page.locator('.emergency-controls').first().innerText()).includes('急停锁定'))
    camera = false
    await wait(() => document.querySelector('.video-frame img') === null)
    camera = true; badVideo = true
    await wait(() => document.body.innerText.includes('画面请求失败（HTTP 503）'))
    disconnected = true
    await wait(() => document.body.innerText.includes('安全状态未知') && !document.body.innerText.includes('118.98765432'))
    assert(await page.getByRole('button', { name: '解除急停', exact: true }).first().isDisabled())
    assert(await lidarControl.isDisabled())
    await page.locator('.location-card').scrollIntoViewIfNeeded()
    await wait(() => document.querySelector('.robot-map-pin.stale') !== null)
    assert.equal(await page.locator('.robot-map-pin.live').count(), 0)
    assert.equal(await page.locator('.video-frame img').count(), 0)
    assert(!(await page.locator('.robot-card').innerText()).includes('15.67'))
    disconnected = false
    await page.goto(`${base}/monitor`)
    frame = 'odom'
    await wait(() => document.body.innerText.includes('路径坐标系不是 map'))
    assert.equal(await page.locator('.field-card polyline').first().getAttribute('points'), '')
    assert.deepEqual(errors, [])
    console.log('PASS: live polling, position/path, device/task states, unknown/stale data, rejected reset, camera disconnection and stream error')
  } finally { await browser.close() }
})().catch(error => { console.error(error); process.exitCode = 1 })
