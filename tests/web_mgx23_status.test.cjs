// Render the real status component without HTTP, ROS, a browser, or hardware.
const assert = require('node:assert/strict')
const { test } = require('node:test')
const fs = require('node:fs')
const path = require('node:path')
const deps = [path.resolve(__dirname, '../agri_web')]
const dependency = name => require(require.resolve(name, { paths: deps }))
const vue = dependency('vue')
const { renderToString } = dependency('@vue/server-renderer')
const { parse, compileScript } = dependency('@vue/compiler-sfc')
const ts = dependency('typescript')
const source = fs.readFileSync(path.resolve(__dirname, '../agri_web/src/components/dashboard/RobotStatus.vue'), 'utf8')
const { descriptor } = parse(source)
const script = compileScript(descriptor, { id: 'mgx23-status-test', inlineTemplate: true }).content
const code = ts.transpileModule(script, {
  compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022 },
}).outputText

function fixture() {
  const devices = vue.ref(null), transportFresh = vue.ref(true)
  const realtime = {
    robot: vue.ref(null), devices, poseFresh: vue.ref(false), velocityFresh: vue.ref(false),
    robotFresh: vue.ref(false), safetyFresh: vue.ref(false), refresh: async () => {},
    isFresh: value => transportFresh.value && value?.fresh === true && value.age_seconds < 1,
  }
  const modules = {
    vue, '@element-plus/icons-vue': dependency('@element-plus/icons-vue'),
    '../../api/workspace': { workModeText: mode => mode || '运行模式未更新' },
    '../../composables/useRealtime': {
      useRealtime: () => realtime, modeText: () => '',
      numberText: value => value == null ? '--' : String(value),
    },
  }
  const exports = {}
  new Function('require', 'exports', code)(name => {
    assert.ok(modules[name], name)
    return modules[name]
  }, exports)
  const render = async () => {
    const app = vue.createSSRApp(exports.default)
    for (const name of ['ElCard', 'ElTag', 'ElIcon', 'ElDivider', 'ElAlert', 'ElProgress']) {
      app.component(name, { props: ['type'], setup: (props, { slots, attrs }) => () =>
        vue.h('div', { ...attrs, 'data-type': props.type }, [slots.header?.(), slots.default?.()]) })
    }
    return (await renderToString(app)).replace(/<!--[\s\S]*?-->/g, '')
  }
  const update = (initialized, active, gnssActive = [false, false]) => {
    devices.value = { mgx23: { fresh: true, age_seconds: 0.05, initialized, status_word: 0x001c0002,
      flags: ['滤波器收敛', '收到 PPS 信号', '时间同步成功', '解析到 RTK 数据'].map((label, index) =>
        ({ name: String(index), label, active: active[index] })),
      gnss_flags: ['GNSS 数据差', 'GNSS 数据无效'].map((label, index) =>
        ({ name: String(index), label, active: gnssActive[index] })) } }
  }
  return { devices, transportFresh, render, update }
}

test('GNSS panel shows four initialization flags and two unknown error flags', async () => {
  const f = fixture(), html = await f.render()
  assert.match(html, /MGX23 初始化状态/)
  assert.doesNotMatch(html, /角速度（rad\/s）/)
  assert.doesNotMatch(html, /初始化成功/)
  for (const label of ['滤波器收敛', '收到 PPS 信号', '时间同步成功', '解析到 RTK 数据']) assert.ok(html.includes(label))
  assert.ok(html.includes('GNSS 数据差') && html.includes('GNSS 数据无效'))
  assert.equal((html.match(/未更新/g) || []).length, 11) // Work mode, authority, pose, speed, summary, four init and two GNSS flags.
})

test('each missing condition is visible, and all four satisfied show success', async () => {
  const f = fixture()
  for (let missing = 0; missing < 4; missing++) {
    f.update(false, [0, 1, 2, 3].map(index => index !== missing))
    const html = await f.render()
    assert.match(html, /未初始化完成/)
    assert.equal((html.match(/未满足/g) || []).length, 1)
    assert.equal((html.match(/已满足/g) || []).length, 3)
  }
  f.update(true, [true, true, true, true])
  const html = await f.render()
  assert.match(html, /初始化成功/)
  assert.equal((html.match(/已满足/g) || []).length, 4)
})

test('stale source, delayed snapshot or lost HTTP freshness clears previous green state', async () => {
  const f = fixture()
  for (const expire of [
    () => { f.devices.value.mgx23.fresh = false },
    () => { f.devices.value.mgx23.age_seconds = 1.1 },
    () => { f.transportFresh.value = false },
  ]) {
    f.transportFresh.value = true
    f.update(true, [true, true, true, true], [true, true])
    expire()
    const html = await f.render()
    assert.doesNotMatch(html, /初始化成功|已满足/)
    assert.equal((html.match(/未更新/g) || []).length, 11)
    for (const label of ['GNSS 数据差', 'GNSS 数据无效']) {
      assert.match(html, new RegExp(`${label}</span>\\s*<div[^>]*data-type="info"[^>]*>未更新</div>`))
    }
  }
})

test('GNSS error bits render independently with red set flags and preserve initialization success', async () => {
  const f = fixture()
  for (let bits = 0; bits < 4; bits++) {
    const active = [Boolean(bits & 1), Boolean(bits & 2)]
    f.update(true, [true, true, true, true], active)
    const html = await f.render()
    assert.match(html, /初始化成功/)
    for (const [index, label] of ['GNSS 数据差', 'GNSS 数据无效'].entries()) {
      assert.match(html, new RegExp(`${label}</span>\\s*<div[^>]*data-type="${active[index] ? 'danger' : 'info'}"[^>]*>${active[index] ? '已置位' : '未置位'}</div>`))
    }
    assert.match(html, /未置位不代表卫星信号良好或 RTK 固定解/)
  }
})

test('an older backend without GNSS error flags displays unknown instead of no error', async () => {
  const f = fixture()
  f.update(true, [true, true, true, true])
  delete f.devices.value.mgx23.gnss_flags
  const html = await f.render()
  for (const label of ['GNSS 数据差', 'GNSS 数据无效']) {
    assert.match(html, new RegExp(`${label}</span>\\s*<div[^>]*data-type="info"[^>]*>未更新</div>`))
  }
})
