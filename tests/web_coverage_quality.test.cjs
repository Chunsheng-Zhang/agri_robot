// Render the real coverage-quality panel; no HTTP, ROS or real vehicle.
const assert = require('node:assert/strict')
const { test } = require('node:test')
const fs = require('node:fs'), path = require('node:path')
const deps = [path.resolve(__dirname, '../agri_web')]
const dependency = name => require(require.resolve(name, { paths: deps }))
const vue = dependency('vue'), ts = dependency('typescript')
const { parse, compileScript } = dependency('@vue/compiler-sfc')
const { renderToString } = dependency('@vue/server-renderer')
const file = path.resolve(__dirname, '../agri_web/src/components/tasks/CoverageQuality.vue')
const { descriptor } = parse(fs.readFileSync(file, 'utf8'))
const code = ts.transpileModule(compileScript(descriptor, { id: 'quality', inlineTemplate: true }).content,
  { compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022 } }).outputText
const modules = {
  vue,
  './OptimizationResults.vue': {default:{render:()=>null}},
  '../../api/workspace': { coverageQualityUrl: (id, artifact) => `/api/runs/${id}/coverage-quality/${artifact}` },
  '../../composables/useRealtime': { numberText: (value, digits = 2) =>
    typeof value === 'number' && Number.isFinite(value) ? value.toFixed(digits) : '--' },
}
const exportsObject = {}
new Function('require', 'exports', code)(name => { assert.ok(modules[name], name); return modules[name] }, exportsObject)

async function render(run, parameters = false) {
  const app = vue.createSSRApp({ render: () => vue.h(exportsObject.default, { run },
    parameters ? { parameters: () => vue.h('form', { 'data-quality-config': true }, '评价幅宽和网格配置') } : {}) })
  for (const name of ['ElCard', 'ElAlert', 'ElRadioGroup', 'ElRadioButton', 'ElSelect', 'ElOption', 'ElProgress']) {
    app.component(name, { props: ['title', 'percentage'], setup: (props, { attrs, slots }) => () =>
      vue.h('div', { ...attrs, 'data-percentage': props.percentage }, [props.title, slots.header?.(), slots.default?.()]) })
  }
  return renderToString(app)
}
const region = { available: true, area_m2: 100, covered_m2: 75, missed_m2: 25, overlap_m2: 10,
  coverage_percent: 75, overlap_percent: 10, miss_percent: 25 }
const ready = () => ({ id: 'synthetic-run', name: '地块', mission_revision: 3, coverage_quality_version: 1,
  coverage_quality: { state: 'SUCCEEDED', planned: { available: true, width_m: .8, resolution_m: .05,
    metrics: { field: region, interior: region, headland: region } } } })

test('no round or legacy round shows no fabricated coverage percentages', async () => {
  for (const run of [null, { id: 'legacy', name: '旧任务' }]) {
    const html = await render(run)
    assert.match(html, /生成覆盖路径后显示/)
    assert.doesNotMatch(html, /75\.00%|100\.00%/)
  }
})
test('planning metrics show coverage overlap misses and round-specific image', async () => {
  const html = await render(ready())
  for (const text of ['75.00%', '10.00%', '25.00%', '0.800', '0.050']) assert.ok(html.includes(text), text)
  assert.match(html, /synthetic-run\/coverage-quality\/plan.png/)
  assert.match(html, /包括连接转弯/)
  assert.match(html, /整田指标包含保留地头/)
  assert.doesNotMatch(html, /期刊参考文献|\.bib/)
})
test('pending and failed evaluation report their status instead of zero error', async () => {
  const run = ready()
  run.coverage_quality = { state: 'PROCESSING' }
  assert.match(await render(run), /后台计算作业质量/)
  run.coverage_quality = { state: 'FAILED', reason: '合成评价失败' }
  const html = await render(run)
  assert.match(html, /合成评价失败/)
  assert.doesNotMatch(html, /0\.00%/)
})
test('ended round with missing actual trajectory reports unknown and exposes report', async () => {
  const run = ready()
  run.coverage_quality.actual = { available: false, reason: '无正式跟踪样本' }
  const html = await render(run)
  assert.match(html, /无正式跟踪样本/)
  assert.match(html, /coverage-quality\/report.md/)
  assert.match(html, /coverage-quality\/metrics.json/)
})
test('evaluation configuration remains available before planning, during calculation and after failure', async () => {
  const pending = ready(); pending.coverage_quality = { state: 'PROCESSING' }
  const failed = ready(); failed.coverage_quality = { state: 'FAILED', reason: '评价失败' }
  for (const run of [null, pending, failed, ready()]) {
    const html = await render(run, true)
    assert.match(html, /规划作业质量评估/)
    assert.match(html, /data-quality-config/)
    assert.match(html, /评价幅宽和网格配置/)
  }
})
