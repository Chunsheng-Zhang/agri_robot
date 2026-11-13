// Render the actual optimizer result panel, with no browser/HTTP/ROS connection.
const assert = require('node:assert/strict')
const { test } = require('node:test')
const fs = require('node:fs'), path = require('node:path')
const deps = [path.resolve(__dirname, '../agri_web')]
const dependency = name => require(require.resolve(name, { paths: deps }))
const vue = dependency('vue'), ts = dependency('typescript')
const { parse, compileScript } = dependency('@vue/compiler-sfc')
const { renderToString } = dependency('@vue/server-renderer')
const file = path.resolve(__dirname, '../agri_web/src/components/tasks/OptimizationResults.vue')
const { descriptor } = parse(fs.readFileSync(file, 'utf8'))
const code = ts.transpileModule(compileScript(descriptor, { id: 'smart', inlineTemplate: true }).content,
  { compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022 } }).outputText
const modules = { vue, '../../api/workspace': { optimizationReportUrl: id => `/api/runs/${id}/optimization` },
  '../../composables/useRealtime': { numberText: (v,d = 2) => typeof v === 'number' && Number.isFinite(v) ? v.toFixed(d) : '--' } }
const out = {}
new Function('require', 'exports', code)(name => { assert.ok(modules[name], name); return modules[name] }, out)
async function render(optimization) {
  const app = vue.createSSRApp({render:()=>vue.h(out.default,{run:{id:'synthetic',optimization}})})
  app.component('ElAlert',{props:['title'],setup:p=>()=>vue.h('p',p.title)})
  return renderToString(app)
}
const report = () => ({state:'SUCCEEDED',selected:{id:1,mode:'SNAKE',headland_width_m:3,
  angle_rad:0,length_m:240,energy_proxy:280,coverage_percent:96,overlap_percent:15},
  coverage_constraint_enabled:false,evaluated_count:24,possible_count:28,budget_exhausted:true,elapsed_seconds:3,
  search_resolution_m:.1,final_resolution_m:.05,candidates:[{id:2,mode:'CONTOUR',headland_width_m:1,
    angle_rad:0,reason:'完整路径车体净空不足'}],curve_type:'REEDS_SHEPP',allow_reverse:false,
    turn_curves:[{type:'REEDS_SHEPP',radius_m:1,
      metrics:{length_m:3,reverse_m:1,energy_proxy:5},executable:false,reason:'原生曲线包含倒车段，请勾选允许倒车后重新规划'}]})
test('pending optimizer does not fabricate metrics or executable turns',async()=>{
  const html=await render(undefined)
  assert.match(html,/正在生成、校验并比较候选/)
  assert.doesNotMatch(html,/96\.00%|280\.00/)
})
test('selected plan exposes effective width constraints proxy costs and rejected cases',async()=>{
  const html=await render(report())
  for(const text of ['蛇形','3.000 m','96.00%','280.00','非 Wh','不保证全局最优','完整路径车体净空不足',
    'Reeds–Shepp','本轮不允许倒车','/api/runs/synthetic/optimization']) assert.ok(html.includes(text),text)
  assert.doesNotMatch(html,/U／Ω|鱼尾|转弯候选库/)
})
test('reverse-enabled report states permission and retains the native curve name',async()=>{
  const r=report();r.allow_reverse=true;r.turn_curves[0].executable=true;r.turn_curves[0].reason=''
  const html=await render(r)
  assert.match(html,/本轮允许倒车，换向点停车对齐后继续/)
  assert.match(html,/Reeds–Shepp/)
  assert.doesNotMatch(html,/本轮不允许倒车|鱼尾/)
})
test('infeasible geometry shows failure without coverage rejection text',async()=>{
  const r=report();delete r.selected;r.state='FAILED';r.reason='没有满足净空和前进执行要求的完整候选'
  const html=await render(r)
  assert.match(html,/没有满足净空和前进执行要求的完整候选/)
  assert.doesNotMatch(html,/选定覆盖形式|未达覆盖门槛|未达标方案/)
})
test('low coverage is displayed as a selected result rather than a failure',async()=>{
  const r=report();r.selected.coverage_percent=42
  r.candidates=[{id:2,mode:'CONTOUR',angle_rad:0,headland_width_m:1,coverage_percent:30,executable:true}]
  const html=await render(r)
  assert.match(html,/42\.00%/);assert.match(html,/备选/)
  assert.match(html,/覆盖率仅作评价展示，不设最低门槛/)
  assert.doesNotMatch(html,/未达覆盖门槛|未达标方案|没有可执行/)
})
test('new strategy card has fixed objectives and no legacy strategy controls',()=>{
  const parametersFile=path.resolve(__dirname,'../agri_web/src/components/tasks/taskParameters.ts')
  const fieldsCode=ts.transpileModule(fs.readFileSync(parametersFile,'utf8'),{
    compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022}}).outputText
  const fields={};new Function('exports',fieldsCode)(fields)
  assert.ok(!fields.smartPlanningFields.some(f=>['optimization_patterns','optimization_max_candidates','optimization_time_budget','optimization_headland_min','optimization_headland_max'].includes(f.key)))
  assert.deepEqual(fields.smartSearchFields,[])
  const editor=fs.readFileSync(path.resolve(__dirname,'../agri_web/src/components/tasks/TaskEditor.vue'),'utf8')
  assert.doesNotMatch(editor,/<details><summary>高级：计算限制/)
  assert.match(editor,/原地转向/)
  assert.match(editor,/0.5/)
  assert.match(editor,/minimumHeadland/)
  assert.deepEqual(fields.smartPlanningFields,[])
  assert.deepEqual(fields.smartAdvancedFields,[])
  assert.ok(!fields.coverageQualityFields.some(f=>f.key==='evaluation_width'))
})
test('automatic width results expose inferred limits and manual results identify the exact source',async()=>{
  const r=report();r.headland_search={method:'GEOMETRY_NATIVE_ADAPTIVE',range_min_m:.1751,range_max_m:5.4999,
    coarse_evaluated_count:16,refinement_evaluated_count:8,limitations:'有限预算不保证全局最优'}
  let html=await render(r)
  assert.match(html,/自动地头搜索依据/);assert.match(html,/5\.500/);assert.match(html,/细化搜索 8 套/)
  r.headland_search={method:'MANUAL',range_min_m:3,range_max_m:3}
  html=await render(r)
  assert.match(html,/严格采用任务“规划与路径跟踪参数”中的人工值/)
  assert.doesNotMatch(html,/自动地头搜索依据/)
})

test('point-turn policy shows fixed objectives minimum clearance and angle candidates without energy proxies',async()=>{
  const r=report();r.schema_version=2;r.turn_mode='POINT_TURN';r.search_complete=true;r.weights={coverage:.5,overlap:.3,length:.2}
  r.selected={...r.selected,mode:'BOUSTROPHEDON',order:'LOCAL_2OPT',point_turn_count:20,headland_width_m:.296321445,score:.125}
  r.headland_search={method:'MIN_SAFE_POINT_TURN',robot_length_m:.45,robot_width_m:.2,safety_margin_m:.05}
  r.candidates=[r.selected]
  const html=await render(r)
  for(const text of ['推荐作业角度','最小安全地头','0.296 m','原地转向点','覆盖率 0.5','重复覆盖率 0.3','总长度 0.2','行顺序已局部优化','L/(L+A/W)','不保证全局最优','已完成全部预定角度及行顺序比较'])assert.ok(html.includes(text),text)
  assert.doesNotMatch(html,/能耗代理成本|非 Wh|仅作评价展示|Fields2Cover 原生转弯曲线/)
})
