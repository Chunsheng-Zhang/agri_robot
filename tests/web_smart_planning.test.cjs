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
async function render(optimization,runOverrides={}) {
  const app = vue.createSSRApp({render:()=>vue.h(out.default,{run:{id:'synthetic',optimization,...runOverrides}})})
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
  assert.match(editor,/0.40/)
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

test('cell planner displays block integrity and connection distance without new controls',async()=>{
  const r=report();r.schema_version=2;r.turn_mode='POINT_TURN';r.search_complete=true
  r.selected={...r.selected,mode:'BOUSTROPHEDON',order:'LOCAL_2OPT',cell_count:4,cell_reentries:0,
    swath_length_m:680,transition_length_m:70,connection_policy:'CELL_BOUNDARIES'}
  r.candidates=[r.selected]
  const html=await render(r)
  for(const value of ['Fields2Cover 分区 4 个','分区顺序已局部优化','680.00 m','70.00 m','连接沿分区边界通道行驶'])assert.ok(html.includes(value),value)
  assert.doesNotMatch(html,/<input|<select/)
})

test('native TSP report allows cross-cell rows and identifies the two open route ends',async()=>{
  const r=report();r.policy='POINT_TURN_NATIVE_TSP';r.turn_mode='POINT_TURN';r.search_complete=true
  r.selected={...r.selected,mode:'TSP',order:'F2C_TSP',variant:1,cell_count:4,cell_reentries:2,
    swath_length_m:680,transition_length_m:70,routing:'F2C_TSP',connection_policy:'F2C_ROUTE_CONNECTIONS',
    native_tsp:{time_limit_seconds:1}}
  r.candidates=[r.selected]
  const html=await render(r,{entry:{source:'coordinates'}})
  for(const value of ['Fields2Cover 原生TSP','允许跨分区安排作业行','完整保留原生Route连接','原生自由路线另一端','不强制返回起点','1 秒'])assert.ok(html.includes(value),value)
  assert.doesNotMatch(html,/各分区连续往复作业|分区顺序已局部优化|起步方案 2|<input|<select/)
  r.selected.variant=3;r.selected.native_seed='ENTRY_ANCHORED_CYCLE_OPENED'
  assert.match(await render(r,{entry:{source:'coordinates'}}),/入口引导路线另一端/)
})

test('edge-aligned layout is explained without new inputs or relabeling legacy paths',async()=>{
  const r=report();r.policy='POINT_TURN_NATIVE_TSP';r.turn_mode='POINT_TURN'
  assert.doesNotMatch(await render(r),/靠边排线：/)
  r.selected.row_layout={policy:'EDGE_ALIGNED_FIXED_SPACING'}
  const html=await render(r)
  for(const value of ['靠边排线：','中间保持作业幅宽间距','作业方向继续自动搜索','原地转向净空保持不变'])assert.ok(html.includes(value),value)
  assert.doesNotMatch(html,/<input|<select/)
})

test('four-objective boundary report uses archived weights and exposes turns and supplements without inputs',async()=>{
  const r=report();r.schema_version=3;r.policy='POINT_TURN_NATIVE_TSP';r.turn_mode='POINT_TURN'
  r.weights={coverage:.4,overlap:.25,length:.2,turns:.15}
  r.normalization={turn_reference_count:34.4};r.candidates_truncated=true
  r.selected={...r.selected,point_turn_count:33,order:'F2C_TSP',
    basis:{kind:'EDGE',ring:'OUTER',ring_index:0,edge_index:3},
    start:{x:1,y:2},end:{x:3,y:4},row_layout:{policy:'EDGE_ANCHORED_FIXED_SPACING',last_side_row_added:false}}
  r.candidates=[r.selected]
  const html=await render(r)
  for(const value of ['覆盖率 0.4','重复覆盖率 0.25','总长度 0.2','转弯次数 0.15','N/(N+N0)',
    '34.40','外边界 · 边 4','不追加补边行','绿色起点','红色终点','前128条及选定候选'])assert.ok(html.includes(value),value)
  assert.doesNotMatch(html,/<input|<select|覆盖率 0.5/)
  r.selected.row_layout.last_side_row_added=true
  assert.match(await render(r),/本方案采用补边布局/)
  r.weights={coverage:.5,overlap:.3,length:.2};r.schema_version=2
  const old=await render(r)
  assert.match(old,/覆盖率 0.5/);assert.doesNotMatch(old,/转弯次数 0.15|N\/\(N\+N0\)/)
})

test('native whole-field selection is visible without adding manual strategy controls',async()=>{
  const r=report();r.turn_mode='POINT_TURN';r.policy='POINT_TURN_NATIVE_TSP'
  r.layout_search={families:['EDGE_GLOBAL','NATIVE_WHOLE_FIELD','NATIVE_PER_CELL']}
  r.selected={...r.selected,layout_family:'NATIVE_WHOLE_FIELD',direction_mode:'GLOBAL',order:'F2C_TSP'}
  r.candidates=[r.selected]
  const html=await render(r)
  assert.match(html,/原生整田不分解/)
  assert.match(html,/沿边统一方向、原生整田不分解、原生分区独立方向/)
  assert.doesNotMatch(html,/<input|<select/)
})

test('per-cell directions are displayed individually rather than as one global direction',async()=>{
  const r=report();r.turn_mode='POINT_TURN';r.policy='POINT_TURN_NATIVE_TSP'
  r.selected={...r.selected,layout_family:'NATIVE_PER_CELL',direction_mode:'PER_CELL',angle_rad:.7,
    cell_count:2,cell_directions:[{cell_id:0,angle_rad:0,swath_count:2},{cell_id:1,angle_rad:Math.PI/2,swath_count:3}]}
  r.candidates=[r.selected]
  const html=await render(r)
  for(const text of ['各分区独立','各分区作业方向','分区 1：0.00°','分区 2：90.00°','不额外预留地头'])assert.ok(html.includes(text),text)
  assert.doesNotMatch(html,/40\.11°/)
})
