// Execute real mode-selector and monitor-map setup code without a ROS bridge.
const assert=require('node:assert/strict'),{test}=require('node:test')
const fs=require('node:fs'),path=require('node:path')
const root=path.resolve(__dirname,'..'),deps=[path.join(root,'agri_web')]
const dep=n=>require(require.resolve(n,{paths:deps}))
const vue=dep('vue'),ts=dep('typescript'),{parse}=dep('@vue/compiler-sfc')
const transpile=s=>ts.transpileModule(s,{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022}}).outputText
function fixture(){
  const exports={};new Function('require','exports',transpile(fs.readFileSync(path.join(root,'agri_web/src/api/workspace.ts'),'utf8')))(()=>({}),exports)
  const devices=vue.ref({safety:{mode:'base',owner:''},following:{fresh:true,state:'IDLE'}})
  const context=vue.ref({active_run:null,teaching:{state:'IDLE'}}),safetyFresh=vue.ref(true),calls=[],unmounted=[]
  let confirm=async()=>{},props={},confirmCount=0
  const modules={vue:{...vue,onMounted:()=>{},onUnmounted:f=>unmounted.push(f)},
    'element-plus':{ElMessage:{success:()=>{},error:()=>{}},ElMessageBox:{confirm:async()=>{confirmCount++;return confirm()}}},
    '../api/workspace':{...exports,selectMode:async mode=>calls.push(mode)},
    '../composables/useRealtime':{useRealtime:()=>({devices,safetyFresh,isFresh:s=>s?.fresh===true,refresh:async()=>{}})},
    '../composables/useWorkspace':{useWorkspace:()=>({context,refresh:async()=>{}})},
  }
  const scope=vue.effectScope()
  const defaults=(p,values)=>{
    for(const [key,value] of Object.entries(values)) if(p[key]===undefined)p[key]=typeof value==='function'?value():value
    return p
  }
  function load(file,names){
    const {descriptor}=parse(fs.readFileSync(path.join(root,'agri_web/src',file),'utf8'))
    return scope.run(()=>new Function('require','exports','defineProps','withDefaults',transpile(`${descriptor.scriptSetup.content}\nreturn {${names.join(',')}}`))(
      n=>n.endsWith('.vue')?{}:modules[n],{},()=>props,defaults))
  }
  return {exports,devices,context,safetyFresh,calls,modules,load,scope,
    confirm:f=>confirm=f,get confirmations(){return confirmCount},stop:()=>{unmounted.forEach(f=>f());scope.stop()}}
}
test('all four buttons are the default, and loading a page never selects a mode',()=>{
  const f=fixture(),m=f.load('components/ModeSelector.vue',['availableModes','current'])
  assert.deepEqual(m.availableModes.value.map(v=>v.mode),['base','waypoint','coverage','following'])
  assert.equal(f.exports.workModeText('following'),'路径跟随')
  assert.equal(f.exports.workModeText('manual'),'运行模式未更新')
  assert.equal(f.calls.length,0);f.stop()
})
test('each mode can be selected explicitly, while selecting the current mode is a no-op',async()=>{
  const f=fixture(),m=f.load('components/ModeSelector.vue',['change'])
  for(const mode of f.exports.workModes){
    f.devices.value.safety.mode=mode==='base'?'following':'base'
    await m.change(mode)
  }
  assert.deepEqual(f.calls,['base','waypoint','coverage','following'])
  f.devices.value.safety.mode='following';await m.change('following');assert.equal(f.calls.length,4)
  f.stop()
})
test('fresh following acquisition prompts even before it owns the chassis; cancellation leaves it untouched',async()=>{
  const f=fixture();f.devices.value.following.state='ACQUIRING'
  const m=f.load('components/ModeSelector.vue',['change','busy'])
  f.confirm(async()=>{throw 'cancel'})
  await m.change('waypoint')
  assert.equal(f.confirmations,1);assert.equal(f.calls.length,0);assert.equal(m.busy.value,false);f.stop()
})
test('one confirmation at a time; state becoming stale while waiting cannot switch modes',async()=>{
  const f=fixture();f.context.value.teaching.state='RECORDING'
  let release;f.confirm(()=>new Promise(resolve=>release=resolve))
  const m=f.load('components/ModeSelector.vue',['change','busy'])
  const first=m.change('following');await m.change('coverage')
  assert.equal(f.confirmations,1);assert.equal(m.busy.value,true)
  f.safetyFresh.value=false;release();await first
  assert.equal(f.calls.length,0);assert.equal(m.busy.value,false);f.stop()
})
test('leaving the page while its confirmation is pending cannot select a mode later',async()=>{
  const f=fixture();f.context.value.teaching.state='RECORDING'
  let release;f.confirm(()=>new Promise(resolve=>release=resolve))
  const m=f.load('components/ModeSelector.vue',['change'])
  const first=m.change('following');f.stop();release();await first
  assert.equal(f.calls.length,0)
})
test('other task editors still target their own controller with all four selectors visible',()=>{
  for(const file of ['components/tasks/TaskWorkspace.vue','views/FollowingView.vue','views/MonitorView.vue','views/DashboardView.vue']){
    const {descriptor}=parse(fs.readFileSync(path.join(root,'agri_web/src',file),'utf8'))
    assert.match(descriptor.template.content,/<ModeSelector\s*\/>/)
    assert.doesNotMatch(descriptor.template.content,/<ModeSelector[^>]*:modes/)
  }
})
test('monitor map selects following topics and discards late paths after switching modes',async()=>{
  const props=vue.reactive({pathSource:'following',showLivePath:true,coverageMode:false})
  const calls=[],pending=[],cleanup=[]
  const empty={frame_id:'map',points:[],count:0}
  const modules={
    vue:{...vue,useId:()=> 'four-mode-map',onMounted:()=>{},onUnmounted:f=>cleanup.push(f)},
    '@element-plus/icons-vue':{},
    '../../api/robot':{
      getReferencePath:source=>{calls.push(['reference',source]);return new Promise(resolve=>pending.push(resolve))},
      getTrackedPath:async source=>{calls.push(['track',source]);return empty},getWaypoints:async()=>empty,
    },
    '../../composables/useRealtime':{useRealtime:()=>({robot:vue.ref(null),devices:vue.ref(null),safetyFresh:vue.ref(false),poseFresh:vue.ref(false)})},
    '../../composables/useCoverageLayers':{useCoverageLayers:()=>({snapshot:vue.ref(null),raster:vue.ref(null)})},
  }
  const {descriptor}=parse(fs.readFileSync(path.join(root,'agri_web/src/components/dashboard/FieldMap.vue'),'utf8'))
  const scope=vue.effectScope()
  const code=transpile(`${descriptor.scriptSetup.content}\nreturn {refreshPath,displayReference,referencePath}`)
  const state=scope.run(()=>new Function('require','exports','defineProps','withDefaults','defineExpose','defineEmits',code)(n=>modules[n],{},()=>props,p=>p,()=>{},()=>()=>{}))
  try{
    const first=state.refreshPath()
    assert.deepEqual(calls,[['reference','following'],['track','following']])
    props.pathSource='tracking';await vue.nextTick()
    assert.deepEqual(state.displayReference.value,[])
    pending[0]({...empty,points:[{x:999,y:999}],count:1});await first
    await vue.nextTick()
    assert.deepEqual(calls.slice(-2),[['reference','tracking'],['track','tracking']])
    assert.deepEqual(state.displayReference.value,[])
    pending[1]({...empty,points:[{x:1,y:2}],count:1})
    for(let i=0;i<6;i++)await vue.nextTick()
    assert.deepEqual(state.displayReference.value,[{x:1,y:2}])
    props.showLivePath=false;await vue.nextTick()
    assert.deepEqual(state.displayReference.value,[])
  }finally{cleanup.forEach(f=>f());scope.stop()}
})
