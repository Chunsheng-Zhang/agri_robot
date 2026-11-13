// Real Vue setup scripts; no HTTP server, ROS context or hardware.
const assert=require('node:assert/strict'), {test}=require('node:test')
const fs=require('node:fs'),path=require('node:path')
const deps=[path.resolve(__dirname,'../agri_web')]
const dep=name=>require(require.resolve(name,{paths:deps}))
const vue=dep('vue'),ts=dep('typescript'),{parse}=dep('@vue/compiler-sfc')
const projection={latitude:36.64,longitude:117.02,epsg:32650}
const mission=()=>({id:'mission-a',revision:1,name:'地块A',mode:'coverage',parameters:{speed:.1},waypoints:[],
  boundary:{points:[]},projection,preview:{points:[],rings:[],inset_polygons:[]},
  coverage_plans:[{id:'plan-a'},{id:'plan-b'}]})
const archived=(id='plan-a')=>({count:2,frame_id:'map',projection,fingerprint:`fingerprint-${id}`,
  preview:mission().preview,
  points:[{x:1,y:1},{x:5,y:1}],run:{id,mission_id:'mission-a',mission_revision:1,mode:'coverage',state:'COMPLETED'}})
const flush=async()=>{for(let i=0;i<12;i++)await vue.nextTick()}
function fixture(active=null){
  const props=vue.reactive({mission:mission(),planId:'plan-a',kind:'coverage',defaults:{speed:.1}})
  const context=vue.ref({active_run:active,missions:[props.mission],runs:[]})
  const devices=vue.ref({safety:{mode:'coverage',owner:'',motion_inhibited:true},waypoint:{state:'IDLE'},tracking:{state:'IDLE'},planning:{state:'IDLE'}})
  const scope=vue.effectScope(),cleanup=[],calls=[],emitted=[]
  let reply=id=>Promise.resolve(archived(id))
  let saveReply=async(id,plan)=>{
    context.value.active_run={...context.value.active_run,saved_plan_version:1}
    return {success:true,already_saved:false,run:context.value.active_run}
  }
  const modules={vue:{...vue,onUnmounted:fn=>cleanup.push(fn)},'element-plus':{ElMessage:{success(){},error(){}},ElMessageBox:{}},
    '../../composables/useWorkspace':{useWorkspace:()=>({context,refresh:async()=>{},error:vue.ref(''),loading:vue.ref(false)})},
    '../../composables/useRealtime':{useRealtime:()=>({devices,isFresh:()=>true,safetyFresh:vue.ref(true),refresh:async()=>{}})},
    '../../api/workspace':{getSavedPlanPath:async(id,plan)=>{calls.push(['preview',id,plan]);return reply(plan)},
      loadSavedPlan:async(id,plan)=>{calls.push(['load',id,plan]);context.value.active_run={id:'loaded-new',source_plan_id:plan,
        mission_id:id,mission_revision:1,state:'READY',stage:'planning',mode:'coverage'}},
      previewMission:async()=>props.mission.preview,
      savePlannedPath:async(id,plan)=>{calls.push(['save',id,plan]);return saveReply(id,plan)}},
    './taskParameters':{coverageQualityFields:[],parameterFields:{coverage:[]},smartPlanningFields:[],smartSearchFields:[],smartAdvancedFields:[]},
    './defaultTaskPoints':{defaultTaskPoints:[]}}
  function load(file,names){
    const {descriptor}=parse(fs.readFileSync(path.resolve(__dirname,'../agri_web/src/components/tasks',file),'utf8'))
    const code=ts.transpileModule(descriptor.scriptSetup.content+`\nreturn {${names.join(',')}}`,{
      compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022}}).outputText
    return scope.run(()=>new Function('require','exports','defineProps','defineEmits',code)(
      name=>{if(name.endsWith('.vue'))return {};assert.ok(modules[name],name);return modules[name]},
      {},()=>props,()=>(...args)=>emitted.push(args)))
  }
  return {props,context,devices,calls,emitted,load,reply:fn=>{reply=fn},saveReply:fn=>{saveReply=fn},close:()=>{cleanup.forEach(fn=>fn());scope.stop()}}
}
const editor=f=>f.load('TaskEditor.vue',['savedReference','savedPathError','displayRun','showLive','planReady','parameters','activateSavedPath'])
test('selecting a mission defaults to its latest saved path without loading or moving',async()=>{
  const f=fixture(),w=f.load('TaskWorkspace.vue',['choose','selectedId','selectedPlanId'])
  w.choose(f.props.mission)
  assert.equal(w.selectedId.value,'mission-a');assert.equal(w.selectedPlanId.value,'plan-a')
  w.choose(f.props.mission,'plan-b');assert.equal(w.selectedPlanId.value,'plan-b')
  assert.deepEqual(f.calls,[]);f.close()
})
test('saved path is previewed while start remains unavailable until explicit loading',async()=>{
  const f=fixture(),e=editor(f);await flush()
  assert.equal(e.savedReference.value.count,2);assert.equal(e.displayRun.value.id,'plan-a')
  assert.equal(e.planReady.value,false);assert.equal(f.context.value.active_run,null)
  assert.deepEqual(f.calls,[['preview','mission-a','plan-a']])
  await e.activateSavedPath();await flush()
  assert.equal(e.showLive.value,true);assert.equal(e.planReady.value,true)
  assert.equal(e.savedReference.value,null)
  assert.deepEqual(f.calls,[['preview','mission-a','plan-a'],['load','mission-a','plan-a']])
  assert.equal(f.devices.value.safety.motion_inhibited,true);f.close()
})
test('another ready path in the same mission does not enable start for an archived selection',async()=>{
  const f=fixture({id:'plan-b',mission_id:'mission-a',mission_revision:1,state:'READY',stage:'planning'})
  const e=editor(f);await flush()
  assert.equal(e.showLive.value,false);assert.equal(e.planReady.value,false)
  assert.equal(e.displayRun.value.id,'plan-a');f.close()
})
test('editing or having another control owner prevents archived-path activation',async()=>{
  const f=fixture(),e=editor(f);await flush()
  f.devices.value.safety.owner='waypoint';await e.activateSavedPath()
  assert.equal(f.calls.length,1)
  f.devices.value.safety.owner='';e.parameters.value.speed=.2;await flush()
  assert.equal(e.savedReference.value,null);await e.activateSavedPath()
  assert.equal(f.calls.length,1);f.close()
})
test('late preview for the previous selection cannot replace the selected path',async()=>{
  const f=fixture();let finish
  f.reply(id=>id==='plan-a'?new Promise(resolve=>{finish=resolve}):Promise.resolve(archived(id)))
  const e=editor(f);f.props.planId='plan-b';await flush()
  assert.equal(e.displayRun.value.id,'plan-b')
  finish(archived('plan-a'));await flush()
  assert.equal(e.displayRun.value.id,'plan-b');f.close()
})
test('mismatched archived response is hidden and never loaded',async()=>{
  const f=fixture();f.reply(()=>Promise.resolve({...archived(),projection:{...projection,epsg:32649}}))
  const e=editor(f);await flush()
  assert.equal(e.savedReference.value,null);assert.match(e.savedPathError.value,/原点不一致/)
  await e.activateSavedPath();assert.equal(f.calls.length,1);f.close()
})
test('reverse is false by default and only an explicit checkbox edit changes the task payload',async()=>{
  const f=fixture(),e=f.load('TaskEditor.vue',['parameters','payload','dirty']);await flush()
  assert.equal(e.parameters.value.allow_reverse,false)
  assert.equal(e.payload().parameters.allow_reverse,false)
  e.parameters.value.allow_reverse=true
  assert.equal(e.payload().parameters.allow_reverse,true)
  assert.equal(e.dirty.value,true)
  assert.deepEqual(f.calls,[['preview','mission-a','plan-a']]);f.close()
})
test('explicit save persists the ready plan with no loading or motion action',async()=>{
  const f=fixture({id:'plan-a',mission_id:'mission-a',mission_revision:1,state:'READY',stage:'planning',plan_save_required:true})
  const e=f.load('TaskEditor.vue',['savePath','planSaved','planReady']);await flush()
  assert.equal(e.planReady.value,true);assert.equal(e.planSaved.value,false)
  await e.savePath();await flush()
  assert.deepEqual(f.calls,[['save','mission-a','plan-a']])
  assert.deepEqual(f.emitted,[['pathSaved','mission-a','plan-a']])
  assert.equal(e.planSaved.value,true)
  await e.savePath();assert.equal(f.calls.length,1)
  f.close()
})
test('save is blocked for dirty tasks, another preview, planning or failed plans',async()=>{
  for(const state of ['PLANNING','FAILED','RUNNING']){
    const f=fixture({id:'plan-a',mission_id:'mission-a',mission_revision:1,state,stage:'planning'})
    const e=f.load('TaskEditor.vue',['savePath']);await flush();await e.savePath()
    assert.equal(f.calls.length,0);f.close()
  }
  const f=fixture({id:'plan-a',mission_id:'mission-a',mission_revision:1,state:'READY',stage:'planning'})
  const e=f.load('TaskEditor.vue',['savePath','parameters']);await flush()
  e.parameters.value.speed=.2;await e.savePath();assert.equal(f.calls.length,0);f.close()
})
test('failed save leaves the plan unsaved and does not emit success',async()=>{
  const f=fixture({id:'plan-a',mission_id:'mission-a',mission_revision:1,state:'READY',stage:'planning'})
  f.saveReply(async()=>{throw new Error('磁盘保存失败')})
  const e=f.load('TaskEditor.vue',['savePath','planSaved']);await flush();await e.savePath()
  assert.equal(e.planSaved.value,false);assert.deepEqual(f.emitted,[])
  assert.deepEqual(f.calls,[['save','mission-a','plan-a']]);f.close()
})
test('save completion highlights its own path without stealing another selected mission',async()=>{
  const f=fixture(),w=f.load('TaskWorkspace.vue',['choose','pathSaved','selectedId','selectedPlanId'])
  w.choose(f.props.mission)
  w.pathSaved('mission-a','new-saved-plan')
  assert.equal(w.selectedPlanId.value,'new-saved-plan')
  w.choose({id:'mission-b',coverage_plans:[]})
  w.pathSaved('mission-a','late-saved-plan')
  assert.equal(w.selectedId.value,'mission-b');assert.equal(w.selectedPlanId.value,undefined)
  assert.deepEqual(f.calls,[]);f.close()
})
