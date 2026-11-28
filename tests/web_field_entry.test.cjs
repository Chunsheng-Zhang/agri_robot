const assert=require('node:assert/strict'),{test}=require('node:test')
const fs=require('node:fs'),path=require('node:path')
const root=path.resolve(__dirname,'..'),deps=[path.join(root,'agri_web')]
const dep=n=>require(require.resolve(n,{paths:deps}))
const vue=dep('vue'),ts=dep('typescript'),{parse}=dep('@vue/compiler-sfc')
const compile=(file,names)=>{
  const {descriptor}=parse(fs.readFileSync(path.join(root,'agri_web/src',file),'utf8'))
  return ts.transpileModule(`${descriptor.scriptSetup.content}\nreturn {${names.join(',')}}`,{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022}}).outputText
}
const projection={latitude:36,longitude:117,epsg:32650}
const trackingFields={}
new Function('exports',ts.transpileModule(fs.readFileSync(path.join(root,'agri_web/src/components/tasks/taskParameters.ts'),'utf8'),{compilerOptions:{module:ts.ModuleKind.CommonJS}}).outputText)(trackingFields)
function editor(){
  const props=vue.reactive({kind:'coverage',defaults:{},mission:{id:'mission',revision:1,name:'入口测试',mode:'coverage',parameters:{},waypoints:[],
    boundary:{points:[{longitude:117,latitude:36},{longitude:117.001,latitude:36},{longitude:117,latitude:36.001}]},
    preview:{points:[],rings:[],inset_polygons:[]},projection}})
  const context=vue.ref({origin:projection,active_run:{id:'run',mission_id:'mission',mission_revision:1,stage:'planning',state:'READY'}})
  const cleanup=[],calls=[],requests=[],scope=vue.effectScope()
  const modules={vue:{...vue,onUnmounted:f=>cleanup.push(f)},'element-plus':{ElMessage:{success(){},error(){}}},
    '../../composables/useRealtime':{useRealtime:()=>({devices:vue.ref({safety:{mode:'coverage',owner:''}}),safetyFresh:vue.ref(true),isFresh:v=>!!v,refresh:async()=>{}})},
    '../../composables/useWorkspace':{useWorkspace:()=>({context,refresh:async()=>{}})},
    '../../api/workspace':{
      mapToWgs84:(...args)=>new Promise(resolve=>requests.push({args,resolve})),
      missionAction:async(...args)=>calls.push(args),previewMission:async()=>props.mission.preview,
      saveMission:async payload=>({...payload,id:'mission',revision:2,projection,preview:props.mission.preview}),
    },'./taskParameters':trackingFields,'./defaultTaskPoints':{defaultTaskPoints:[]},
  }
  const code=compile('components/tasks/TaskEditor.vue',['entrySource','entryLongitude','entryLatitude','entryLocal','entryPicking','entryLoading','selectEntry','clearEntry','payload','dirty','planReady','operate','save','parameters','parameterChoiceChanged'])
  const state=scope.run(()=>new Function('require','exports','defineProps','defineEmits',code)(n=>n.endsWith('.vue')?{}:modules[n],{},()=>props,()=>()=>{}))
  return {state,props,context,calls,requests,stop:()=>{cleanup.forEach(f=>f());scope.stop()}}
}
test('choosing yesterday PP changes only the draft base distance and invalidates old planning',()=>{
  const f=editor(),e=f.state
  e.parameters.value.speed=.5;e.parameters.value.max_angular=5
  e.parameters.value.controller_algorithm='pp_speed';e.parameterChoiceChanged('controller_algorithm')
  assert.equal(e.parameters.value.lookahead,.8)
  assert.equal(e.parameters.value.speed,.5);assert.equal(e.parameters.value.max_angular,5)
  assert.equal(e.dirty.value,true);assert.equal(e.planReady.value,false)
  assert.equal(f.calls.length,0)
  e.parameters.value.lookahead=.9;e.parameterChoiceChanged('route_mode')
  assert.equal(e.parameters.value.lookahead,.9)
  e.parameters.value.controller_algorithm='pp';e.parameterChoiceChanged('controller_algorithm')
  assert.equal(e.parameters.value.lookahead,.5590)
  assert.equal(f.props.mission.parameters.lookahead,undefined);f.stop()
})
test('coordinate entry is saved with the task and invalidates both planning execution and old readiness',async()=>{
  const f=editor(),e=f.state
  assert.equal(e.planReady.value,true)
  e.entryLongitude.value='117.00001';e.entryLatitude.value='36.00002'
  assert.equal(e.dirty.value,true);assert.equal(e.planReady.value,false)
  assert.deepEqual(e.payload().entry,{longitude:117.00001,latitude:36.00002,source:'coordinates'})
  await e.operate('plan');await e.operate('optimize');assert.equal(f.calls.length,0)
  await e.save();assert.equal(e.dirty.value,false);assert.equal(e.planReady.value,false)
  await e.operate('plan');await e.operate('optimize')
  assert.deepEqual(f.calls,[['mission','plan'],['mission','optimize']]);f.stop()
})
test('map pick fills WGS84 fields without sending motion or planning commands',async()=>{
  const f=editor(),e=f.state
  const pending=e.selectEntry({x:2,y:3})
  assert.deepEqual(f.requests[0].args,[2,3,projection])
  f.requests[0].resolve({x:2,y:3,longitude:117.00002,latitude:36.00003,projection})
  await pending
  assert.equal(e.entrySource.value,'map')
  assert.deepEqual(e.entryLocal.value,{x:2,y:3})
  assert.deepEqual(e.payload().entry,{source:'map',longitude:117.00002,latitude:36.00003})
  assert.equal(f.calls.length,0);f.stop()
})
test('clear or another task selection discards late coordinate conversion',async()=>{
  const f=editor(),e=f.state
  const pending=e.selectEntry({x:2,y:3});e.clearEntry()
  f.requests[0].resolve({x:2,y:3,longitude:117.00002,latitude:36.00003,projection});await pending
  assert.equal(e.payload().entry,null);assert.equal(e.entryLocal.value,null)
  e.entryLongitude.value='117';assert.throws(()=>e.payload(),/入口/)
  const later=e.selectEntry({x:5,y:6})
  f.props.mission={...f.props.mission,id:'another',entry:{longitude:117.00008,latitude:36.00009,source:'coordinates'}}
  await vue.nextTick()
  f.requests[1].resolve({x:5,y:6,longitude:117.00005,latitude:36.00006,projection});await later
  assert.equal(e.payload().entry.longitude,117.00008)
  f.stop()
})
test('SVG picking converts screen coordinates through zoom/pan back to local map metres',()=>{
  const props=vue.reactive({coverageMode:false,showLivePath:false,entryPicking:true,entryEditable:true,entryPoint:null,
    preview:{points:[],rings:[[{x:0,y:0},{x:20,y:0},{x:20,y:10},{x:0,y:10}]]}})
  const scope=vue.effectScope(),cleanup=[],emitted=[]
  const modules={vue:{...vue,useId:()=> 'entry-test',onMounted:()=>{},onUnmounted:f=>cleanup.push(f)},'@element-plus/icons-vue':{},
    '../../composables/useRealtime':{useRealtime:()=>({robot:vue.ref(null),devices:vue.ref(null),safetyFresh:vue.ref(false),poseFresh:vue.ref(false)})},
    '../../composables/useCoverageLayers':{useCoverageLayers:()=>({snapshot:vue.ref(null),raster:vue.ref(null)})},'../../api/robot':{},
    './pathSequence':{sequenceBands:()=>[]}}
  const state=scope.run(()=>new Function('require','exports','defineProps','withDefaults','defineEmits','defineExpose',
    compile('components/dashboard/FieldMap.vue',['transform','pointerDown','pointerMove','pointerUp']))(n=>modules[n],{},()=>props,p=>p,()=> (...args)=>emitted.push(args),()=>{}))
  try{
    const local={x:3,y:4},pixel=state.transform.value(local)
    const element={setPointerCapture(){},getScreenCTM:()=>({a:2,b:0,c:0,d:2,e:10,f:20})}
    const event={clientX:pixel.x*2+10,clientY:pixel.y*2+20,pointerId:1,currentTarget:element,target:{closest:()=>null}}
    state.pointerDown(event);state.pointerUp(event)
    assert.equal(emitted[0][0],'entrySelected')
    assert.ok(Math.abs(emitted[0][1].x-3)<1e-9 && Math.abs(emitted[0][1].y-4)<1e-9)
    props.entryPicking=false;props.entryPoint={x:3,y:4}
    const destination=state.transform.value({x:7,y:2})
    const markerEvent={...event,target:{closest:()=>({})}}
    state.pointerDown(markerEvent)
    const moved={...markerEvent,clientX:destination.x*2+10,clientY:destination.y*2+20}
    state.pointerMove(moved);state.pointerUp(moved)
    assert.ok(Math.abs(emitted[1][1].x-7)<1e-9 && Math.abs(emitted[1][1].y-2)<1e-9)
  }finally{cleanup.forEach(f=>f());scope.stop()}
})
