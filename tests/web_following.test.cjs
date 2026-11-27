// Actual Vue setup logic with simulated API/lifecycle; no ROS or network.
const assert=require('node:assert/strict')
const {test}=require('node:test')
const fs=require('node:fs'),path=require('node:path')
const root=path.resolve(__dirname,'..'),deps=[path.join(root,'agri_web')]
const dependency=n=>require(require.resolve(n,{paths:deps}))
const vue=dependency('vue'),ts=dependency('typescript'),{parse}=dependency('@vue/compiler-sfc')
const controllerFields={}
new Function('exports',ts.transpileModule(fs.readFileSync(path.join(root,'agri_web/src/components/tasks/taskParameters.ts'),'utf8'),{compilerOptions:{module:ts.ModuleKind.CommonJS}}).outputText)(controllerFields)

test('coverage and following offer exactly the four migrated algorithms, never the old PP',()=>{
  const choices=controllerFields.trackingControllerChoices
  assert.deepEqual(Object.keys(choices),['pp','stanley','lqr','mpc'])
  const selection=controllerFields.parameterFields.coverage.find(field=>field.key==='controller_algorithm')
  assert.deepEqual(selection.choices,choices)
  assert.equal(controllerFields.parameterFields.waypoint.some(field=>field.key==='controller_algorithm'),false)
  const view=fs.readFileSync(path.join(root,'agri_web/src/views/FollowingView.vue'),'utf8')
  assert.match(view,/v-model="tracking.controller_algorithm" :disabled="active\|\|busy"/)
})
function fixture(){
  const calls=[],mounted=[],unmounted=[],responses=[],errors=[],devices=vue.ref({safety:{mode:'base',motion_inhibited:false}}),safetyFresh=vue.ref(true)
  const modules={
    '../components/tasks/taskParameters':controllerFields,
    vue:{...vue,onMounted:f=>mounted.push(f),onUnmounted:f=>unmounted.push(f)},
    'element-plus':{ElMessage:{success:()=>{},error:e=>errors.push(e)},ElMessageBox:{confirm:async()=>{},prompt:async()=>({value:'新名称'})}},
    '../api/following':{getFollowing:()=>new Promise((resolve,reject)=>responses.push({resolve,reject})),getTaughtPath:async id=>({id,name:'路径',points:[{x:0,y:0}],reference_fingerprint:'ack'}),followingRequest:async(...args)=>calls.push(args),
      importFollowingFile:async body=>{calls.push(['import',body]);return{id:'imported',name:body.name,reference_fingerprint:'new',points:[{x:1,y:0}],source_kind:'import'}}},
    '../composables/useRealtime':{useRealtime:()=>({devices,safetyFresh}),numberText:String,taskText:String},'../api/http':{apiUrl:x=>x},
  }
  const {descriptor}=parse(fs.readFileSync(path.join(root,'agri_web/src/views/FollowingView.vue'),'utf8'))
  const names=['context','selected','busy','received','now','fresh','controllerFresh','canEdit','canImport','importFile','name','processing','tracking','active','actualControllerLabel','pendingControllerLabel','mapRaw','mapTrack','filteredPaths','query','selectedLoaded','refresh','choose','start','recordStart','operate','rename','remove']
  const code=ts.transpileModule(`${descriptor.scriptSetup.content}\nreturn {${names.join(',')}}`,{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022}}).outputText
  const scope=vue.effectScope();let state
  scope.run(()=>state=new Function('require','exports',code)(n=>n.endsWith('.vue')?{}:modules[n],{}))
  const data=()=>({paths:[{id:'p',name:'道路 A'},{id:'q',name:'田间 B'}],recording:{state:'IDLE'},status:{fresh:true,age_seconds:.02,state:'IDLE',reference_fingerprint:'ack'},loaded_id:'p',reference:[],track:[],raw:[]})
  state.context.value=data();state.received.value=Date.now()
  return {state,calls,responses,errors,devices,safetyFresh,modules,mounted,data,stop:()=>{unmounted.forEach(f=>f());scope.stop()}}
}
test('opening the following page does not switch mode, acquire control, load or start',()=>{
  const f=fixture();assert.equal(f.calls.length,0);assert.equal(f.mounted.length,1);assert.equal(f.state.canEdit.value,true);f.stop()
})
test('choosing LQR while idle shows it as pending and never presents default PP as actual',()=>{
  const f=fixture()
  f.state.context.value.status.controller_label='PP'
  f.state.tracking.controller_algorithm='lqr'
  assert.equal(f.state.pendingControllerLabel.value,'LQR')
  for(const state of ['IDLE','COMPLETED','FAULT','ACQUIRING']){
    f.state.context.value.status.state=state
    assert.equal(f.state.actualControllerLabel.value,undefined)
  }
  f.state.context.value.status.controller_label='LQR'
  for(const state of ['RUNNING','PAUSED']){
    f.state.context.value.status.state=state
    assert.equal(f.state.actualControllerLabel.value,'LQR')
  }
  f.state.context.value.status.age_seconds=2
  assert.equal(f.state.actualControllerLabel.value,undefined)
  assert.equal(f.calls.length,0);f.stop()
})
test('the selected algorithm is sent on explicit start; active and paused tasks cannot restart',async()=>{
  const f=fixture()
  assert.equal(f.state.tracking.controller_algorithm,'pp')
  f.devices.value.safety.mode='following'
  await f.state.choose({id:'p',state:'READY'})
  for(const algorithm of ['pp','stanley','lqr','mpc']){
    f.state.tracking.controller_algorithm=algorithm
    const pending=f.state.start()
    await new Promise(resolve=>setImmediate(resolve))
    assert.equal(f.calls.at(-1)[0],'start')
    assert.equal(f.calls.at(-1)[1].controller_algorithm,algorithm)
    f.responses.at(-1).resolve(f.data());await pending
  }
  const before=f.calls.length
  for(const state of ['RUNNING','PAUSED','ACQUIRING']){
    f.state.context.value.status.state=state
    assert.equal(f.state.active.value,true)
    await f.state.start()
  }
  assert.equal(f.calls.length,before)
  f.stop()
})
test('selecting a saved path only previews; start requires explicit load acknowledgement and following mode',async()=>{
  const f=fixture();await f.state.choose({id:'p',state:'READY'})
  assert.equal(f.calls.length,0);assert.equal(f.state.selectedLoaded.value,true)
  await f.state.start();assert.equal(f.calls.length,0) // base mode cannot start replay
  f.devices.value.safety.mode='following';f.state.context.value.status.reference_fingerprint='old'
  await f.state.start();assert.equal(f.calls.length,0)
  f.stop()
})
test('expired safety/controller status blocks starting a recording or replay',async()=>{
  const f=fixture();f.state.context.value.status.age_seconds=2
  assert.equal(f.state.canEdit.value,false);await f.state.recordStart();assert.equal(f.calls.length,0)
  f.state.context.value.status.age_seconds=.02;f.safetyFresh.value=false
  await f.state.recordStart();assert.equal(f.calls.length,0)
  f.state.received.value=Date.now()-3000;assert.equal(f.state.fresh.value,false);f.stop()
})
test('recording, processing and active replay exclude library changes',()=>{
  const f=fixture()
  for(const state of ['RECORDING','PROCESSING']){f.state.context.value.recording.state=state;assert.equal(f.state.canEdit.value,false)}
  f.state.context.value.recording.state='READY'
  for(const state of ['RUNNING','PAUSED','ACQUIRING']){f.state.context.value.status.state=state;assert.equal(f.state.canEdit.value,false)}
  f.stop()
})
test('path-name search is local and does not change ROS state',()=>{
  const f=fixture();f.state.query.value='田间';assert.deepEqual(f.state.filteredPaths.value.map(p=>p.id),['q']);assert.equal(f.calls.length,0);f.stop()
})
test('polls never overlap and a late response cannot resurrect an unmounted page',async()=>{
  const f=fixture();const first=f.state.refresh();await f.state.refresh();assert.equal(f.responses.length,1)
  f.stop();f.responses[0].resolve({...f.data(),loaded_id:'late'});await first
  assert.equal(f.state.context.value.loaded_id,'p')
})
test('failed rename or delete retains the selected reference and name',async()=>{
  const f=fixture();f.state.selected.value={id:'p',name:'原名称'}
  f.modules['../api/following'].followingRequest=async()=>{throw Error('busy')}
  const rename=f.state.rename(f.state.selected.value);await new Promise(setImmediate);f.responses[0].resolve(f.data());await rename
  assert.equal(f.state.selected.value.name,'原名称')
  const remove=f.state.remove(f.state.selected.value);await new Promise(setImmediate);f.responses[1].resolve(f.data());await remove
  assert.equal(f.state.selected.value.id,'p');f.stop()
})
test('file import previews the saved path without loading, switching mode or starting; offline import is allowed',async()=>{
  const f=fixture();f.state.context.value.status.fresh=false
  f.state.context.value.raw=[{x:-1,y:0}];f.state.context.value.track=[{x:-2,y:0}]
  assert.equal(f.state.canImport.value,true);assert.equal(f.state.canEdit.value,false)
  const input={files:[{name:'road.csv',size:30,text:async()=>'longitude,latitude\n117,36'}],value:'chosen'}
  const pending=f.state.importFile({target:input});await new Promise(setImmediate)
  assert.equal(f.calls.length,1);assert.equal(f.calls[0][0],'import')
  assert.equal(f.calls[0][1].processing.spacing,.1)
  assert.equal(f.state.selected.value.id,'imported');assert.equal(f.state.selectedLoaded.value,false)
  assert.deepEqual(f.state.mapRaw.value,[]);assert.deepEqual(f.state.mapTrack.value,[])
  f.responses[0].resolve(f.data());await pending
  assert.equal(input.value,'');assert.equal(f.state.busy.value,false);f.stop()
})
test('oversized or failed file imports retain previous preview and clear input',async()=>{
  const f=fixture();f.state.selected.value={id:'previous',name:'旧预览'}
  const input={files:[{name:'huge.csv',size:2*1024*1024+1,text:async()=>{throw Error('must not read')}}],value:'chosen'}
  let pending=f.state.importFile({target:input});await new Promise(setImmediate)
  f.responses[0].resolve(f.data());await pending
  assert.equal(f.calls.length,0);assert.match(f.errors[0],/2 MiB/);assert.equal(input.value,'')
  input.files=[{name:'bad.json',size:20,text:async()=>'{}'}];input.value='chosen'
  f.modules['../api/following'].importFollowingFile=async()=>{throw Error('轨迹无效')}
  pending=f.state.importFile({target:input});await new Promise(setImmediate)
  f.responses[1].resolve(f.data());await pending
  assert.equal(f.state.selected.value.id,'previous');assert.equal(input.value,'');assert.match(f.errors[1],/轨迹无效/);f.stop()
})
test('recording, replay and stale context block file import; unmount during file read does not submit',async()=>{
  const f=fixture(),input={files:[{name:'road.csv',size:10,text:async()=>''}],value:'chosen'}
  for(const state of ['RECORDING','PROCESSING']){f.state.context.value.recording.state=state;await f.state.importFile({target:input})}
  f.state.context.value.recording.state='IDLE';f.state.context.value.status.state='RUNNING'
  await f.state.importFile({target:input});assert.equal(f.calls.length,0)
  f.state.context.value.status.state='IDLE';f.state.received.value=Date.now()-3000
  await f.state.importFile({target:input});assert.equal(f.calls.length,0)
  f.state.received.value=Date.now();let resolveText
  input.files=[{name:'road.csv',size:10,text:()=>new Promise(resolve=>resolveText=resolve)}]
  const pending=f.state.importFile({target:input});f.stop();resolveText('longitude,latitude');await pending
  assert.equal(f.calls.length,0);assert.equal(input.value,'')
})
