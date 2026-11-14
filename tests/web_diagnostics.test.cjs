const assert=require('node:assert/strict')
const {test}=require('node:test')
const fs=require('node:fs'),path=require('node:path')
const deps=[path.resolve(__dirname,'../agri_web')]
const dependency=name=>require(require.resolve(name,{paths:deps}))
const vue=dependency('vue'),ts=dependency('typescript')
const flush=()=>new Promise(setImmediate)
function fixture(){
  const calls=[],timers=new Map(),events={},unmounted=[];let timerId=0
  const doc={hidden:false,addEventListener:(name,fn)=>{events[name]=fn},removeEventListener:name=>{delete events[name]}}
  const modules={vue:{...vue,onUnmounted:fn=>unmounted.push(fn)},'../api/diagnostics':{
    getDiagnosticPage:(kind,after,session,signal)=>new Promise((resolve,reject)=>calls.push({kind,after,session,signal,resolve,reject}))}}
  const source=fs.readFileSync(path.resolve(__dirname,'../agri_web/src/composables/useDiagnostics.ts'),'utf8')
  const code=ts.transpileModule(source,{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022}}).outputText
  const exports={};new Function('require','exports','document','setTimeout','clearTimeout',code)(name=>modules[name],exports,doc,
    (fn,ms)=>{timers.set(++timerId,{fn,ms});return timerId},id=>timers.delete(id))
  const scope=vue.effectScope();let state;scope.run(()=>{state=exports.useDiagnostics()})
  return {state,calls,timers,doc,events,stop:()=>{unmounted.forEach(f=>f());scope.stop()},
    tick:ms=>{const item=[...timers].find(([,v])=>v.ms===ms);assert.ok(item,`timer ${ms}`);timers.delete(item[0]);item[1].fn()}}
}
const page=(ids,session='s')=>({session,reset:false,cursor:ids.at(-1)||0,entries:ids.map(id=>({id,source:'test',message:`row${id}`})),skipped:0})
test('diagnostic streams are opt-in and have one pending request per open panel',async()=>{
  const f=fixture();assert.equal(f.calls.length,0)
  f.state.open('logs');assert.equal(f.calls.length,1);assert.equal(f.calls[0].kind,'logs')
  assert.ok(![...f.timers.values()].some(t=>t.ms===500))
  f.calls[0].resolve(page([1,2]));await flush()
  assert.deepEqual(f.state.entries.value.map(v=>v.id),[1,2])
  f.tick(500);assert.equal(f.calls.length,2);assert.equal(f.calls[1].after,2)
  f.state.close();assert.ok(f.calls[1].signal.aborted)
  f.calls[1].resolve(page([3]));await flush();assert.equal(f.state.entries.value.length,0)
  assert.equal(f.timers.size,0);f.stop()
})
test('pause keeps capture view frozen and resume requests latest without duplicate requests',async()=>{
  const f=fixture();f.state.open('logs');f.calls[0].resolve(page([1,2]));await flush()
  f.state.paused.value=true;assert.equal(f.timers.size,0)
  f.state.paused.value=false;assert.equal(f.calls.length,2);assert.equal(f.calls[1].after,0)
  f.state.paused.value=true;f.state.open('serial')
  assert.equal(f.calls.length,3);assert.equal(f.calls[2].kind,'serial');assert.equal(f.state.paused.value,false)
  f.calls[1].resolve(page([9]));f.calls[2].resolve(page([1],'driver'));await flush()
  assert.equal(f.state.page.value.session,'driver');f.stop()
})
test('hidden page cancels polling and resumes only while a panel is open',async()=>{
  const f=fixture();f.state.open('serial');f.doc.hidden=true;f.events.visibilitychange()
  assert.ok(f.calls[0].signal.aborted)
  f.doc.hidden=false;f.events.visibilitychange();assert.equal(f.calls.length,2)
  f.state.close();f.doc.hidden=true;f.events.visibilitychange();f.doc.hidden=false;f.events.visibilitychange()
  assert.equal(f.calls.length,2);f.stop();assert.equal(Object.keys(f.events).length,0)
})
test('clear is local and cleared entries do not reappear on pause/resume',async()=>{
  const f=fixture();f.state.open('logs');f.calls[0].resolve(page([1,2]));await flush()
  f.state.clearDisplay();assert.equal(f.state.entries.value.length,0);assert.equal(f.calls.length,1)
  f.state.paused.value=true;f.state.paused.value=false;f.calls[1].resolve(page([1,2,3]));await flush()
  assert.deepEqual(f.state.entries.value.map(v=>v.id),[3]);f.stop()
})
test('request timeout reports stale data while backend restart resets cursor and view',async()=>{
  const f=fixture();f.state.open('logs');f.calls[0].resolve(page([1,2]));await flush()
  f.tick(500);f.tick(2000);assert.ok(f.calls[1].signal.aborted)
  f.calls[1].reject(Error('abort'));await flush();assert.match(f.state.error.value,/未应答/)
  assert.equal(f.state.entries.value.length,2)
  f.tick(500);f.calls[2].resolve({...page([1],'new-session'),reset:true});await flush()
  assert.deepEqual(f.state.entries.value.map(v=>v.id),[1]);assert.equal(f.state.error.value,'');f.stop()
})
test('memory remains bounded and diagnostics are placed to the left of emergency controls',async()=>{
  const f=fixture();f.state.open('logs');f.calls[0].resolve(page(Array.from({length:700},(_,i)=>i+1)));await flush()
  assert.equal(f.state.entries.value.length,500)
  const header=fs.readFileSync(path.resolve(__dirname,'../agri_web/src/components/AppHeader.vue'),'utf8')
  assert.ok(header.indexOf('<RuntimeDiagnostics />')<header.indexOf('<EmergencyControls />'))
  f.stop()
})

test('actual serial panel renders observed HEX and stale status without motion controls',async()=>{
  const {parse,compileScript}=dependency('@vue/compiler-sfc')
  const {renderToString}=dependency('@vue/server-renderer')
  const source=fs.readFileSync(path.resolve(__dirname,'../agri_web/src/components/RuntimeDiagnostics.vue'),'utf8')
  const {descriptor}=parse(source)
  const code=ts.transpileModule(compileScript(descriptor,{id:'diagnostic-render',inlineTemplate:true}).content,
    {compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022}}).outputText
  const state={active:vue.ref('serial'),paused:vue.ref(false),entries:vue.ref([{id:1,unix_ns:1794550000000000000,
    direction:'RX',function:10,function_name:'运动数据',summary:'vx=+0.100 m/s',frame:'FF FB 0A 0A',checksum_valid:true}]),
    page:vue.ref({status:{observer_online:true,rx_fresh:true,tx_frames:4,rx_frames:8,port:'/test-memory'},archive_directory:'.runtime/test'}),
    error:vue.ref(''),skipped:vue.ref(0),loading:vue.ref(false),fresh:vue.ref(true),open:()=>{},close:()=>{},clearDisplay:()=>{}}
  const modules={vue,'@element-plus/icons-vue':{Document:null,Connection:null,Close:null},'../composables/useDiagnostics':{useDiagnostics:()=>state}}
  const exports={};new Function('require','exports',code)(name=>modules[name],exports)
  async function render(){
    const context={};const app=vue.createSSRApp(exports.default)
    for(const name of ['ElButton','ElSelect','ElOption','ElInput','ElCheckbox'])app.component(name,{setup:(p,c)=>()=>vue.h('span',c.slots.default?.())})
    const main=await renderToString(app,context);return main+Object.values(context.teleports||{}).join('')
  }
  let html=await render();assert.match(html,/有新鲜有效RX帧/);assert.match(html,/FF FB 0A 0A/);assert.match(html,/vx=\+0.100 m\/s/)
  assert.match(html,/清屏不删除文件/);assert.doesNotMatch(html,/启用遥控|解除急停|开始任务/)
  state.fresh.value=false;html=await render();assert.match(html,/未更新 \/ 未接入/);assert.doesNotMatch(html,/有新鲜有效RX帧/)
})

test('source age limits live indication even when an HTTP response just arrived',async()=>{
  const f=fixture();f.state.open('serial')
  f.calls[0].resolve({...page([1]),status:{observer_online:true,rx_fresh:true,status_age_seconds:.1,observed_rx_age_seconds:.9}})
  await flush();assert.equal(f.state.fresh.value,true)
  const expiry=[...f.timers].find(([,v])=>v.ms>0&&v.ms<=100)
  assert.ok(expiry);expiry[1].fn()
  assert.equal(f.state.fresh.value,false);assert.equal(f.state.entries.value.length,1);f.stop()
})
