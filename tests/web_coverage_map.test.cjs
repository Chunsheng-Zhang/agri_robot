// Exercise the real layer lifecycle and real SVG component without ROS.
const assert = require('node:assert/strict')
const { test } = require('node:test')
const fs = require('node:fs'), path = require('node:path')
const deps = [path.resolve(__dirname, '../agri_web')]
const dependency = name => require(require.resolve(name, { paths: deps }))
const vue = dependency('vue'), ts = dependency('typescript')
const root = path.resolve(__dirname, '..')
const transpile = source => ts.transpileModule(source, { compilerOptions: {
  module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022 } }).outputText
const sequenceModule = new Function('exports', `${transpile(fs.readFileSync(path.join(root,'agri_web/src/components/dashboard/pathSequence.ts'),'utf8'))}; return exports`)({})
const projection = { epsg: 32650, latitude: 36.64, longitude: 117.02 }
const run = { id: 'run-a', reference_fingerprint: 'fingerprint-a', projection }
const ring = [[0,0],[10,0],[10,10],[0,10],[0,0]].map(([x,y]) => ({x,y}))
const hole = [[4,4],[6,4],[6,6],[4,6],[4,4]].map(([x,y]) => ({x,y}))
const insetRings = [ring.map(p=>({x:1+p.x*.8,y:1+p.y*.8})), hole.map(p=>({x:5+(p.x-5)*2,y:5+(p.y-5)*2}))]
const metadata = (id = run.id) => ({ state: 'READY', run_id: id, frame_id: 'map', projection,
  reference_fingerprint: run.reference_fingerprint, extent: [0,0,10,10], inset_polygons: [insetRings],
  field_rings: [ring,hole], sources: { planned: { version: '1'.repeat(20), status: 'PREDICTED' },
    actual: { version: '2'.repeat(20), status: 'OBSERVED', track_lines: [[{x:1,y:1},{x:2,y:1}]] } } })
const flush = async () => { for (let i=0;i<12;i++) await vue.nextTick() }
function fixture() {
  const mounted = [], unmounted = [], timers = new Map(), requests = [], urls = [], revoked = []
  let counter = 0, payload = metadata(), deferred = false, metadataDeferred = null
  const current = vue.ref({...run}), source = vue.ref('actual')
  const doc = { visibilityState:'visible', addEventListener(){}, removeEventListener(){} }
  const modules = { vue: {...vue, onMounted: f=>mounted.push(f), onUnmounted:f=>unmounted.push(f)},
    '../api/workspace': { getCoverageMap: () => metadataDeferred || Promise.resolve(payload),
      coverageMapImageUrl: (id,source,layer,v)=>`${id}/${source}/${layer}/${v}` } }
  const api = new Function('require','exports','document','fetch','URL','setInterval','setTimeout','clearTimeout',
    `${transpile(fs.readFileSync(path.join(root,'agri_web/src/composables/useCoverageLayers.ts'),'utf8'))}; return exports`)(
    name=>modules[name],{},doc,(url,options)=>new Promise(resolve=>{
      requests.push({url,options,resolve})
      if(!deferred) resolve({ok:true,blob:async()=>({url})})
    }),{createObjectURL:blob=>{const url=`blob:${urls.length}-${blob.url}`;urls.push(url);return url},revokeObjectURL:url=>revoked.push(url)},
    f=>{timers.set(++counter,f);return counter},f=>{timers.set(++counter,f);return counter},id=>timers.delete(id))
  const scope = vue.effectScope(); const state = scope.run(()=>api.useCoverageLayers(()=>current.value,()=>source.value))
  return {state,api,current,source,requests,urls,revoked,doc,
    mount:()=>mounted.forEach(f=>f()),unmount:()=>{unmounted.forEach(f=>f());scope.stop()},
    payload:value=>{payload=value},defer:()=>{deferred=true},deferMetadata:p=>{metadataDeferred=p} }
}
test('reject wrong run, origin and reference before showing layers',()=>{
  const f=fixture()
  assert.throws(()=>f.api.validateCoverageMap(metadata('run-b'),run),/当前任务/)
  assert.throws(()=>f.api.validateCoverageMap({...metadata(),projection:{...projection,epsg:32649}},run),/固定原点/)
  assert.throws(()=>f.api.validateCoverageMap({...metadata(),reference_fingerprint:'wrong'},run),/路径/)
  assert.throws(()=>f.api.validateCoverageMap({...metadata(),extent:[0,0,NaN,1]},run),/范围/)
  f.unmount()
})
test('all three images swap together, retain previous while loading, revoke on source/run change',async()=>{
  const f=fixture(); f.mount();await flush()
  assert.equal(f.state.raster.value.source,'actual');assert.equal(f.urls.length,3)
  f.defer();f.source.value='planned';await flush()
  assert.equal(f.state.raster.value,null)
  const pending=f.requests.slice(-3)
  pending[0].resolve({ok:true,blob:async()=>({url:'first'})});await flush()
  assert.equal(f.state.raster.value,null,'Must not mix images from different snapshots')
  pending.slice(1).forEach(r=>r.resolve({ok:true,blob:async()=>({url:r.url})}));await flush()
  assert.equal(f.state.raster.value.source,'planned');assert.equal(f.revoked.length,3)
  f.current.value=null;await flush()
  assert.equal(f.state.snapshot.value,null);assert.equal(f.state.raster.value,null);assert.equal(f.revoked.length,6)
  f.unmount()
})
test('late image replies after unmount cannot restore old task overlays',async()=>{
  const f=fixture();f.defer();f.mount();await flush();assert.equal(f.requests.length,3)
  f.unmount();assert.ok(f.requests.every(r=>r.options.signal.aborted))
  f.requests.forEach(r=>r.resolve({ok:true,blob:async()=>({url:r.url})}));await flush()
  assert.equal(f.state.raster.value,null);assert.equal(f.revoked.length,f.urls.length)
})
test('invalid metadata and hidden browser never start new texture requests',async()=>{
  const f=fixture();f.payload({...metadata(),frame_id:'odom'});f.mount();await flush()
  assert.match(f.state.error.value,/固定原点/);assert.equal(f.requests.length,0)
  f.doc.visibilityState='hidden';f.payload(metadata());await f.state.refresh();await flush()
  assert.equal(f.requests.length,0);f.unmount()
})
test('outdated metadata response is discarded when task changes',async()=>{
  const f=fixture();let resolve
  f.deferMetadata(new Promise(r=>{resolve=r}));f.mount()
  f.current.value=null;await flush();resolve(metadata());await flush()
  assert.equal(f.state.snapshot.value,null);assert.equal(f.requests.length,0);f.unmount()
})

// Render actual FieldMap to a small Vue host so checkbox clicks execute real logic.
function mountMap(extra = {}) {
  const {parse,compileScript}=dependency('@vue/compiler-sfc')
  const {descriptor}=parse(fs.readFileSync(path.join(root,'agri_web/src/components/dashboard/FieldMap.vue'),'utf8'))
  const snapshot=vue.ref(metadata()),raster=vue.ref({run:run.id,source:'actual',version:'2'.repeat(20),
    extent:[0,0,10,10],images:{completed:'completed.png',remaining:'remaining.png',overlap:'overlap.png'}})
  const modules={vue,'./pathSequence':sequenceModule,'@element-plus/icons-vue':{MapLocation:{render:()=>null},Refresh:{render:()=>null}},
    '../../api/robot':{getReferencePath:async()=>({frame_id:'map',points:[{x:0,y:0},{x:10,y:0}],fingerprint:run.reference_fingerprint})},
    '../../composables/useRealtime':{useRealtime:()=>({robot:vue.ref(null),devices:vue.ref(null),safetyFresh:vue.ref(false),poseFresh:vue.ref(false)})},
    '../../composables/useCoverageLayers':{useCoverageLayers:()=>({snapshot,raster,error:vue.ref(''),loading:vue.ref(false),imagesLoading:vue.ref(false),stale:vue.ref(false),refresh(){}})}}
  const exports={};new Function('require','exports',transpile(compileScript(descriptor,{id:'map',inlineTemplate:true}).content))(
    name=>{assert.ok(modules[name],name);return modules[name]},exports)
  const node=(type,text='')=>({type,text,props:{},children:[],parent:null})
  const renderer=vue.createRenderer({createElement:type=>node(type),createText:text=>node('#text',text),createComment:text=>node('#comment',text),
    setText:(n,text)=>{n.text=text},setElementText:(n,text)=>{n.text=text;n.children=[]},
    patchProp:(n,key,old,value)=>{n.props[key]=value},parentNode:n=>n.parent,
    nextSibling:n=>n.parent?.children[n.parent.children.indexOf(n)+1],
    insert:(n,parent,anchor)=>{if(n.parent){const i=n.parent.children.indexOf(n);if(i>=0)n.parent.children.splice(i,1)}n.parent=parent;const i=parent.children.indexOf(anchor);parent.children.splice(i<0?parent.children.length:i,0,n)},
    remove:n=>{if(n.parent)n.parent.children.splice(n.parent.children.indexOf(n),1)}})
  const app=renderer.createApp(exports.default,{coverageMode:true,coverageRun:run,
    preview:{frame_id:'map',points:[],rings:[ring,hole],inset_polygons:[insetRings],inset_distance_m:1},...extra})
  for(const name of ['ElCard','ElTag','ElAlert','ElIcon','ElButton','ElRadioGroup','ElRadioButton'])
    app.component(name,{setup:(p,{attrs,slots})=>()=>vue.h('div',attrs,[slots.header?.(),slots.default?.()])})
  app.component('ElCheckbox',{props:['modelValue'],emits:['update:modelValue'],setup:(p,{emit,slots})=>()=>vue.h('button',{
    'data-checked':p.modelValue,onClick:()=>emit('update:modelValue',!p.modelValue)},slots.default?.())})
  const container=node('root');app.mount(container)
  const all=(n=container)=>[n,...n.children.flatMap(c=>all(c))]
  const text=n=>n.text+n.children.map(text).join('')
  return {app,all,text,snapshot,raster}
}
test('six map layers use one north-up metric transform, clip holes, toggle independently',async()=>{
  const f=mountMap();await flush()
  const buttons=f.all().filter(n=>n.type==='button')
  assert.equal(buttons.length,8)
  assert.ok(buttons.some(n=>f.text(n).includes('已完成作业区域')))
  const image=f.all().find(n=>n.type==='image'&&n.props['data-layer']==='completed')
  assert.equal(image.props.x,210);assert.equal(image.props.y,35)
  assert.equal(image.props.width,380);assert.equal(image.props.height,380)
  assert.equal(f.all().find(n=>n.type==='clipPath').children[0].props['clip-rule'],'evenodd')
  assert.equal(f.all().filter(n=>n.props['data-layer']==='forbidden').length,1)
  const reference=f.all().find(n=>n.props['data-layer']==='reference')
  assert.ok(reference.props.points)
  // Optional local visual artifact from the real SVG renderer and real PNGs.
  if(process.env.AGRI_MAP_REVIEW_DIRECTORY) {
    const directory=process.env.AGRI_MAP_REVIEW_DIRECTORY
    const escape=value=>String(value).replaceAll('&','&amp;').replaceAll('"','&quot;').replaceAll('<','&lt;')
    const serialize=n=>{
      if(n.type==='#text')return escape(n.text)
      if(n.type==='#comment')return ''
      const props={...n.props}
      if(n.type==='svg')Object.assign(props,{xmlns:'http://www.w3.org/2000/svg',width:800,height:450})
      if(n.type==='image')props.href=`data:image/png;base64,${fs.readFileSync(path.join(directory,props.href)).toString('base64')}`
      const attrs=Object.entries(props).filter(([key,value])=>!key.startsWith('on')&&value!=null).map(([k,v])=>`${k}="${escape(v)}"`).join(' ')
      return `<${n.type} ${attrs}>${escape(n.text)}${n.children.map(serialize).join('')}</${n.type}>`
    }
    fs.writeFileSync(path.join(directory,'map.svg'),serialize(f.all().find(n=>n.type==='svg')))
  }
  buttons.find(n=>f.text(n).includes('已完成作业区域')).props.onClick();await flush()
  assert.equal(f.all().filter(n=>n.type==='image'&&n.props['data-layer']==='completed').length,0)
  assert.equal(f.all().filter(n=>n.type==='image'&&n.props['data-layer']==='overlap').length,1)
  buttons.find(n=>f.text(n).includes('内缩轮廓')).props.onClick();await flush()
  assert.equal(f.all().filter(n=>n.props['data-layer']==='inset').length,0)
  assert.ok(f.all().some(n=>n.type==='image'&&n.props['data-layer']==='remaining'))
  f.app.unmount()
})

test('saved reference is visible with live ROS path queries disabled',async()=>{
  const f=mountMap({showLivePath:false,savedReference:{frame_id:'map',count:2,fingerprint:'saved-path',
    points:[{x:1,y:2},{x:8,y:2}]}})
  await flush()
  const line=f.all().find(n=>n.props['data-layer']==='reference')
  assert.ok(line.props.points)
  assert.ok(f.all().some(n=>n.type==='text'&&f.text(n)==='起点'))
  assert.ok(f.all().some(n=>n.type==='text'&&f.text(n)==='终点'))
  assert.ok(f.all().some(n=>f.text(n).includes('规划预测')))
  f.app.unmount()
})

test('sequence compacts dense straight samples while preserving bends and reversals',()=>{
  const points=Array.from({length:1001},(_,i)=>({x:i/100,y:0}))
  points.push({x:10,y:0},{x:10,y:2},{x:0,y:2},{x:4,y:2})
  const bands=sequenceModule.sequenceBands(points)
  assert.equal(bands.length,4)
  assert.deepEqual(bands[0].points,[{x:0,y:0},{x:10,y:0}])
  assert.deepEqual(bands.at(-1).points,[{x:0,y:2},{x:4,y:2}])
  assert.equal(bands[0].color,'#440154');assert.equal(bands.at(-1).color,'#fde725')
  assert.ok(sequenceModule.sequenceBands(Array.from({length:20000},(_,i)=>({x:i/100,y:Math.sin(i/30)}))).length<=96)
})

const displayOrder={state:'READY',rows:[
  {visit:1,start:{x:1,y:1},end:{x:9,y:1}},
  {visit:2,start:{x:9,y:3},end:{x:1,y:3}}],retracing_state:'READY',retraced_lines:[[{x:4,y:1},{x:7,y:1}]]}
test('real SVG shows traversal colours, real visit numbers and partial retrace in red, independently toggled',async()=>{
  const reference={frame_id:'map',count:4,fingerprint:'saved-a',points:[{x:1,y:1},{x:9,y:1},{x:9,y:3},{x:1,y:3}],path_order:displayOrder}
  const before=JSON.stringify(reference)
  const f=mountMap({showLivePath:false,savedReference:reference});await flush()
  const bands=f.all().filter(n=>n.props['data-layer']==='reference'&&n.props.points)
  assert.equal(bands.length,3);assert.equal(new Set(bands.map(n=>n.props.stroke)).size,3)
  const labels=f.all().find(n=>n.props['data-layer']==='row-order').children.filter(n=>n.type==='text')
  assert.equal(labels.length,2);assert.ok(f.text(labels[0]).endsWith('1'));assert.ok(f.text(labels[1]).endsWith('2'))
  assert.equal(labels[0].props.x,400);assert.equal(labels[0].props.y,377)
  const retrace=f.all().find(n=>n.props['data-layer']==='retraced').children.find(n=>n.type==='polyline')
  assert.equal(retrace.props.stroke,'#d52d35');assert.equal(retrace.props.points,'362,377 476,377')
  const buttons=f.all().filter(n=>n.type==='button')
  buttons.find(n=>f.text(n).includes('作业行访问序号')).props.onClick();await flush()
  assert.equal(f.all().filter(n=>n.props['data-layer']==='row-order').length,0)
  assert.ok(f.all().some(n=>n.props['data-layer']==='retraced'))
  buttons.find(n=>f.text(n).includes('重走路径')).props.onClick();await flush()
  assert.equal(f.all().filter(n=>n.props['data-layer']==='retraced').length,0)
  buttons.find(n=>f.text(n).includes('规划作业路径')).props.onClick();await flush()
  assert.equal(f.all().filter(n=>n.props['data-layer']==='reference-sequence').length,0)
  assert.equal(JSON.stringify(reference),before)
  f.app.unmount()
})

test('another reference fingerprint cannot leave old row labels or red segments on the new path',async()=>{
  const f=mountMap();f.snapshot.value={...metadata(),path_order:displayOrder};await flush()
  assert.ok(f.all().find(n=>n.props['data-layer']==='row-order').children.some(n=>n.type==='text'))
  f.snapshot.value={...metadata(),reference_fingerprint:'different-path',path_order:displayOrder};await flush()
  assert.equal(f.all().find(n=>n.props['data-layer']==='row-order').children.filter(n=>n.type==='text').length,0)
  assert.equal(f.all().find(n=>n.props['data-layer']==='retraced').children.filter(n=>n.type==='polyline').length,0)
  f.app.unmount()
})

test('legacy path still has order colours, with no invented work-row numbers',async()=>{
  const f=mountMap({showLivePath:false,savedReference:{frame_id:'map',count:3,fingerprint:'legacy',
    points:[{x:1,y:1},{x:9,y:1},{x:9,y:3}]}});await flush()
  assert.equal(f.all().filter(n=>n.props['data-layer']==='reference'&&n.props.points).length,2)
  assert.equal(f.all().find(n=>n.props['data-layer']==='row-order').children.filter(n=>n.type==='text').length,0)
  f.app.unmount()
})

// Optional offline visual proof using the actual component and a frozen field.
test('optional frozen native route renders the real visit labels and red overlays',async()=>{
  if(!process.env.AGRI_MAP_SEQUENCE_REFERENCE_FILE)return
  const saved=JSON.parse(fs.readFileSync(process.env.AGRI_MAP_SEQUENCE_REFERENCE_FILE,'utf8'))
  const f=mountMap({showLivePath:false,savedReference:saved,preview:saved.preview})
  f.snapshot.value=null;f.raster.value=null;await flush()
  assert.equal(f.all().find(n=>n.props['data-layer']==='row-order').children.filter(n=>n.type==='text').length,saved.path_order.rows.length)
  assert.ok(f.all().find(n=>n.props['data-layer']==='retraced').children.some(n=>n.type==='polyline'))
  const escape=value=>String(value).replaceAll('&','&amp;').replaceAll('"','&quot;').replaceAll('<','&lt;')
  const serialize=n=>{
    if(n.type==='#text')return escape(n.text)
    if(n.type==='#comment')return ''
    const props={...n.props}
    if(n.type==='svg')Object.assign(props,{xmlns:'http://www.w3.org/2000/svg',width:1200,height:675})
    const attrs=Object.entries(props).filter(([key,value])=>!key.startsWith('on')&&value!=null).map(([k,v])=>`${k}="${escape(v)}"`).join(' ')
    return `<${n.type} ${attrs}>${escape(n.text)}${n.children.map(serialize).join('')}</${n.type}>`
  }
  const css='.row-visit-label{fill:#233d30;font-weight:650;paint-order:stroke;stroke:white;stroke-linejoin:round}.endpoint-label{font-size:13px;font-weight:600;paint-order:stroke;stroke:white;stroke-width:3px}'
  const svg=serialize(f.all().find(n=>n.type==='svg')).replace(/(<svg[^>]*>)/,`$1<style>${css}</style>`)
  const out=path.dirname(process.env.AGRI_MAP_SEQUENCE_REFERENCE_FILE)
  fs.writeFileSync(path.join(out,'map_sequence.svg'),svg)
  fs.writeFileSync(path.join(out,'map_sequence.html'),`<!doctype html><html lang="zh"><meta charset="utf-8"><style>body{font:16px sans-serif;background:white;margin:24px}.ramp{display:inline-block;width:160px;height:10px;background:linear-gradient(90deg,#440154,#21918c,#fde725)}</style><h2>覆盖作业地图 · 按行驶顺序着色与编号</h2><p>行驶顺序：先 <span class="ramp"></span> 后　<span style="color:#d52d35">━ 重走路径</span>　● 起点 / 终点旁标文字</p>${svg}<p>离线双孔洞样例；使用真实Vue地图组件绘制。未连接ROS或车辆。</p></html>`)
  f.app.unmount()
})
