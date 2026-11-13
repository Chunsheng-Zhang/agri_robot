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
  const modules={vue,'@element-plus/icons-vue':{MapLocation:{render:()=>null},Refresh:{render:()=>null}},
    '../../api/robot':{getReferencePath:async()=>({frame_id:'map',points:[{x:0,y:0},{x:10,y:0}],fingerprint:run.reference_fingerprint})},
    '../../composables/useRealtime':{useRealtime:()=>({robot:vue.ref(null),devices:vue.ref(null),safetyFresh:vue.ref(false),poseFresh:vue.ref(false)})},
    '../../composables/useCoverageLayers':{useCoverageLayers:()=>({snapshot,raster,error:vue.ref(''),loading:vue.ref(false),imagesLoading:vue.ref(false),stale:vue.ref(false),refresh(){}})}}
  const exports={};new Function('require','exports',transpile(compileScript(descriptor,{id:'map',inlineTemplate:true}).content))(
    name=>{assert.ok(modules[name],name);return modules[name]},exports)
  const node=(type,text='')=>({type,text,props:{},children:[],parent:null})
  const renderer=vue.createRenderer({createElement:node,createText:text=>node('#text',text),createComment:text=>node('#comment',text),
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
  assert.equal(buttons.length,6)
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
