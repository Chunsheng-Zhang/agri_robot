<script setup lang="ts">
import { computed, onMounted, onUnmounted, reactive, ref } from 'vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import ModeSelector from '../components/ModeSelector.vue'
import FollowingMap from '../components/FollowingMap.vue'
import { trackingControllerChoices } from '../components/tasks/taskParameters'
import { followingRequest, getFollowing, getTaughtPath, importFollowingFile } from '../api/following'
import type { FollowingContext, FollowingParameters, ProcessingParameters, TaughtPath } from '../api/following'
import { useRealtime, numberText, taskText } from '../composables/useRealtime'
import { apiUrl } from '../api/http'
const { devices, safetyFresh }=useRealtime()
const context=ref<FollowingContext|null>(null), selected=ref<TaughtPath|null>(null)
const error=ref(''), busy=ref(false), received=ref(0), now=ref(Date.now())
const name=ref('田间道路示教')
const query=ref('')
const fileInput=ref<HTMLInputElement|null>(null)
const processing=reactive<ProcessingParameters>({spacing:.1,min_spacing:.03,max_speed:1.5,max_gap:.5,smoothing:.03,allow_reverse:false})
const tracking=reactive<FollowingParameters>({controller_algorithm:'pp',lookahead:.5,speed:.1,max_angular:.5,completion_tolerance:.15,start_tolerance:.3,search_window:1,max_path_error:.5})
let poll:ReturnType<typeof setInterval>|undefined, clock:ReturnType<typeof setInterval>|undefined
let disposed=false,inflight=false,selection=0
const fresh=computed(()=>!!context.value && now.value-received.value<1500)
const controllerFresh=computed(()=>fresh.value && context.value?.status.fresh===true && (context.value.status.age_seconds??Infinity)*1000+now.value-received.value<1000)
const recording=computed(()=>context.value?.recording.state==='RECORDING')
const processingNow=computed(()=>context.value?.recording.state==='PROCESSING')
const active=computed(()=>['ACQUIRING','RUNNING','PAUSED'].includes(context.value?.status.state||''))
const actualControllerLabel=computed(()=>controllerFresh.value && ['RUNNING','PAUSED'].includes(context.value?.status.state||'')
  ? context.value?.status.controller_label : undefined)
const pendingControllerLabel=computed(()=>trackingControllerChoices[tracking.controller_algorithm])
const canEdit=computed(()=>controllerFresh.value && !recording.value && !processingNow.value && !active.value && !busy.value)
const canImport=computed(()=>fresh.value && !recording.value && !processingNow.value && !active.value && !busy.value)
const baseMode=computed(()=>safetyFresh.value && devices.value?.safety.mode==='base')
const followMode=computed(()=>safetyFresh.value && devices.value?.safety.mode==='following')
const reference=computed(()=>selected.value?.points||context.value?.reference||[])
const mapRaw=computed(()=>fresh.value&&(!selected.value||selected.value.id===context.value?.recording.id)?context.value?.raw||[]:[])
const mapTrack=computed(()=>!selected.value||selected.value.id===context.value?.loaded_id?context.value?.track||[]:[])
const filteredPaths=computed(()=>context.value?.paths.filter(path=>path.name.toLowerCase().includes(query.value.trim().toLowerCase()))||[])
const selectedLoaded=computed(()=>!!selected.value && selected.value.id===context.value?.loaded_id && selected.value.reference_fingerprint===context.value?.status.reference_fingerprint)
async function refresh(){
  if(inflight||disposed)return
  inflight=true;const started=Date.now()
  try{const data=await getFollowing();if(!disposed){context.value=data;received.value=started;now.value=Date.now();error.value=''}}
  catch(e){if(!disposed)error.value=e instanceof Error?e.message:'路径跟随状态查询失败'}
  finally{inflight=false}
}
async function operate(path:string,body?:object,method='POST'){
  if(busy.value)return false
  busy.value=true
  try{await followingRequest(path,body,method);ElMessage.success('请求已确认，请核对实际状态');return true}
  catch(e){ElMessage.error(e instanceof Error?e.message:'操作失败');return false}
  finally{busy.value=false;await refresh()}
}
async function choose(path:TaughtPath){
  const epoch=++selection
  if(path.state!=='READY'){selected.value=null;return}
  try{const value=await getTaughtPath(path.id);if(!disposed&&epoch===selection)selected.value=value}
  catch(e){ElMessage.error(e instanceof Error?e.message:'路径读取失败')}
}
async function rename(path:TaughtPath){
  try{const {value}=await ElMessageBox.prompt('输入新路径名称','重命名',{inputValue:path.name,inputValidator:v=>!!v?.trim()||'请输入名称'});const saved=await operate(`paths/${path.id}`,{name:value},'PATCH');if(saved&&selected.value?.id===path.id)selected.value.name=value}
  catch{/* canceled */}
}
async function remove(path:TaughtPath){
  try{await ElMessageBox.confirm('从路径库移除此路径？原始记录及既有任务数据保留。','删除示教路径',{type:'warning'});const saved=await operate(`paths/${path.id}`,undefined,'DELETE');if(saved&&selected.value?.id===path.id)selected.value=null}
  catch{/* canceled */}
}
async function recordStart(){if(!canEdit.value||!baseMode.value)return;selected.value=null;await operate('record/start',{name:name.value,processing:{...processing}})}
async function importFile(event:Event){
  const input=event.target as HTMLInputElement, file=input.files?.[0]
  if(!file)return
  if(!canImport.value){input.value='';return}
  busy.value=true
  const epoch=++selection
  const pathName=name.value.trim(), settings={...processing}
  try{
    if(!pathName)throw new Error('请先填写路径名称')
    if(file.size>2*1024*1024)throw new Error('文件超过 2 MiB，请减少输入点数')
    const content=await file.text()
    if(disposed)return
    const path=await importFollowingFile({name:pathName,filename:file.name,content,processing:settings})
    if(!disposed&&epoch===selection){selected.value=path;ElMessage.success('轨迹已导入路径库，请核对预览；加载和开始需单独操作')}
  }catch(e){if(!disposed)ElMessage.error(e instanceof Error?e.message:'轨迹导入失败')}
  finally{input.value='';busy.value=false;if(!disposed)await refresh()}
}
async function start(){
  if(!canEdit.value||!followMode.value||!selectedLoaded.value||devices.value?.safety.motion_inhibited)return
  try{await ElMessageBox.confirm('确认车辆在示教起点附近、道路可通行且已释放手柄控制权。确认后将对齐航向并自主行驶。','开始路径跟随',{type:'warning'});await operate('start',{...tracking})}
  catch{/* canceled */}
}
onMounted(()=>{void refresh();poll=setInterval(()=>{if(!document.hidden)void refresh()},500);clock=setInterval(()=>now.value=Date.now(),200)})
onUnmounted(()=>{disposed=true;selection++;clearInterval(poll);clearInterval(clock)})
</script>
<template>
  <div class="following-page">
    <h2>路径跟随</h2><p class="intro">手柄示教录制或导入坐标轨迹，保存道路路径后自主跟随。进入本页不会切换模式或启动车辆。</p>
    <ModeSelector />
    <el-alert v-if="error||!fresh" :title="error||'路径跟随状态未更新'" type="warning" :closable="false"/>
    <div class="following-layout">
      <el-card class="library" shadow="never">
        <template #header><strong>已保存路径</strong><el-button link @click="refresh">刷新</el-button></template>
        <p class="note">选择只预览；加载后需单独开始。录制失败的原始数据保留，可删除记录。</p>
        <el-input v-model="query" placeholder="按路径名称查询" clearable/>
        <el-empty v-if="!context?.paths.length" description="暂无示教记录" :image-size="50"/>
        <div v-for="path in filteredPaths" :key="path.id" class="path-row" :class="{selected:selected?.id===path.id}">
          <el-button link :disabled="busy||active||recording" @click="choose(path)">{{ path.name }}</el-button>
          <div>{{ taskText(path.state) }} · {{ path.created_at.slice(0,19).replace('T',' ') }}</div>
          <el-tag v-if="path.source_kind==='import'" size="small" type="info">文件导入</el-tag>
          <div v-if="path.statistics">{{ numberText(path.statistics.length_m) }} m · {{ path.statistics.path_points }} 点</div>
          <div v-if="path.reason" class="failure">{{ path.reason }}</div>
          <el-tag v-if="context?.loaded_id===path.id" size="small">当前已加载</el-tag>
          <div class="actions"><el-button size="small" :disabled="!canEdit" @click="rename(path)">重命名</el-button><el-button size="small" type="danger" plain :disabled="!canEdit" @click="remove(path)">删除</el-button></div>
        </div>
      </el-card>
      <div class="main-panels">
        <el-card shadow="never">
          <template #header><strong>1. 录制或导入路径</strong><el-tag>{{ taskText(context?.recording.state) }}</el-tag></template>
          <el-form label-position="top" class="parameters">
            <el-form-item label="路径名称"><el-input v-model="name" :maxlength="80" :disabled="recording||processingNow"/></el-form-item>
            <el-form-item label="重采样间距（m）"><el-input-number v-model="processing.spacing" :min=".03" :max="1" :step=".01" :precision="2" :disabled="recording||processingNow"/></el-form-item>
            <el-form-item label="最大平滑位移（m）"><el-input-number v-model="processing.smoothing" :min="0" :max=".1" :step=".01" :precision="2" :disabled="recording||processingNow"/></el-form-item>
            <el-form-item label="行驶方向"><el-checkbox v-model="processing.allow_reverse" :disabled="recording||processingNow||busy">录制、导入及跟随允许倒车</el-checkbox></el-form-item>
          </el-form>
          <div class="actions"><el-button type="primary" :disabled="!canEdit||!baseMode||!name.trim()" @click="recordStart">开始录制</el-button>
            <el-button type="success" :disabled="!recording||busy" @click="operate('record/stop')">结束录制并保存</el-button>
            <el-button :disabled="!recording||busy" @click="operate('record/cancel')">取消录制</el-button>
            <el-button :disabled="!canImport||!name.trim()" @click="fileInput?.click()">导入文件</el-button>
            <input ref="fileInput" type="file" accept=".csv,.json,.geojson" hidden @change="importFile"/></div>
          <p class="note">导入支持 CSV（表头 longitude,latitude，可选 yaw）、JSON 坐标点和 GeoJSON LineString，最多 2 MiB / 20000 点，至少 3 点及 0.5 m。坐标顺序就是行驶顺序；不是地块边界 Polygon。重采样、平滑及倒车选项同时用于导入；无 yaw 时按点序生成前进航向，有 yaw 时须为 ROS map 弧度。导入只保存并预览，不切换模式、不加载或行驶。</p>
          <p>采样 {{ context?.recording.samples||0 }} 个 · {{ numberText(context?.recording.elapsed_seconds,1) }} s</p>
          <p v-if="context?.recording.reason" class="failure">{{ context.recording.reason }}</p>
          <p class="note">基础待命模式 → 开始录制 → 按原有手柄方式行驶 → 手动停车 → 结束录制并保存。结束/取消录制只结束采样，不代替手柄停车。定位中断超过 0.5 s 会终止录制。请避免横向平移；差速底盘无法复现该动作。</p>
        </el-card>
        <el-card shadow="never">
          <template #header><strong>2. 路径加载与自主跟随</strong><el-tag>{{ controllerFresh?taskText(context?.status.state):'状态未更新' }}</el-tag></template>
          <p>预览：{{ selected?.name||'未选择' }}<span v-if="selectedLoaded"> · 参考路径已确认</span></p>
          <p v-if="actualControllerLabel" class="note">实际跟踪算法：{{ actualControllerLabel }}</p>
          <p v-else-if="active" class="note">等待本次任务的实际跟踪算法确认</p>
          <p v-else class="note">待执行跟踪算法：{{ pendingControllerLabel }}（点击“开始路径跟随”时生效）</p>
          <p v-if="selected?.import_source" class="note">来源：{{ selected.import_source.filename }} · {{ selected.import_source.coordinate_system }}。{{ selected.import_source.note }}。开始前请核对道路可通行性。</p>
          <el-form label-position="top" class="parameters">
            <el-form-item label="路径跟踪算法"><el-select v-model="tracking.controller_algorithm" :disabled="active||busy"><el-option v-for="(label,key) in trackingControllerChoices" :key="key" :label="label" :value="key"/></el-select></el-form-item>
            <el-form-item label="跟随速度（m/s）"><el-input-number v-model="tracking.speed" :min=".02" :max="1" :step=".05" :disabled="active"/></el-form-item>
            <el-form-item v-if="tracking.controller_algorithm==='pp'" label="PP基础前视距离（m）"><el-input-number v-model="tracking.lookahead" :min=".1" :max="3" :step=".1" :disabled="active"/></el-form-item>
            <el-form-item label="最大角速度（rad/s）"><el-input-number v-model="tracking.max_angular" :min=".05" :max="1" :step=".05" :disabled="active"/></el-form-item>
            <el-form-item label="最大路径偏离（m）"><el-input-number v-model="tracking.max_path_error" :min=".1" :max="2" :step=".1" :disabled="active"/></el-form-item>
            <el-form-item label="起点允许距离（m）"><el-input-number v-model="tracking.start_tolerance" :min=".05" :max=".5" :step=".05" :disabled="active"/></el-form-item>
            <el-form-item label="终点到达阈值（m）"><el-input-number v-model="tracking.completion_tolerance" :min=".03" :max=".5" :step=".05" :disabled="active"/></el-form-item>
            <el-form-item label="路径进度搜索窗口（m）"><el-input-number v-model="tracking.search_window" :min=".1" :max="5" :step=".1" :disabled="active"/></el-form-item>
          </el-form>
          <p v-if="tracking.controller_algorithm==='pp'" class="note">PP实际前视随速度和前方路径曲率自动调整，限定0.65～1.05 m；上方设置的是基础前视值，开始跟随后生效。</p>
          <div class="actions"><el-button :disabled="!canEdit||!followMode||!selected" @click="operate(`paths/${selected!.id}/load`)">加载此路径</el-button>
            <el-button type="success" :disabled="!canEdit||!followMode||!selectedLoaded||!safetyFresh||devices?.safety.motion_inhibited" @click="start">开始路径跟随</el-button>
            <el-button :disabled="!controllerFresh||context?.status.state!=='RUNNING'||busy" @click="operate('task/pause')">暂停</el-button>
            <el-button :disabled="!controllerFresh||context?.status.state!=='PAUSED'||busy" @click="operate('task/resume')">继续</el-button>
            <el-button type="danger" plain :disabled="busy" @click="operate('task/cancel')">取消跟随</el-button></div>
          <p class="note">先用手柄将车辆移到示教起点附近，释放控制权，切换“路径跟随”模式并加载。路径跟随不自动从远处接近起点；暂停后继续会重新对齐，急停/故障后不会自动恢复。前方雷达保护沿用现有开关，倒车没有后向雷达保护。</p>
          <p v-if="context?.status.reason" class="failure">{{ context.status.reason }}</p>
        </el-card>
        <el-card shadow="never">
          <template #header><strong>参考路径与实际轨迹</strong></template>
          <div class="metrics"><span>横向误差：{{ controllerFresh && context?.error ? numberText(context.status.lateral_error,3) : '--' }} m</span>
            <span>航向误差：{{ controllerFresh && context?.error ? numberText((context.error.heading*180/Math.PI),1) : '--' }}°</span>
            <span>进度：{{ controllerFresh?numberText(context?.status.progress):'--' }} / {{ numberText(context?.status.length) }} m</span>
            <span>阶段：{{ ({TRACKING:'路径跟随',ALIGNING:'航向对齐',IDLE:'待命'} as Record<string,string>)[context?.status.phase||'IDLE']||context?.status.phase }}</span></div>
          <FollowingMap :reference="reference" :raw="mapRaw" :track="mapTrack" :pose="fresh?context?.pose||null:null"/>
          <p v-if="context?.active_run" class="note">本次记录：{{ context.active_run.id }} · {{ taskText(context.active_run.state) }}
            <a :href="apiUrl(`/api/runs/${context.active_run.id}/download`)" target="_blank" rel="noopener">下载运行数据</a></p>
        </el-card>
      </div>
    </div>
  </div>
</template>
<style scoped>
.following-page h2{margin:0 0 8px}.intro,.note{color:#718178;font-size:13px;line-height:1.7}.following-layout{display:grid;grid-template-columns:270px minmax(0,1fr);gap:18px;margin-top:18px;align-items:start}.main-panels{display:grid;gap:18px;min-width:0}.parameters{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:0 15px}.actions,.metrics{display:flex;gap:10px;flex-wrap:wrap;align-items:center}.actions .el-button{margin:0}.metrics{margin-bottom:14px;color:#3e5d4c;font-size:14px}.path-row{padding:13px 8px;border-bottom:1px solid #e8eee9;font-size:12px;color:#718178;line-height:1.8}.path-row.selected{background:#f0f8f3}.path-row .actions{margin-top:8px}.failure{color:#bc3e3e;overflow-wrap:anywhere}:deep(.el-card__header){display:flex;justify-content:space-between;align-items:center;gap:8px}@media(max-width:1050px){.following-layout{grid-template-columns:1fr}.parameters{grid-template-columns:repeat(auto-fit,minmax(150px,1fr))}}
</style>
