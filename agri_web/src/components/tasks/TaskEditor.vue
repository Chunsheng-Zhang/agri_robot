<script setup lang="ts">
import { computed, ref, watch, onUnmounted } from 'vue'
import { ElMessage } from 'element-plus'
import { useRealtime } from '../../composables/useRealtime'
import { useWorkspace } from '../../composables/useWorkspace'
import { saveMission, previewMission, missionAction, downloadJson, getSavedPlanPath, loadSavedPlan, savePlannedPath, mapToWgs84 } from '../../api/workspace'
import type { PathPoint } from '../../api/robot'
import type { MissionDocument, MissionInput, MissionMode, GeoPoint, Preview, Projection, SavedPlanPath } from '../../api/workspace'
import { coverageQualityFields, parameterFields } from './taskParameters'
import { defaultTaskPoints } from './defaultTaskPoints'
import type { ParameterField } from './taskParameters'
import FieldMap from '../dashboard/FieldMap.vue'
import CoverageQuality from './CoverageQuality.vue'

interface Row { longitude: string; latitude: string; dwell_time: number }
const props = defineProps<{ mission?: MissionDocument | null; planId?: string; kind: MissionMode; defaults: Readonly<Record<string, number | string | boolean>> }>()
const emit = defineEmits<{ saved: [value: MissionDocument]; planned: []; pathSaved: [missionId: string, planId: string] }>()
const { devices, isFresh, safetyFresh, refresh } = useRealtime()
const { context, refresh: refreshWorkspace } = useWorkspace()
const name = ref(''), parameters = ref<Record<string, number | string | boolean>>({}), rows = ref<Row[]>([])
const rings = ref<Row[][]>([]), ringIndex = ref(0), id = ref<string>(), revision = ref<number>()
const origin = ref<Projection>(), preview = ref<Preview | null>(null), error = ref(''), busy = ref(false), previewBusy = ref(false)
const entrySource=ref<'coordinates'|'map'>('coordinates'),entryLongitude=ref(''),entryLatitude=ref('')
const entryLocal=ref<PathPoint|null>(null),entryPicking=ref(false),entryLoading=ref(false)
const mapContainer=ref<HTMLElement|null>(null)
let entryGeneration=0
watch([entryLongitude,entryLatitude],()=>{entryLocal.value=null},{flush:'sync'})
watch(entrySource,value=>{if(value==='coordinates')entryPicking.value=false})
const minimumHeadland = computed(() => Math.hypot(
  Number(parameters.value.robot_length ?? 0.45) / 2 + Math.abs(Number(parameters.value.rotation_center_x ?? 0)),
  Number(parameters.value.robot_width ?? 0.2) / 2 + Math.abs(Number(parameters.value.rotation_center_y ?? 0)),
) + Number(parameters.value.rotation_safety_margin ?? 0.05) + 0.0001)
const lastSaved = ref(''), fileInput = ref<HTMLInputElement | null>(null)
const savedPath = ref<SavedPlanPath | null>(null), savedPathLoading = ref(false), savedPathError = ref('')
let savedPathGeneration = 0
let timer: ReturnType<typeof setTimeout> | undefined
let disposed = false
const copy = <T,>(value: T): T => JSON.parse(JSON.stringify(value))
function copyParameters(value: Readonly<Record<string, number | string | boolean>>) {
  const result = copy(value) as Record<string, number | string | boolean>
  if (props.kind === 'coverage' && typeof result.allow_reverse !== 'boolean') result.allow_reverse = false
  delete result.optimization_min_coverage
  delete result.optimization_turns
  delete result.optimization_patterns
  delete result.optimization_headland_min
  delete result.optimization_headland_max
  for (const key of ['optimization_headland_mode', 'optimization_search_angle', 'optimization_weight_coverage',
    'optimization_weight_overlap', 'optimization_weight_length', 'optimization_weight_energy',
    'energy_forward', 'energy_turn', 'energy_reverse', 'energy_switch',
    'optimization_max_candidates', 'optimization_time_budget']) delete result[key]
  if (result.path_continuity === 'CONTINUOUS') result.path_continuity = 'DUBINS_CC'
  if (result.path_continuity === 'DISCONTINUOUS') result.path_continuity = 'DUBINS'
  return result
}
const blank = (): Row => ({ longitude: '', latitude: '', dwell_time: Number(parameters.value.dwell_time ?? 2) })
const toRows = (points: readonly GeoPoint[]): Row[] => points.map(p => ({ longitude: String(p.longitude), latitude: String(p.latitude), dwell_time: p.dwell_time ?? 2 }))
const fingerprint = computed(() => JSON.stringify([name.value, rows.value, rings.value, parameters.value,
  entryLongitude.value,entryLatitude.value,entrySource.value]))
const dirty = computed(() => !id.value || fingerprint.value !== lastSaved.value)
const editableRows = computed(() => props.kind === 'waypoint' ? rows.value : rings.value[ringIndex.value] || [])
const active = computed(() => context.value?.active_run)
const planReady = computed(() => !dirty.value && showLive.value && active.value?.mission_id === id.value
  && active.value?.mission_revision === revision.value && active.value?.stage === 'planning' && active.value?.state === 'READY')
const planSaved = computed(() => planReady.value && active.value?.saved_plan_version === 1)
const showLive = computed(() => !dirty.value && active.value?.mission_id === id.value && active.value?.mission_revision === revision.value
  && (!props.planId || props.planId === active.value?.id || props.planId === active.value?.source_plan_id))
const savedReference = computed(() => !dirty.value && !showLive.value ? savedPath.value : null)
const displayRun = computed(() => savedReference.value?.run || (showLive.value ? active.value || null : null))
const displayPreview = computed(() => savedReference.value?.preview || preview.value)
const executingThis = computed(() => active.value?.mission_id === id.value && ['PLANNING', 'ACQUIRING', 'RUNNING', 'PAUSED'].includes(active.value?.state || ''))
const controlsIdle = computed(() => safetyFresh.value && !devices.value?.safety.owner &&
  !['waypoint', 'tracking', 'following', 'planning'].some(key => {
    const status = devices.value?.[key as 'waypoint' | 'tracking' | 'following' | 'planning']
    return isFresh(status) && ['PLANNING', 'ACQUIRING', 'RUNNING', 'PAUSED'].includes(status?.state || '')
  }))

function loadBoundary(data: Record<string, unknown> | null) {
  if (!data) return [[blank(), blank(), blank(), blank()]]
  const geom = data.type === 'Feature' ? data.geometry as Record<string, unknown> : data
  if (geom.type === 'Polygon') {
    return (geom.coordinates as number[][][]).map(ring => {
      const points = ring.map(p => ({ longitude: String(p[0]), latitude: String(p[1]), dwell_time: 0 }))
      if (points.length > 1 && points[0]?.longitude === points.at(-1)?.longitude && points[0]?.latitude === points.at(-1)?.latitude) points.pop()
      return points
    })
  }
  const raw = (data.rings || [data.points || []]) as GeoPoint[][]
  return raw.map(toRows)
}
function reset() {
  ++entryGeneration;entryPicking.value=false;entryLoading.value=false
  const mission = props.mission
  name.value = mission?.name || (props.kind === 'waypoint' ? '新多点导航任务' : '新农田覆盖任务')
  parameters.value = copyParameters({ ...props.defaults, ...mission?.parameters })
  rows.value = toRows(mission ? copy(mission.waypoints) : defaultTaskPoints)
  rings.value = mission
    ? loadBoundary(copy(mission.boundary) as Record<string, unknown> | null)
    : [toRows(defaultTaskPoints)]
  ringIndex.value = 0; id.value = mission?.id; revision.value = mission?.revision
  origin.value = mission ? copy(mission.projection) : undefined
  preview.value = mission ? copy(mission.preview) : null
  entrySource.value=mission?.entry?.source||'coordinates'
  entryLongitude.value=mission?.entry?String(mission.entry.longitude):''
  entryLatitude.value=mission?.entry?String(mission.entry.latitude):''
  entryLocal.value=mission?.preview.entry||null
  error.value = ''; lastSaved.value = fingerprint.value
  clearTimeout(timer)
  if (!mission || (props.kind === 'coverage' && !preview.value?.inset_polygons))
    timer = setTimeout(() => void buildPreview(true), 650)
}
watch([() => props.mission?.id, () => props.mission?.revision, () => props.kind], reset, { immediate: true })
function parsePoints(points: Row[]): GeoPoint[] {
  return points.map((p, index) => {
    if (!p.longitude.trim() || !p.latitude.trim() || !Number.isFinite(Number(p.longitude)) || !Number.isFinite(Number(p.latitude))) throw new Error(`第 ${index + 1} 点需要有效经纬度`)
    return { longitude: Number(p.longitude), latitude: Number(p.latitude), dwell_time: p.dwell_time }
  })
}
function payload(): MissionInput {
  let boundary: object | null = null
  if (props.kind === 'coverage') {
    boundary = { type: 'Polygon', coordinates: rings.value.map(ring => {
      const points = parsePoints(ring).map(p => [p.longitude, p.latitude])
      return points.length ? [...points, points[0]] : []
    }) }
  }
  return { name: name.value, mode: props.kind, parameters: copy(parameters.value),
    ...(props.kind==='coverage'?{entry:entryInput()}:{}),
    waypoints: props.kind === 'waypoint' ? parsePoints(rows.value) : [], boundary,
    ...(revision.value ? { revision: revision.value } : {}), ...(origin.value ? { import_origin: origin.value } : {}) }
}
function entryInput() {
  if(!entryLongitude.value.trim()&&!entryLatitude.value.trim())return null
  const longitude=Number(entryLongitude.value),latitude=Number(entryLatitude.value)
  if(!entryLongitude.value.trim()||!entryLatitude.value.trim()||!Number.isFinite(longitude)||!Number.isFinite(latitude))
    throw Error('入口需要完整且有效的经纬度；也可以清除入口后按原方式规划')
  if(longitude < -180 || longitude > 180 || latitude < -80 || latitude > 84)throw Error('入口经纬度超出 WGS84 / UTM 有效范围')
  return {longitude,latitude,source:entrySource.value}
}
function clearEntry(){++entryGeneration;entryLongitude.value='';entryLatitude.value='';entryLocal.value=null;entryPicking.value=false;entryLoading.value=false}
function pickEntry(){if(busy.value||executingThis.value)return;entrySource.value='map';entryPicking.value=true;mapContainer.value?.scrollIntoView({behavior:'smooth',block:'center'})}
async function selectEntry(point:PathPoint){
  if(busy.value||executingThis.value)return
  const projection=origin.value||context.value?.origin
  if(!projection){ElMessage.error('固定坐标原点尚未确认');return}
  const generation=++entryGeneration,version=fingerprint.value
  entryLoading.value=true
  try{
    const result=await mapToWgs84(point.x,point.y,projection)
    if(disposed||generation!==entryGeneration||version!==fingerprint.value)return
    entrySource.value='map';entryLongitude.value=String(result.longitude);entryLatitude.value=String(result.latitude)
    entryLocal.value={x:result.x,y:result.y};entryPicking.value=false
  }catch(value){if(!disposed&&generation===entryGeneration)ElMessage.error(value instanceof Error?value.message:'入口选点失败')}
  finally{if(generation===entryGeneration)entryLoading.value=false}
}
async function buildPreview(silent = false) {
  const version = fingerprint.value
  try {
    const data = payload(); previewBusy.value = true
    const result = await previewMission(data)
    if (!disposed && version === fingerprint.value) { preview.value = result; entryLocal.value=result.entry||null; error.value = '' }
  } catch (value) {
    if (!disposed && version === fingerprint.value) {
      error.value = value instanceof Error ? value.message : '预览失败'
      preview.value=props.kind==='coverage'&&preview.value&&error.value.includes('入口')
        ? {...preview.value,entry:null,entry_anchor:null}:null
    }
    if (!silent) ElMessage.error(error.value)
  } finally { previewBusy.value = false }
}
watch(fingerprint, () => { clearTimeout(timer); timer = setTimeout(() => void buildPreview(true), 650) })
function move(index: number, direction: number) {
  const list = editableRows.value, other = index + direction
  if (other >= 0 && other < list.length) [list[index], list[other]] = [list[other]!, list[index]!]
}
function capture() {
  const gnss = devices.value?.gnss
  if (!isFresh(gnss) || gnss?.latitude == null || gnss.longitude == null) return
  editableRows.value.push({ longitude: String(gnss.longitude), latitude: String(gnss.latitude), dwell_time: Number(parameters.value.dwell_time ?? 2) })
}
function numberValue(field: ParameterField) {
  const value = Number(parameters.value[field.key]); return field.degrees ? value * 180 / Math.PI : value
}
function setNumber(field: ParameterField, value: number | undefined) {
  if (value != null) parameters.value[field.key] = field.degrees ? value * Math.PI / 180 : value
}
async function save() {
  if (busy.value) return
  busy.value = true
  try {
    const result = await saveMission(payload(), id.value)
    id.value = result.id; revision.value = result.revision; origin.value = result.projection
    preview.value = result.preview; lastSaved.value = fingerprint.value
    emit('saved', result); await refreshWorkspace(); ElMessage.success('任务已保存，尚未开始运动')
  } catch (value) { ElMessage.error(value instanceof Error ? value.message : '保存失败') }
  finally { busy.value = false }
}
async function operate(action: 'plan' | 'start' | 'optimize') {
  if (!id.value || dirty.value || busy.value) return
  busy.value = true
  try { await missionAction(id.value, action); if (action !== 'start') emit('planned'); ElMessage.success(action === 'start' ? '开始请求已确认，请查看运行反馈' : '正在生成覆盖路径，请等待结果') }
  catch (value) { ElMessage.error(value instanceof Error ? value.message : '任务请求失败') }
  finally { busy.value = false; await Promise.all([refresh(), refreshWorkspace()]) }
}
async function importFile(event: Event) {
  const file = (event.target as HTMLInputElement).files?.[0]
  if (!file) return
  try {
    if (file.size > 2 * 1024 * 1024) throw new Error('文件超过 2 MB，请减少输入点数')
    const text = await file.text()
    if (file.name.endsWith('.csv')) {
      if (props.kind !== 'waypoint') throw new Error('CSV 航点请在“多点导航”栏目中导入')
      const [header, ...lines] = text.trim().split(/\r?\n/), columns = header!.split(',').map(p => p.trim())
      const lon = columns.indexOf('longitude'), lat = columns.indexOf('latitude'), dwell = columns.indexOf('dwell_time')
      if (lon < 0 || lat < 0) throw new Error('CSV 表头需要 longitude,latitude，可选 dwell_time')
      rows.value = lines.filter(Boolean).map(line => { const values = line.split(','); return { longitude: values[lon]!.trim(), latitude: values[lat]!.trim(), dwell_time: dwell < 0 ? 2 : Number(values[dwell]) } })
    } else {
      const data = JSON.parse(text), task = data.task || data
      if (task.mode) {
        if (task.mode !== props.kind) throw new Error('任务类型与当前栏目不符，请到对应的多点导航或全覆盖栏目导入')
        name.value = task.name; parameters.value = copyParameters({ ...props.defaults, ...task.parameters })
        ++entryGeneration;entryLongitude.value=task.entry?String(task.entry.longitude):'';entryLatitude.value=task.entry?String(task.entry.latitude):'';entrySource.value=task.entry?.source||'coordinates';entryPicking.value=false;entryLoading.value=false
        rows.value = toRows(task.waypoints || []); rings.value = loadBoundary(task.boundary)
        origin.value = data.projection || task.projection
      } else if (props.kind === 'coverage') rings.value = loadBoundary(data)
      else if (Array.isArray(task.waypoints)) rows.value = toRows(task.waypoints)
      else throw new Error('不支持的任务文件格式')
    }
    id.value = undefined; revision.value = undefined; ringIndex.value = 0; lastSaved.value = ''
    await buildPreview(); ElMessage.success('已导入为新任务，请核对后保存')
  } catch (value) { ElMessage.error(value instanceof Error ? value.message : '导入失败') }
  finally { (event.target as HTMLInputElement).value = '' }
}
function exportTask() {
  try { downloadJson(`${name.value || 'mission'}.json`, { schema_version: 1, task: payload(), projection: origin.value || context.value?.origin }) }
  catch (value) { ElMessage.error(value instanceof Error ? value.message : '导出失败') }
}
async function fetchSavedPath() {
  const generation = ++savedPathGeneration, mission = props.mission, plan = props.planId
  savedPath.value = null; savedPathError.value = ''; savedPathLoading.value = false
  if (props.kind !== 'coverage' || !mission || !plan || plan === active.value?.id || plan === active.value?.source_plan_id) return
  savedPathLoading.value = true
  try {
    const result = await getSavedPlanPath(mission.id, plan)
    if (disposed || generation !== savedPathGeneration) return
    if (result.run.id !== plan || result.run.mission_id !== mission.id || result.run.mission_revision !== mission.revision
      || result.frame_id !== 'map' || !result.fingerprint || result.count !== result.points.length
      || !result.points.every(p => Number.isFinite(p.x) && Number.isFinite(p.y))
      || (['latitude', 'longitude', 'epsg'] as const).some(k => result.projection[k] !== mission.projection[k]))
      throw new Error('已保存路径与当前任务、版本或坐标原点不一致')
    savedPath.value = result
  } catch (value) {
    if (!disposed && generation === savedPathGeneration) savedPathError.value = value instanceof Error ? value.message : '读取已保存路径失败'
  } finally { if (generation === savedPathGeneration) savedPathLoading.value = false }
}
async function activateSavedPath() {
  if (!id.value || !props.planId || !savedReference.value || dirty.value || busy.value || !controlsIdle.value || devices.value?.safety.mode !== 'coverage') return
  busy.value = true
  try {
    await loadSavedPlan(id.value, props.planId)
    ElMessage.success('路径已加载，尚未开始行驶；确认安全状态后可开始覆盖')
  } catch (value) { ElMessage.error(value instanceof Error ? value.message : '加载路径失败') }
  finally { busy.value = false; await Promise.all([refresh(), refreshWorkspace()]) }
}
async function savePath() {
  if (props.kind !== 'coverage' || !id.value || !planReady.value || planSaved.value || busy.value) return
  const missionId = id.value, planId = active.value!.id
  busy.value = true
  try {
    await savePlannedPath(missionId, planId)
    emit('pathSaved', missionId, planId)
    ElMessage.success('规划路径已保存到左侧任务下，尚未开始行驶')
  } catch (value) { ElMessage.error(value instanceof Error ? value.message : '保存规划路径失败') }
  finally { busy.value = false; await refreshWorkspace() }
}
watch([() => props.planId, () => props.mission?.id, () => props.mission?.revision], () => void fetchSavedPath(), { immediate: true })
onUnmounted(() => { disposed = true; ++savedPathGeneration; ++entryGeneration; clearTimeout(timer) })
</script>

<template>
  <div class="mission-editor">
    <el-card shadow="never">
      <template #header><div class="editor-heading"><strong>{{ id ? '编辑' : '创建' }}{{ kind === 'waypoint' ? '多点导航任务' : '覆盖任务' }}</strong><el-tag :type="dirty ? 'warning' : 'success'">{{ dirty ? '有未保存内容' : `已保存 · v${revision}` }}</el-tag></div></template>
      <el-form label-position="top" :disabled="busy || executingThis">
        <el-form-item label="任务名称"><el-input v-model="name" maxlength="80" placeholder="例如：东侧试验田第一轮" /></el-form-item>
        <div v-if="kind==='coverage'" class="entry-settings">
          <strong>作业入口设置</strong>
          <el-radio-group v-model="entrySource" aria-label="作业入口输入方式"><el-radio-button value="coordinates">输入经纬度</el-radio-button><el-radio-button value="map">地图选择入口</el-radio-button></el-radio-group>
          <div class="parameter-grid">
            <el-form-item label="入口经度（WGS84 十进制度）"><el-input v-model="entryLongitude" inputmode="decimal" placeholder="例如 117.0214"/></el-form-item>
            <el-form-item label="入口纬度（WGS84 十进制度）"><el-input v-model="entryLatitude" inputmode="decimal" placeholder="例如 36.6468"/></el-form-item>
          </div>
          <el-button v-if="entrySource==='map'" :loading="entryLoading" @click="pickEntry">{{ entryPicking?'请在下方地图点击入口':'在地图上选入口' }}</el-button>
          <el-button @click="clearEntry">清除入口</el-button>
          <p class="hint">两种方式设置同一个固定入口，可拖动紫色入口标记调整。保存任务后，“生成覆盖路径”和“一键优化规划”都据此选择起点；清除入口则沿用原规则。入口仅用于选择规划起点。开始后仍先单点导航到绿色覆盖起点、对齐再跟踪，不要求先经过入口。</p>
        </div>
        <div class="editor-toolbar">
          <el-button @click="editableRows.push(blank())">添加{{ kind === 'waypoint' ? '目标点' : '角点' }}</el-button>
          <el-button :disabled="!isFresh(devices?.gnss)" @click="capture">采集当前 RTK 点</el-button>
          <el-button @click="fileInput?.click()">导入文件</el-button><el-button @click="exportTask">导出 JSON</el-button>
          <input ref="fileInput" type="file" :accept="kind === 'waypoint' ? '.json,.csv' : '.json,.geojson'" hidden @change="importFile" />
        </div>
        <div v-if="kind === 'coverage'" class="ring-select">
          <el-select v-model="ringIndex" aria-label="边界环"><el-option v-for="(_, index) in rings" :key="index" :value="index" :label="index === 0 ? '农田外边界' : `内部禁入区域 ${index}`" /></el-select>
          <el-button @click="rings.push([blank(), blank(), blank()]); ringIndex = rings.length - 1">添加禁入区域</el-button>
          <el-button v-if="ringIndex > 0" type="danger" plain @click="rings.splice(ringIndex, 1); ringIndex = 0">删除该区域</el-button>
        </div>
        <el-table :data="editableRows" size="small" max-height="340">
          <el-table-column type="index" label="点序" width="55" />
          <el-table-column label="经度（WGS84）" min-width="160"><template #default="{ row }"><el-input v-model="row.longitude" inputmode="decimal" placeholder="经度" /></template></el-table-column>
          <el-table-column label="纬度（WGS84）" min-width="160"><template #default="{ row }"><el-input v-model="row.latitude" inputmode="decimal" placeholder="纬度" /></template></el-table-column>
          <el-table-column v-if="kind === 'waypoint'" label="停留（s）" width="145"><template #default="{ row }"><el-input-number v-model="row.dwell_time" :min="0" :max="600" size="small" /></template></el-table-column>
          <el-table-column label="调整" width="148"><template #default="{ $index }"><el-button link @click="move($index, -1)">上移</el-button><el-button link @click="move($index, 1)">下移</el-button><el-button link type="danger" @click="editableRows.splice($index, 1)">删除</el-button></template></el-table-column>
        </el-table>
        <p class="hint">{{ kind === 'coverage' ? '按边界顺序输入，至少 3 个不同角点；内部禁入区域作为孔洞处理。' : '按实际行驶顺序输入目标点，每个点可单独设置停留时间。' }}坐标统一使用项目固定原点，不按地块重新归零。</p>
        <el-divider content-position="left">{{ kind === 'coverage' ? '规划与路径跟踪参数' : '多点导航参数' }}</el-divider>
        <div class="parameter-grid">
          <el-form-item v-for="field in parameterFields[kind]" :key="field.key" :label="`${field.label}${field.unit ? '（' + field.unit + '）' : ''}`">
            <el-checkbox v-if="field.toggle" v-model="parameters[field.key]">允许倒车作业</el-checkbox>
            <el-select v-else-if="field.choices" v-model="parameters[field.key]"><el-option v-for="(label, value) in field.choices" :key="value" :label="label" :value="value" /></el-select>
            <el-input-number v-else :model-value="numberValue(field)" :min="field.min" :max="field.max" :step="field.step || 0.1" :precision="3" controls-position="right" @update:model-value="setNumber(field, $event)" />
            <small v-if="field.hint">{{ field.hint }}</small>
          </el-form-item>
        </div>
        <el-alert v-if="error" :title="error" type="warning" :closable="false" />
        <div class="editor-toolbar actions">
          <el-button :loading="previewBusy" @click="buildPreview()">{{ kind === 'waypoint' ? '检查目标点' : '检查地块边界' }}</el-button>
          <el-button type="primary" :loading="busy" @click="save">保存任务</el-button>
          <el-button v-if="kind === 'coverage'" type="success" plain :disabled="dirty || busy || !controlsIdle || devices?.safety.mode !== 'coverage'" @click="operate('plan')">生成覆盖路径</el-button>
          <el-button v-if="kind === 'coverage'" type="primary" plain :loading="busy" :disabled="!planReady || planSaved || busy" @click="savePath">{{ planSaved ? '规划路径已保存' : '保存规划路径' }}</el-button>
          <el-button type="success" :disabled="dirty || busy || !controlsIdle || devices?.safety.mode !== kind || devices?.safety.motion_inhibited !== false || devices?.safety.localization_valid !== true || (kind === 'coverage' && !planReady)" @click="operate('start')">{{ kind === 'coverage' ? '前往起点并开始覆盖' : '开始多点导航' }}</el-button>
        </div>
        <p v-if="kind === 'coverage'" class="hint">先保存任务、选择全覆盖模式并生成路径。要留作以后复用，请点击“保存规划路径”，左侧会显示该路径；以后选择并加载后再明确开始。当前刚生成的就绪路径也可直接开始。接近起点采用直达导航，请先确认接近通道可通行。</p>
        <p v-else class="hint">先保存，再选择多点导航模式，定位与安全状态有效后明确开始任务。</p>
      </el-form>
    </el-card>
    <el-card v-if="kind === 'coverage' && planId" shadow="never" class="saved-path-card">
      <template #header><strong>已保存覆盖路径</strong></template>
      <p v-if="savedPathLoading" class="hint">正在读取完整路径…</p>
      <p v-if="dirty" class="hint">任务有未保存修改；保存并重新规划后才能使用新版本路径。</p>
      <el-alert v-if="savedPathError" :title="savedPathError" type="warning" :closable="false" />
      <template v-if="savedReference">
        <p class="hint">{{ new Date(savedReference.run.created_at).toLocaleString('zh-CN') }} · {{ savedReference.count }}个路径点。当前为已保存路径预览，加载后才能开始覆盖。</p>
        <el-button type="primary" :loading="busy" :disabled="dirty || !controlsIdle || active?.state === 'PLANNING' || devices?.safety.mode !== 'coverage'" @click="activateSavedPath">加载此路径</el-button>
        <p class="hint">加载不启动车辆；再次执行从该路径起点开始，并保存到新的运行记录。</p>
      </template>
      <p v-else-if="showLive" class="hint">此路径已加载到当前核心，按上方任务与安全状态操作。</p>
      <el-button v-if="savedPathError" plain size="small" @click="fetchSavedPath">重试读取</el-button>
    </el-card>
    <el-card v-if="kind === 'coverage'" shadow="never" class="smart-card">
      <template #header><strong>智能规划策略</strong></template>
      <p>固定原地转向：停车等待 → 原地对齐 → 前进行驶；以往复式作业为基础，必要时局部优化行访问顺序。</p>
      <p>机具有效作业宽度 {{ Number(parameters.operation_width ?? 0.8).toFixed(3) }} m，由上方参数人工设置，规划与评价共用。</p>
      <p>最小安全地头 <strong>{{ minimumHeadland.toFixed(3) }} m</strong>，由车辆长宽、旋转中心和安全余量计算，不搜索人工地头宽度。</p>
      <p>自动搜索 0～180° 作业方向，先粗搜再细化；固定权重：覆盖率最大化 0.5、重复覆盖最小化 0.3、总长度最小化 0.2。</p>
      <p class="hint">总长度包含作业行和换行连接。车辆尺寸须包含旋转时的实际外伸，默认车长及余量为初值。完整执行预定角度与行顺序比较，顶部显示进度，可人工取消；不保证全局最优。</p>
      <div class="editor-toolbar">
        <el-button type="primary" plain :loading="busy" :disabled="!dirty" @click="save">保存任务</el-button>
        <el-button type="success" :loading="busy" :disabled="dirty || !controlsIdle || devices?.safety.mode !== 'coverage'" @click="operate('optimize')">一键优化规划</el-button>
      </div>
      <p class="hint">上方条带顺序、指定角度、地头预留、最小转弯半径和曲线形式仍供手工“生成覆盖路径”使用；本策略固定原地转向和安全地头。规划不会自动开始行驶。</p>
    </el-card>
    <CoverageQuality v-if="kind === 'coverage'" :run="displayRun">
      <template #parameters>
        <el-form label-position="top" :disabled="busy || executingThis" class="quality-parameters">
          <div class="parameter-grid">
            <el-form-item v-for="field in coverageQualityFields" :key="field.key" :label="`${field.label}（${field.unit}）`">
              <el-input-number :model-value="numberValue(field)" :min="field.min" :max="field.max" :step="field.step" :precision="3" controls-position="right" @update:model-value="setNumber(field, $event)" />
              <small>{{ field.hint }}</small>
            </el-form-item>
          </div>
          <p class="hint">评价参数与当前任务一并保存；修改后请保存任务并重新生成覆盖路径，结果才会采用新参数。</p>
          <el-button type="primary" plain :disabled="!dirty" :loading="busy" @click="save">保存任务（含评价参数）</el-button>
        </el-form>
      </template>
    </CoverageQuality>
    <div v-if="preview" class="preview-summary">{{ kind === 'coverage' ? `地块面积 ${(preview.area || 0).toFixed(2)} m² · 周界 ${preview.length.toFixed(2)} m` : `目标点 ${preview.points.length} 个 · 相邻点连线长 ${preview.length.toFixed(2)} m` }} · map 米制坐标</div>
    <div ref="mapContainer">
      <FieldMap :preview="displayPreview" :show-live-path="showLive" :saved-reference="savedReference" :coverage-mode="kind === 'coverage'"
        :entry-point="kind==='coverage'?entryLocal:null" :entry-picking="entryPicking&&!busy&&!executingThis"
        :entry-editable="kind==='coverage'&&!busy&&!executingThis" :entry-plan="!dirty?displayRun?.entry:null"
        @entry-selected="selectEntry" :coverage-run="kind === 'coverage' ? displayRun : null" />
    </div>
  </div>
</template>

<style scoped>
.mission-editor { min-width: 0; }.editor-heading { display: flex; justify-content: space-between; align-items: center; gap: 10px; }
.editor-toolbar, .ring-select { display: flex; flex-wrap: wrap; align-items: center; gap: 8px; margin: 14px 0; }.editor-toolbar .el-button, .ring-select .el-button { margin: 0; }.ring-select .el-select { width: 190px; }
.parameter-grid { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 0 18px; }.parameter-grid .el-input-number { width: 100%; }.parameter-grid small { display: block; color: #829087; line-height: 1.5; padding-top: 5px; }
.quality-parameters { padding-bottom: 18px; margin-bottom: 18px; border-bottom: 1px solid #e6ece8; }.quality-parameters .parameter-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
.smart-card { margin-top: 20px; } summary { cursor: pointer; color: #376e4b; padding: 8px 0; }
.saved-path-card { margin-top: 20px; }
.hint { color: #7c8a81; font-size: 12px; line-height: 1.8; }.actions { margin-top: 20px; }.preview-summary { margin: 18px 0 10px; padding: 14px; background: #eaf3ec; color: #376e4b; border-radius: 8px; font-size: 13px; }
@media (max-width: 1100px) { .parameter-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); } }@media (max-width: 620px) { .parameter-grid, .quality-parameters .parameter-grid { grid-template-columns: 1fr; } }
</style>

<style scoped>
.entry-settings{padding:14px;border:1px solid #e0e7e2;background:#f7faf7;border-radius:8px;margin-bottom:18px}.entry-settings strong{display:block;margin-bottom:10px}.entry-settings .el-radio-group{margin-bottom:12px}.entry-settings .parameter-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px}
</style>
