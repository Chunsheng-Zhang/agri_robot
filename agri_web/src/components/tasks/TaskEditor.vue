<script setup lang="ts">
import { computed, ref, watch, onUnmounted } from 'vue'
import { ElMessage } from 'element-plus'
import { useRealtime } from '../../composables/useRealtime'
import { useWorkspace } from '../../composables/useWorkspace'
import { saveMission, previewMission, missionAction, downloadJson } from '../../api/workspace'
import type { MissionDocument, MissionInput, MissionMode, GeoPoint, Preview, Projection } from '../../api/workspace'
import { parameterFields } from './taskParameters'
import { defaultTaskPoints } from './defaultTaskPoints'
import type { ParameterField } from './taskParameters'
import FieldMap from '../dashboard/FieldMap.vue'

interface Row { longitude: string; latitude: string; dwell_time: number }
const props = defineProps<{ mission?: MissionDocument | null; kind: MissionMode; defaults: Readonly<Record<string, number | string>> }>()
const emit = defineEmits<{ saved: [value: MissionDocument] }>()
const { devices, isFresh, safetyFresh, refresh } = useRealtime()
const { context, refresh: refreshWorkspace } = useWorkspace()
const name = ref(''), parameters = ref<Record<string, number | string>>({}), rows = ref<Row[]>([])
const rings = ref<Row[][]>([]), ringIndex = ref(0), id = ref<string>(), revision = ref<number>()
const origin = ref<Projection>(), preview = ref<Preview | null>(null), error = ref(''), busy = ref(false), previewBusy = ref(false)
const lastSaved = ref(''), fileInput = ref<HTMLInputElement | null>(null)
let timer: ReturnType<typeof setTimeout> | undefined
let disposed = false
const copy = <T,>(value: T): T => JSON.parse(JSON.stringify(value))
const blank = (): Row => ({ longitude: '', latitude: '', dwell_time: Number(parameters.value.dwell_time ?? 2) })
const toRows = (points: readonly GeoPoint[]): Row[] => points.map(p => ({ longitude: String(p.longitude), latitude: String(p.latitude), dwell_time: p.dwell_time ?? 2 }))
const fingerprint = computed(() => JSON.stringify([name.value, rows.value, rings.value, parameters.value]))
const dirty = computed(() => !id.value || fingerprint.value !== lastSaved.value)
const editableRows = computed(() => props.kind === 'waypoint' ? rows.value : rings.value[ringIndex.value] || [])
const active = computed(() => context.value?.active_run)
const planReady = computed(() => !dirty.value && active.value?.mission_id === id.value
  && active.value?.mission_revision === revision.value && active.value?.state === 'READY')
const showLive = computed(() => !dirty.value && active.value?.mission_id === id.value && active.value?.mission_revision === revision.value)
const executingThis = computed(() => active.value?.mission_id === id.value && ['PLANNING', 'ACQUIRING', 'RUNNING', 'PAUSED'].includes(active.value?.state || ''))
const controlsIdle = computed(() => safetyFresh.value && !devices.value?.safety.owner &&
  !['waypoint', 'tracking', 'planning'].some(key => {
    const status = devices.value?.[key as 'waypoint' | 'tracking' | 'planning']
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
  const mission = props.mission
  name.value = mission?.name || (props.kind === 'waypoint' ? '新多点导航任务' : '新农田覆盖任务')
  parameters.value = copy(mission?.parameters || props.defaults)
  rows.value = toRows(mission ? copy(mission.waypoints) : defaultTaskPoints)
  rings.value = mission
    ? loadBoundary(copy(mission.boundary) as Record<string, unknown> | null)
    : [toRows(defaultTaskPoints)]
  ringIndex.value = 0; id.value = mission?.id; revision.value = mission?.revision
  origin.value = mission ? copy(mission.projection) : undefined
  preview.value = mission ? copy(mission.preview) : null
  error.value = ''; lastSaved.value = fingerprint.value
  clearTimeout(timer)
  if (!mission) timer = setTimeout(() => void buildPreview(true), 650)
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
    waypoints: props.kind === 'waypoint' ? parsePoints(rows.value) : [], boundary,
    ...(revision.value ? { revision: revision.value } : {}), ...(origin.value ? { import_origin: origin.value } : {}) }
}
async function buildPreview(silent = false) {
  const version = fingerprint.value
  try {
    const data = payload(); previewBusy.value = true
    const result = await previewMission(data)
    if (!disposed && version === fingerprint.value) { preview.value = result; error.value = '' }
  } catch (value) {
    if (!disposed && version === fingerprint.value) { preview.value = null; error.value = value instanceof Error ? value.message : '预览失败' }
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
async function operate(action: 'plan' | 'start') {
  if (!id.value || dirty.value || busy.value) return
  busy.value = true
  try { await missionAction(id.value, action); ElMessage.success(action === 'plan' ? '正在调用 OpenNav 规划，请等待结果' : '开始请求已确认，请查看运行反馈') }
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
      if (props.kind !== 'waypoint') throw new Error('CSV 航点请在多点导航类型中导入')
      const [header, ...lines] = text.trim().split(/\r?\n/), columns = header!.split(',').map(p => p.trim())
      const lon = columns.indexOf('longitude'), lat = columns.indexOf('latitude'), dwell = columns.indexOf('dwell_time')
      if (lon < 0 || lat < 0) throw new Error('CSV 表头需要 longitude,latitude，可选 dwell_time')
      rows.value = lines.filter(Boolean).map(line => { const values = line.split(','); return { longitude: values[lon]!.trim(), latitude: values[lat]!.trim(), dwell_time: dwell < 0 ? 2 : Number(values[dwell]) } })
    } else {
      const data = JSON.parse(text), task = data.task || data
      if (task.mode) {
        if (task.mode !== props.kind) throw new Error('请先选择与导入文件一致的任务类型')
        name.value = task.name; parameters.value = { ...copy(props.defaults), ...task.parameters }
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
onUnmounted(() => { disposed = true; clearTimeout(timer) })
</script>

<template>
  <div class="mission-editor">
    <el-card shadow="never">
      <template #header><div class="editor-heading"><strong>{{ id ? '编辑任务' : '创建任务' }}</strong><el-tag :type="dirty ? 'warning' : 'success'">{{ dirty ? '有未保存内容' : `已保存 · v${revision}` }}</el-tag></div></template>
      <el-form label-position="top" :disabled="busy || executingThis">
        <el-form-item label="任务名称"><el-input v-model="name" maxlength="80" placeholder="例如：东侧试验田第一轮" /></el-form-item>
        <div class="editor-toolbar">
          <el-button @click="editableRows.push(blank())">添加{{ kind === 'waypoint' ? '目标点' : '角点' }}</el-button>
          <el-button :disabled="!isFresh(devices?.gnss)" @click="capture">采集当前 RTK 点</el-button>
          <el-button @click="fileInput?.click()">导入文件</el-button><el-button @click="exportTask">导出 JSON</el-button>
          <input ref="fileInput" type="file" accept=".json,.geojson,.csv" hidden @change="importFile" />
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
            <el-select v-if="field.choices" v-model="parameters[field.key]"><el-option v-for="(label, value) in field.choices" :key="value" :label="label" :value="value" /></el-select>
            <el-input-number v-else :model-value="numberValue(field)" :min="field.min" :max="field.max" :step="field.step || 0.1" :precision="3" controls-position="right" @update:model-value="setNumber(field, $event)" />
            <small v-if="field.hint">{{ field.hint }}</small>
          </el-form-item>
        </div>
        <el-alert v-if="error" :title="error" type="warning" :closable="false" />
        <div class="editor-toolbar actions">
          <el-button :loading="previewBusy" @click="buildPreview()">检查边界 / 航点</el-button>
          <el-button type="primary" :loading="busy" @click="save">保存任务</el-button>
          <el-button v-if="kind === 'coverage'" type="success" plain :disabled="dirty || busy || !controlsIdle || devices?.safety.mode !== 'coverage'" @click="operate('plan')">生成覆盖路径</el-button>
          <el-button type="success" :disabled="dirty || busy || !controlsIdle || devices?.safety.mode !== kind || devices?.safety.motion_inhibited !== false || devices?.safety.localization_valid !== true || (kind === 'coverage' && !planReady)" @click="operate('start')">{{ kind === 'coverage' ? '开始路径跟踪' : '开始多点导航' }}</el-button>
        </div>
        <p class="hint">先保存，再选择对应模式。全覆盖路径由真实 OpenNav 生成，规划成功不会自动行驶。需要靠近参考路径起点后明确开始跟踪。</p>
      </el-form>
    </el-card>
    <div v-if="preview" class="preview-summary">{{ kind === 'coverage' ? `地块面积 ${(preview.area || 0).toFixed(2)} m² · 周界 ${preview.length.toFixed(2)} m` : `目标点 ${preview.points.length} 个 · 相邻点连线长 ${preview.length.toFixed(2)} m` }} · map 米制坐标</div>
    <FieldMap :preview="preview" :show-live-path="showLive" />
  </div>
</template>

<style scoped>
.mission-editor { min-width: 0; }.editor-heading { display: flex; justify-content: space-between; align-items: center; gap: 10px; }
.editor-toolbar, .ring-select { display: flex; flex-wrap: wrap; align-items: center; gap: 8px; margin: 14px 0; }.editor-toolbar .el-button, .ring-select .el-button { margin: 0; }.ring-select .el-select { width: 190px; }
.parameter-grid { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 0 18px; }.parameter-grid .el-input-number { width: 100%; }.parameter-grid small { display: block; color: #829087; line-height: 1.5; padding-top: 5px; }
.hint { color: #7c8a81; font-size: 12px; line-height: 1.8; }.actions { margin-top: 20px; }.preview-summary { margin: 18px 0 10px; padding: 14px; background: #eaf3ec; color: #376e4b; border-radius: 8px; font-size: 13px; }
@media (max-width: 1100px) { .parameter-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); } }@media (max-width: 620px) { .parameter-grid { grid-template-columns: 1fr; } }
</style>
