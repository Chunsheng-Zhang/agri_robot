<script setup lang="ts">
import { computed, onMounted, onUnmounted, reactive, ref, useId, watch } from 'vue'
import { MapLocation, Refresh } from '@element-plus/icons-vue'
import { getReferencePath, getTrackedPath, getWaypoints } from '../../api/robot'
import type { PathPoint, ReferencePathResponse } from '../../api/robot'
import type { CoverageMapSource, Preview, Run, EntryPlan } from '../../api/workspace'
import { useRealtime } from '../../composables/useRealtime'
import { useCoverageLayers } from '../../composables/useCoverageLayers'
import { sequenceBands } from './pathSequence'

const { robot, devices, safetyFresh, poseFresh } = useRealtime()
const props = withDefaults(defineProps<{ preview?: Preview | null; showLivePath?: boolean;
  pathSource?: 'tracking' | 'following' | 'waypoint';
  entryPoint?:PathPoint|null; entryPicking?:boolean; entryEditable?:boolean; entryPlan?:EntryPlan|null;
  coverageMode?: boolean; coverageRun?: Run | null; savedReference?: ReferencePathResponse | null }>(), { showLivePath: true, coverageMode: false, pathSource: 'tracking' })
const emit=defineEmits<{entrySelected:[point:PathPoint]}>()
const entryDrag=ref<PathPoint|null>(null)
let selectingEntry=false
let frozenProjection:((p:PathPoint)=>PathPoint)|null=null
const source = ref<CoverageMapSource>('actual')
const layers = reactive({ inset: true, completed: true, reference: true, remaining: true, overlap: true, forbidden: true })
const showRowOrder = ref(true), showRetraced = ref(true)
const coverage = useCoverageLayers(() => props.coverageMode && (props.showLivePath || props.savedReference) ? props.coverageRun : null, () => source.value)
watch(() => props.savedReference, value => { if (value) source.value = 'planned' }, { immediate: true })
const coverageData = coverage.snapshot
const coverageRaster = coverage.raster
const mapId = useId().replace(/[^a-zA-Z0-9_-]/g, '')
const clipId = `coverage-field-${mapId}`, gridId = `live-path-grid-${mapId}`, forbiddenId = `forbidden-${mapId}`
const referencePath = ref<PathPoint[]>([])
const referenceFingerprint = ref<string | null>(null)
const trackedPath = ref<PathPoint[]>([])
const liveTargets = ref<PathPoint[]>([])
const targetPoints = computed(() => props.preview?.points.length ? props.preview.points
  : safetyFresh.value && devices.value?.safety.mode === 'waypoint' ? liveTargets.value : [])
const pathLoading = ref(false)
const pathError = ref('')
const pathUpdatedAt = ref('')
const width = 800, height = 450, padding = 35
const zoom = ref(1), panX = ref(0), panY = ref(0)
const viewBox = computed(() => `${panX.value} ${panY.value} ${width / zoom.value} ${height / zoom.value}`)
let drag: { x: number; y: number; px: number; py: number } | null = null
function resetView() { zoom.value = 1; panX.value = panY.value = 0 }
function scale(factor: number) { zoom.value = Math.max(1, Math.min(8, zoom.value * factor)); if (zoom.value === 1) resetView() }
function pointerDown(event: PointerEvent) {
  const element = event.currentTarget as SVGSVGElement
  element.setPointerCapture(event.pointerId)
  if(props.entryPicking || props.entryEditable && (event.target as Element)?.closest?.('[data-entry-marker]')){
    selectingEntry=true;frozenProjection=transform.value
    entryDrag.value=mapPoint(event);return
  }
  drag = { x: event.clientX, y: event.clientY, px: panX.value, py: panY.value }
}
function pointerMove(event: PointerEvent) {
  if(selectingEntry){entryDrag.value=mapPoint(event);return}
  if (!drag) return
  const matrix = (event.currentTarget as SVGSVGElement).getScreenCTM()
  if (!matrix?.a || !matrix.d) return
  panX.value = drag.px - (event.clientX - drag.x) / matrix.a
  panY.value = drag.py - (event.clientY - drag.y) / matrix.d
}
function mapPoint(event:PointerEvent):PathPoint|null{
  const project=frozenProjection||transform.value,matrix=(event.currentTarget as SVGSVGElement).getScreenCTM()
  if(!project||!matrix)return null
  const det=matrix.a*matrix.d-matrix.b*matrix.c
  if(!det)return null
  const dx=event.clientX-matrix.e,dy=event.clientY-matrix.f
  const sx=(dx*matrix.d-dy*matrix.c)/det,sy=(dy*matrix.a-dx*matrix.b)/det
  const zero=project({x:0,y:0}),one=project({x:1,y:1})
  return {x:(sx-zero.x)/(one.x-zero.x),y:(sy-zero.y)/(one.y-zero.y)}
}
function pointerUp(event:PointerEvent){
  if(selectingEntry&&(props.entryPicking||props.entryEditable)){
    const point=mapPoint(event)
    if(point&&Number.isFinite(point.x)&&Number.isFinite(point.y))emit('entrySelected',point)
  }
  selectingEntry=false;entryDrag.value=null;frozenProjection=null;drag=null
}
function pointerCancel(){selectingEntry=false;entryDrag.value=null;frozenProjection=null;drag=null}
let timer: ReturnType<typeof setInterval> | undefined
let disposed = false
let pathGeneration = 0
function validatedPath(path: ReferencePathResponse) {
  if (path.points.length && path.frame_id !== 'map') throw new Error('路径坐标系不是 map，无法叠加')
  if (!path.points.every(point => Number.isFinite(point.x) && Number.isFinite(point.y))) throw new Error('路径含无效坐标')
  return path.points
}
async function refreshPath() {
  if (!props.showLivePath) { referencePath.value = []; trackedPath.value = []; return }
  if (pathLoading.value) return
  pathLoading.value = true
  const generation = pathGeneration
  const empty: ReferencePathResponse = { count: 0, points: [], frame_id: 'map' }
  const results = await Promise.allSettled([getReferencePath(props.pathSource),
    props.coverageMode ? Promise.resolve(empty) : getTrackedPath(props.pathSource),
    props.coverageMode || props.pathSource === 'following' ? Promise.resolve(empty) : getWaypoints()])
  if (disposed) return
  if (generation !== pathGeneration) {
    pathLoading.value = false
    if (props.showLivePath) void refreshPath()
    return
  }
  const errors: string[] = []
  for (const [index, result] of results.entries()) {
    let points: PathPoint[] = []
    try {
      if (result.status === 'rejected') throw result.reason
      points = validatedPath(result.value)
    } catch (error) {
      errors.push(error instanceof Error ? error.message : '路径连接失败')
    }
    if (index === 0) {
      // A stable fingerprint keeps compact display geometry cached during polling.
      if (!points.length || result.status !== 'fulfilled' || !result.value.fingerprint
        || result.value.fingerprint !== referenceFingerprint.value) referencePath.value = points
      referenceFingerprint.value = result.status === 'fulfilled' ? result.value.fingerprint || null : null
    }
    else if (index === 1) trackedPath.value = points
    else liveTargets.value = points
  }
  pathError.value = errors.join('；')
  pathUpdatedAt.value = errors.length ? '' : new Date().toLocaleTimeString()
  pathLoading.value = false
}
watch(() => [props.showLivePath, props.pathSource], () => {
  pathGeneration++
  referencePath.value = []; trackedPath.value = []; liveTargets.value = []; referenceFingerprint.value = null
  if (props.showLivePath) void refreshPath()
})
const position = computed(() => {
  if (!poseFresh.value || !robot.value || robot.value.position_x === null || robot.value.position_y === null) return null
  return { x: robot.value.position_x, y: robot.value.position_y }
})
const displayReference = computed(() => {
  if (props.savedReference) return validatedPath(props.savedReference)
  if (!props.showLivePath) return []
  if (!props.coverageMode) return referencePath.value
  const expected = props.coverageRun?.reference_fingerprint
  return expected && expected === referenceFingerprint.value ? referencePath.value : []
})
const displayTracks = computed(() => !props.showLivePath && !props.savedReference ? [] : props.coverageMode
  ? coverageData.value?.sources?.actual.track_lines || [] : [trackedPath.value])
const rings = computed(() => props.preview?.rings || coverageData.value?.field_rings || [])
const inset = computed(() => (props.showLivePath || props.savedReference) && coverageData.value?.inset_polygons
  ? coverageData.value.inset_polygons : props.preview?.inset_polygons || [])
let previousBounds: number[] = []
let previousTransform: ((p: PathPoint) => PathPoint) | null = null
const transform = computed(() => {
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity
  for (const group of [displayReference.value, displayTracks.value.flat(),
    targetPoints.value, rings.value.flat(), position.value ? [position.value] : [],props.entryPoint?[props.entryPoint]:[]]) {
    for (const p of group) {
      minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x)
      minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y)
    }
  }
  if (!Number.isFinite(minX)) return null
  const bounds = [minX, maxX, minY, maxY]
  if (previousTransform && bounds.every((v, i) => v === previousBounds[i])) return previousTransform
  const spanX = Math.max(maxX - minX, 1), spanY = Math.max(maxY - minY, 1)
  const scale = Math.min((width - padding * 2) / spanX, (height - padding * 2) / spanY)
  const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2
  previousBounds = bounds
  previousTransform = (p: PathPoint) => ({ x: width / 2 + (p.x - cx) * scale, y: height / 2 - (p.y - cy) * scale })
  return previousTransform
})
function polyline(points: readonly PathPoint[]) {
  if (!transform.value) return ''
  const step = Math.max(1, Math.ceil(points.length / 4000))
  return points.filter((_, i) => i % step === 0 || i === points.length - 1)
    .map(p => { const s = transform.value!(p); return `${s.x},${s.y}` }).join(' ')
}
function polygonPath(polygon: readonly (readonly PathPoint[])[]) {
  if (!transform.value) return ''
  return polygon.map(ring => ring.map((point, index) => {
    const p = transform.value!(point)
    return `${index ? 'L' : 'M'}${p.x},${p.y}`
  }).join(' ') + ' Z').join(' ')
}
const referenceLine = computed(() => polyline(displayReference.value))
const sequence = computed(() => props.coverageMode ? sequenceBands(displayReference.value) : [])
const referenceBands = computed(() => sequence.value.map(band => ({ color: band.color, points: polyline(band.points) })))
const pathOrder = computed(() => {
  if (!props.coverageMode || !displayReference.value.length) return null
  if (props.savedReference?.fingerprint) return props.savedReference.path_order || null
  const snapshot = coverageData.value
  return snapshot?.reference_fingerprint && snapshot.reference_fingerprint === referenceFingerprint.value
    && snapshot.reference_fingerprint === props.coverageRun?.reference_fingerprint ? snapshot.path_order || null : null
})
const workRows = computed(() => {
  const value = pathOrder.value
  return value?.state === 'READY' && Array.isArray(value.rows) && value.rows.length <= 1500 && value.rows.every((row, i) =>
    row.visit === i + 1 && [row.start.x, row.start.y, row.end.x, row.end.y].every(Number.isFinite)) ? value.rows : []
})
const rowLabels = computed(() => {
  if (!transform.value) return []
  const step = Math.max(1, Math.ceil(workRows.value.length / (50 * zoom.value)))
  return workRows.value.filter((_, i) => i % step === 0 || i === workRows.value.length - 1).map(row => ({
    visit: row.visit, ...transform.value!({ x: (row.start.x + row.end.x) / 2, y: (row.start.y + row.end.y) / 2 }),
  }))
})
const retracedLines = computed(() => pathOrder.value?.retracing_state === 'READY'
  ? (pathOrder.value.retraced_lines || []).filter(line => line.every(p => Number.isFinite(p.x) && Number.isFinite(p.y))).map(polyline) : [])
const entryMarker=computed(()=>{
  const point=entryDrag.value||props.entryPoint
  return point&&transform.value?transform.value(point):null
})
const entryConnection=computed(()=>props.entryPlan?polyline(props.entryPlan.connection):'')
const entryAccess=computed(()=>props.entryPlan?polyline([props.entryPlan.reference,props.entryPlan.anchor]):'')
const entryAnchorMarker=computed(()=>props.entryPlan&&!props.entryPlan.access_verified&&transform.value
  ? transform.value(props.entryPlan.anchor):null)
const trackedLines = computed(() => displayTracks.value.map(polyline))
const fieldPath = computed(() => polygonPath(rings.value))
const forbiddenPaths = computed(() => rings.value.slice(1).map(ring => polygonPath([ring])))
const insetPaths = computed(() => inset.value.map(polygonPath))
const rasterRect = computed(() => {
  const bounds = coverageRaster.value?.extent
  if (!bounds || !transform.value) return null
  const top = transform.value({ x: bounds[0]!, y: bounds[3]! })
  const bottom = transform.value({ x: bounds[2]!, y: bounds[1]! })
  return { x: top.x, y: top.y, width: bottom.x - top.x, height: bottom.y - top.y }
})
const operationStatus = computed(() => {
  if (source.value === 'planned') return '规划预测，非已完成作业'
  const labels: Record<string, string> = { NOT_STARTED: '尚未开始正式作业', WAITING_TRACKING: '尚无可连接的正式跟踪轨迹',
    OBSERVED: '正式跟踪轨迹的实时累计估计', FINAL: '已与本轮结束报告核对', NO_DATA: '本轮缺少有效正式跟踪数据' }
  return labels[coverageData.value?.sources?.actual.status || ''] || '等待本轮作业区域'
})
const targets = computed(() => targetPoints.value.map(point => transform.value?.(point)).filter(p => !!p))
const marker = computed(() => position.value && transform.value ? transform.value(position.value) : null)
const start = computed(() => displayReference.value[0] && transform.value ? transform.value(displayReference.value[0]) : null)
const end = computed(() => {
  const p = displayReference.value.at(-1)
  return p && transform.value ? transform.value(p) : null
})
onMounted(() => { void refreshPath(); timer = setInterval(() => void refreshPath(), 2000) })
onUnmounted(() => { disposed = true; clearInterval(timer) })
defineExpose({ refreshPath })
</script>

<template>
  <el-card class="field-card" shadow="never">
    <template #header><div class="heading"><span><el-icon><MapLocation /></el-icon> {{ coverageMode ? '覆盖作业地图' : '实时路径与位置' }}</span><el-tag type="info">map · 米制坐标</el-tag></div></template>
    <el-alert v-if="pathError" :title="pathError" type="warning" :closable="false" />
    <el-alert v-if="entryPicking" title="在地图点击作业入口；紫色入口可拖动。选点只修改草稿，请保存任务并重新规划。" type="info" :closable="false"/>
    <div v-if="entryPlan" class="hint">入口参考距离：{{ entryPlan.distance_m.toFixed(2) }} m · 已验证田内连接 {{ entryPlan.connection_length_m.toFixed(2) }} m。{{ entryPlan.access_verified ? '用于起点比较的田内参考连接，不是车辆执行路线。' : '入口通道仅为规划示意；实际单点接近目标仍是覆盖起点。' }}</div>
    <div v-if="coverageMode" class="coverage-toolbar">
      <el-radio-group v-model="source" size="small" aria-label="作业区域数据来源">
        <el-radio-button value="actual">实际作业估计</el-radio-button>
        <el-radio-button value="planned">规划预测</el-radio-button>
      </el-radio-group>
      <span>{{ operationStatus }}</span>
    </div>
    <div v-if="coverageMode" class="layer-switches" aria-label="覆盖地图图层">
      <el-checkbox v-model="layers.inset"><i class="swatch inset" />内缩轮廓</el-checkbox>
      <el-checkbox v-model="layers.completed"><i class="swatch completed" />{{ source === 'actual' ? '已完成作业区域' : '预测作业区域' }}</el-checkbox>
      <el-checkbox v-model="layers.reference"><i class="swatch reference" />规划作业路径</el-checkbox>
      <el-checkbox v-model="layers.remaining"><i class="swatch remaining" />{{ source === 'actual' ? '未完成区域' : '预测漏覆盖区域' }}</el-checkbox>
      <el-checkbox v-model="layers.overlap"><i class="swatch overlap" />{{ source === 'actual' ? '重复覆盖区域' : '预测重复覆盖区域' }}</el-checkbox>
      <el-checkbox v-model="layers.forbidden"><i class="swatch forbidden" />禁入区域</el-checkbox>
      <el-checkbox v-model="showRowOrder" :disabled="!workRows.length">作业行访问序号</el-checkbox>
      <el-checkbox v-model="showRetraced"><i class="swatch retraced" />重走路径</el-checkbox>
    </div>
    <el-alert v-if="coverageMode && coverage.error.value" :title="coverage.error.value" type="warning" :closable="false" />
    <div class="path-canvas">
      <svg v-if="transform" :viewBox="viewBox" role="img" :aria-label="coverageMode ? '内缩轮廓、规划路径、已完成、未完成、重复覆盖及禁入区域' : '参考路径、实际轨迹和机器人位置'"
        @wheel.prevent="scale($event.deltaY < 0 ? 1.15 : 1 / 1.15)" @pointerdown="pointerDown" @pointermove="pointerMove" @pointerup="pointerUp" @pointercancel="pointerCancel">
        <defs>
          <pattern :id="gridId" width="40" height="40" patternUnits="userSpaceOnUse"><path d="M 40 0 L 0 0 0 40" fill="none" stroke="#e4eae6" /></pattern>
          <pattern :id="forbiddenId" width="8" height="8" patternUnits="userSpaceOnUse"><rect width="8" height="8" fill="#d5dbe0" /><path d="M0 8 L8 0" stroke="#84919b" stroke-width="1" /></pattern>
          <clipPath :id="clipId"><path :d="fieldPath" fill-rule="evenodd" clip-rule="evenodd" /></clipPath>
        </defs>
        <rect width="100%" height="100%" :fill="`url(#${gridId})`" />
        <path :d="fieldPath" fill="#e8efe9" fill-rule="evenodd" stroke="#799b7c" stroke-width="2" />
        <g v-if="coverageMode && coverageRaster && rasterRect" :key="coverageRaster.source + coverageRaster.version" :clip-path="`url(#${clipId})`">
          <image v-if="layers.remaining" v-bind="rasterRect" :href="coverageRaster.images.remaining" preserveAspectRatio="none" data-layer="remaining" />
          <image v-if="layers.completed" v-bind="rasterRect" :href="coverageRaster.images.completed" preserveAspectRatio="none" data-layer="completed" />
          <image v-if="layers.overlap" v-bind="rasterRect" :href="coverageRaster.images.overlap" preserveAspectRatio="none" data-layer="overlap" />
        </g>
        <template v-if="!coverageMode || layers.forbidden"><path v-for="(shape, index) in forbiddenPaths" :key="`hole-${index}`" :d="shape" :fill="`url(#${forbiddenId})`" stroke="#6e7c88" stroke-width="1.5" data-layer="forbidden" /></template>
        <template v-if="coverageMode && layers.inset"><path v-for="(shape, index) in insetPaths" :key="`inset-${index}`" :d="shape" fill="none" fill-rule="evenodd" stroke="#8b5cc4" stroke-dasharray="7 4" stroke-width="2" data-layer="inset" /></template>
        <g v-if="coverageMode && layers.reference" data-layer="reference-sequence">
          <polyline v-for="(band, index) in referenceBands" :key="`sequence-${index}`" :points="band.points"
            fill="none" :stroke="band.color" stroke-width="2.7" stroke-linejoin="round" stroke-linecap="round" data-layer="reference" />
        </g>
        <polyline v-else :points="!coverageMode ? referenceLine : ''" fill="none" stroke="#25845c" stroke-width="3" stroke-linejoin="round" data-layer="reference" />
        <polyline v-for="(line, index) in trackedLines" :key="`track-${index}`" :points="line" fill="none" :stroke="coverageMode ? '#445665' : '#e59a36'" stroke-width="2" stroke-linejoin="round" />
        <g v-if="coverageMode && layers.reference && showRetraced" data-layer="retraced">
          <polyline v-for="(line, index) in retracedLines" :key="`retraced-${index}`" :points="line" fill="none" stroke="#d52d35" stroke-width="4" stroke-linejoin="round"><title>重复经过的中心线路段</title></polyline>
        </g>
        <g v-if="coverageMode && layers.reference && showRowOrder" data-layer="row-order" pointer-events="none">
          <text v-for="label in rowLabels" :key="label.visit" :x="label.x" :y="label.y" text-anchor="middle" dominant-baseline="central"
            class="row-visit-label" :font-size="13 / zoom" :stroke-width="3 / zoom"><title>第{{ label.visit }}条访问的作业行</title>{{ label.visit }}</text>
        </g>
        <g v-if="start && (!coverageMode || layers.reference)">
          <circle :cx="start.x" :cy="start.y" r="6" fill="#25845c" stroke="white" stroke-width="2"><title>路径起点（绿色）</title></circle>
          <text :x="start.x + (start.x > width / 2 ? -10 : 10)" :y="start.y - 10" :text-anchor="start.x > width / 2 ? 'end' : 'start'" class="endpoint-label" fill="#25845c">起点</text>
        </g>
        <polyline v-if="entryConnection" :points="entryConnection" fill="none" stroke="#a371bf" stroke-width="2" stroke-dasharray="7 4"><title>用于起点选择的田内参考连接；不是实际单点接近路线</title></polyline>
        <polyline v-if="entryPlan&&!entryPlan.access_verified" :points="entryAccess" fill="none" stroke="#888" stroke-width="2" stroke-dasharray="2 6"><title>未验证入口通道，仅为距离示意</title></polyline>
        <g v-if="entryAnchorMarker"><circle :cx="entryAnchorMarker.x" :cy="entryAnchorMarker.y" r="4" fill="white" stroke="#873cb2"/><text :x="entryAnchorMarker.x+8" :y="entryAnchorMarker.y+16" class="endpoint-label" fill="#873cb2">内缩安全点</text></g>
        <g v-if="entryMarker" data-entry-marker :transform="`translate(${entryMarker.x} ${entryMarker.y})`" :style="{cursor:entryEditable?'move':'default'}">
          <circle r="12" fill="transparent"/><path d="M 0 9 L 0 -13 L 13 -13 L 9 -7 L 0 -7" stroke="#873cb2" stroke-width="2" fill="#d9b6ef"/>
          <text x="16" y="-9" class="endpoint-label" fill="#873cb2">入口</text>
        </g>
        <g v-if="end && (!coverageMode || layers.reference)">
          <circle :cx="end.x" :cy="end.y" r="6" fill="#c65d5d" stroke="white" stroke-width="2"><title>路径终点（红色）</title></circle>
          <text :x="end.x + (end.x > width / 2 ? -10 : 10)" :y="end.y + 20" :text-anchor="end.x > width / 2 ? 'end' : 'start'" class="endpoint-label" fill="#c65d5d">终点</text>
        </g>
        <g v-if="marker" :transform="`translate(${marker.x} ${marker.y}) rotate(${-(robot?.heading ?? 0)})`">
          <circle r="12" fill="#e2efff" stroke="#3479c5" stroke-width="2" />
          <path d="M 18 0 L -6 -6 L -6 6 Z" fill="#3479c5" />
        </g>
        <text x="745" y="28" fill="#67776c" font-size="13">北 ↑</text>
        <circle v-for="(point, index) in targets" :key="`target-${index}`" :cx="point.x" :cy="point.y" r="6" fill="#8b63bd" />
        <text v-for="(point, index) in targets" :key="`label-${index}`" :x="point.x + 9" :y="point.y - 7" font-size="13" fill="#684196">{{ index + 1 }}</text>
      </svg>
      <div v-else class="empty">等待参考路径或有效机器人位姿</div>
    </div>
    <div class="path-footer">
      <span>参考 {{ displayReference.length }} 点 · 轨迹 {{ displayTracks.flat().length }} 点</span>
      <el-button size="small" plain :loading="pathLoading || coverage.loading.value" @click="refreshPath(); coverage.refresh()"><el-icon><Refresh /></el-icon>刷新</el-button>
      <div><el-button size="small" @click="scale(1.4)">放大</el-button><el-button size="small" @click="scale(1 / 1.4)">缩小</el-button><el-button size="small" @click="resetView">复位视图</el-button></div>
    </div>
    <div class="legend"><span v-if="coverageMode" class="sequence-legend">行驶顺序：先 <i class="sequence-ramp" /> 后</span><span v-else class="reference">━ 参考路径</span><span v-if="coverageMode" class="retraced-legend">━ 重走路径（红色）</span><span class="path-start">● 起点（绿色）</span><span class="path-end">● 终点（红色）</span><span class="tracked">━ {{ savedReference ? '本记录实际轨迹' : '最近实际轨迹' }}</span><span class="robot">● 当前位置 / 航向</span></div>
    <p v-if="coverageMode && displayReference.length" class="hint sequence-hint">路径颜色按行驶先后由紫到黄；红线为同一中心线路段重走，{{ pathOrder?.retracing_state === 'READY' ? '已完成识别。' : pathOrder?.retracing_reason || '重走信息准备中。' }}{{ workRows.length ? `作业行按真实访问顺序从1编号，共${workRows.length}行。` : pathOrder?.reason || '作业行访问信息准备中。' }}{{ rowLabels.length < workRows.length ? '当前视图抽稀显示序号，放大可查看更多。' : '' }}序号不包含换行、绕障连接。</p>
    <p v-if="coverageMode" class="hint">内缩距离 {{ coverageData?.inset_distance_m ?? preview?.inset_distance_m ?? '--' }} m；{{ coverage.imagesLoading.value ? '图层读取中；' : '' }}{{ coverage.stale.value ? '区域快照未及时更新；' : '' }}{{ coverageData?.state === 'PENDING' ? '作业区域准备中；' : '' }}实际区域按本轮正式跟踪轨迹和评价幅宽累计，假设机具开启；接近、对齐和空档不涂为已完成。尚无轨迹时红色表示尚未记录作业。约2秒刷新，结束报告为最终核对口径。</p>
    <p class="hint">路径查询：{{ pathUpdatedAt || '--' }}；{{ poseFresh ? '位姿实时更新' : '位姿过期，当前位置已隐藏' }}。历史轨迹保留用于对照。</p>
  </el-card>
</template>

<style scoped>
.field-card { min-width: 0; border-radius: 10px; border-color: #e5eae7; }
.heading, .path-footer { display: flex; align-items: center; justify-content: space-between; gap: 12px; flex-wrap: wrap; }
.heading { font-weight: 600; color: #263b30; }
.path-canvas { display: grid; place-items: center; height: clamp(240px, 38vh, 360px); min-height: 0; border: 1px solid #e5eae7; border-radius: 8px; background: #f7faf7; overflow: hidden; margin-top: 12px; }
.path-canvas svg { display: block; width: 100%; height: 100%; min-height: 0; }
.path-canvas svg { touch-action: none; cursor: grab; }
.empty { padding: 50px 15px; color: #85938a; text-align: center; }
.path-footer { padding-top: 16px; color: #67776c; font-size: 13px; }
.legend { display: flex; gap: 18px; flex-wrap: wrap; padding-top: 14px; font-size: 12px; }
.reference { color: #25845c; } .tracked { color: #bc751e; } .robot { color: #3479c5; }
.path-start { color: #25845c; } .path-end { color: #c65d5d; }
.endpoint-label { font-size: 13px; font-weight: 600; paint-order: stroke; stroke: white; stroke-width: 3px; stroke-linejoin: round; pointer-events: none; }
.hint { color: #85938a; font-size: 12px; line-height: 1.6; }
.coverage-toolbar { display: flex; align-items: center; flex-wrap: wrap; gap: 12px; margin-top: 12px; font-size: 12px; color: #65786b; }
.layer-switches { display: flex; flex-wrap: wrap; gap: 0 14px; margin: 10px 0; }
.layer-switches :deep(.el-checkbox) { margin-right: 0; }
.swatch { display: inline-block; width: 15px; height: 10px; margin-right: 5px; border-radius: 2px; }
.swatch.inset { height: 0; border-top: 2px dashed #8b5cc4; }
.swatch.completed { background: #48a368; }.swatch.reference { height: 3px; background: linear-gradient(90deg,#440154,#21918c,#fde725); }
.swatch.remaining { background: #dc6262; }.swatch.overlap { background: #ef9d30; }
.swatch.forbidden { background: repeating-linear-gradient(135deg, #d5dbe0 0 3px, #84919b 3px 4px); }
.planned { color: #1c71d8; }
.row-visit-label { fill: #233d30; font-weight: 650; paint-order: stroke; stroke: white; stroke-linejoin: round; }
.sequence-legend { display: inline-flex; align-items: center; gap: 6px; color: #4d5d55; }
.sequence-ramp { display: inline-block; width: 100px; height: 7px; border-radius: 3px; background: linear-gradient(90deg,#440154,#443983,#31688e,#21918c,#35b779,#90d743,#fde725); }
.retraced-legend { color: #d52d35; }.swatch.retraced { height: 0; border-top: 3px solid #d52d35; }
</style>
