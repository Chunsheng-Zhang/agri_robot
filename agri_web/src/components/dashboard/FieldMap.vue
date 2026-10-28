<script setup lang="ts">
import { computed, onMounted, onUnmounted, ref } from 'vue'
import { MapLocation, Refresh } from '@element-plus/icons-vue'
import { getReferencePath, getTrackedPath } from '../../api/robot'
import type { PathPoint, ReferencePathResponse } from '../../api/robot'
import { useRealtime } from '../../composables/useRealtime'

const { robot, poseFresh } = useRealtime()
const referencePath = ref<PathPoint[]>([])
const trackedPath = ref<PathPoint[]>([])
const pathLoading = ref(false)
const pathError = ref('')
const pathUpdatedAt = ref('')
const width = 800, height = 450, padding = 35
let timer: ReturnType<typeof setInterval> | undefined
let disposed = false
function validatedPath(path: ReferencePathResponse) {
  if (path.points.length && path.frame_id !== 'map') throw new Error('路径坐标系不是 map，无法叠加')
  if (!path.points.every(point => Number.isFinite(point.x) && Number.isFinite(point.y))) throw new Error('路径含无效坐标')
  return path.points
}
async function refreshPath() {
  if (pathLoading.value) return
  pathLoading.value = true
  const results = await Promise.allSettled([getReferencePath(), getTrackedPath()])
  if (disposed) return
  const errors: string[] = []
  for (const [index, result] of results.entries()) {
    let points: PathPoint[] = []
    try {
      if (result.status === 'rejected') throw result.reason
      points = validatedPath(result.value)
    } catch (error) {
      errors.push(error instanceof Error ? error.message : '路径连接失败')
    }
    if (index === 0) referencePath.value = points
    else trackedPath.value = points
  }
  pathError.value = errors.join('；')
  pathUpdatedAt.value = errors.length ? '' : new Date().toLocaleTimeString()
  pathLoading.value = false
}
const position = computed(() => {
  if (!poseFresh.value || !robot.value || robot.value.position_x === null || robot.value.position_y === null) return null
  return { x: robot.value.position_x, y: robot.value.position_y }
})
const transform = computed(() => {
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity
  for (const group of [referencePath.value, trackedPath.value, position.value ? [position.value] : []]) {
    for (const p of group) {
      minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x)
      minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y)
    }
  }
  if (!Number.isFinite(minX)) return null
  const spanX = Math.max(maxX - minX, 1), spanY = Math.max(maxY - minY, 1)
  const scale = Math.min((width - padding * 2) / spanX, (height - padding * 2) / spanY)
  const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2
  return (p: PathPoint) => ({ x: width / 2 + (p.x - cx) * scale, y: height / 2 - (p.y - cy) * scale })
})
function polyline(points: PathPoint[]) {
  if (!transform.value) return ''
  const step = Math.max(1, Math.ceil(points.length / 4000))
  return points.filter((_, i) => i % step === 0 || i === points.length - 1)
    .map(p => { const s = transform.value!(p); return `${s.x},${s.y}` }).join(' ')
}
const referenceLine = computed(() => polyline(referencePath.value))
const trackedLine = computed(() => polyline(trackedPath.value))
const marker = computed(() => position.value && transform.value ? transform.value(position.value) : null)
const start = computed(() => referencePath.value[0] && transform.value ? transform.value(referencePath.value[0]) : null)
const end = computed(() => {
  const p = referencePath.value.at(-1)
  return p && transform.value ? transform.value(p) : null
})
onMounted(() => { void refreshPath(); timer = setInterval(() => void refreshPath(), 2000) })
onUnmounted(() => { disposed = true; clearInterval(timer) })
defineExpose({ refreshPath })
</script>

<template>
  <el-card class="field-card" shadow="never">
    <template #header><div class="heading"><span><el-icon><MapLocation /></el-icon> 实时路径与位置</span><el-tag type="info">map · 米制坐标</el-tag></div></template>
    <el-alert v-if="pathError" :title="pathError" type="warning" :closable="false" />
    <div class="path-canvas">
      <svg v-if="transform" :viewBox="`0 0 ${width} ${height}`" role="img" aria-label="参考路径、实际轨迹和机器人位置">
        <defs><pattern id="live-path-grid" width="40" height="40" patternUnits="userSpaceOnUse"><path d="M 40 0 L 0 0 0 40" fill="none" stroke="#e4eae6" /></pattern></defs>
        <rect width="100%" height="100%" fill="url(#live-path-grid)" />
        <polyline :points="referenceLine" fill="none" stroke="#25845c" stroke-width="3" stroke-linejoin="round" />
        <polyline :points="trackedLine" fill="none" stroke="#e59a36" stroke-width="2.5" stroke-linejoin="round" />
        <circle v-if="start" :cx="start.x" :cy="start.y" r="6" fill="#25845c" stroke="white" stroke-width="2" />
        <circle v-if="end" :cx="end.x" :cy="end.y" r="6" fill="#c65d5d" stroke="white" stroke-width="2" />
        <g v-if="marker" :transform="`translate(${marker.x} ${marker.y}) rotate(${-(robot?.heading ?? 0)})`">
          <circle r="12" fill="#e2efff" stroke="#3479c5" stroke-width="2" />
          <path d="M 18 0 L -6 -6 L -6 6 Z" fill="#3479c5" />
        </g>
        <text x="745" y="28" fill="#67776c" font-size="13">北 ↑</text>
      </svg>
      <div v-else class="empty">等待参考路径或有效机器人位姿</div>
    </div>
    <div class="path-footer">
      <span>参考 {{ referencePath.length }} 点 · 轨迹 {{ trackedPath.length }} 点</span>
      <el-button size="small" plain :loading="pathLoading" @click="refreshPath"><el-icon><Refresh /></el-icon>刷新</el-button>
    </div>
    <div class="legend"><span class="reference">━ 参考路径</span><span class="tracked">━ 实际轨迹</span><span class="robot">● 当前位置 / 航向</span></div>
    <p class="hint">路径查询：{{ pathUpdatedAt || '--' }}；{{ poseFresh ? '位姿实时更新' : '位姿过期，当前位置已隐藏' }}。历史轨迹保留用于对照。</p>
  </el-card>
</template>

<style scoped>
.field-card { height: 100%; border-radius: 10px; border-color: #e5eae7; }
.heading, .path-footer { display: flex; align-items: center; justify-content: space-between; gap: 12px; flex-wrap: wrap; }
.heading { font-weight: 600; color: #263b30; }
.path-canvas { display: grid; place-items: center; min-height: 300px; border: 1px solid #e5eae7; border-radius: 8px; background: #f7faf7; overflow: hidden; margin-top: 12px; }
.path-canvas svg { width: 100%; max-height: 450px; }
.empty { padding: 50px 15px; color: #85938a; text-align: center; }
.path-footer { padding-top: 16px; color: #67776c; font-size: 13px; }
.legend { display: flex; gap: 18px; flex-wrap: wrap; padding-top: 14px; font-size: 12px; }
.reference { color: #25845c; } .tracked { color: #bc751e; } .robot { color: #3479c5; }
.hint { color: #85938a; font-size: 12px; line-height: 1.6; }
</style>
