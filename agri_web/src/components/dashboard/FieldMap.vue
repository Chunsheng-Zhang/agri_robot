<script setup lang="ts">
import {
  ref,
  computed,
  onMounted,
  onUnmounted,
} from 'vue'
import { Refresh, MapLocation } from '@element-plus/icons-vue'
import { getReferencePath } from '../../api/robot'
import type { PathPoint } from '../../api/robot'

interface SvgPoint {
  x: number
  y: number
}

const referencePath = ref<PathPoint[]>([])
const pathLoading = ref(false)
const pathError = ref('')
const pathUpdatedAt = ref('')

const SVG_WIDTH = 800
const SVG_HEIGHT = 450
const SVG_PADDING = 30

let refreshTimer: ReturnType<typeof setInterval> | undefined

async function refreshPath() {
  if (pathLoading.value) return

  pathLoading.value = true
  pathError.value = ''

  try {
    const data = await getReferencePath()

    if (!Array.isArray(data.points)) {
      throw new Error('后端返回的数据中缺少 points 数组')
    }

    referencePath.value = data.points.filter(
      point =>
        Number.isFinite(point.x) &&
        Number.isFinite(point.y),
    )

    pathUpdatedAt.value = new Date().toLocaleTimeString()
  } catch (error) {
    console.error('获取参考路径失败：', error)

    pathError.value =
      error instanceof Error
        ? error.message
        : '参考路径加载失败'
  } finally {
    pathLoading.value = false
  }
}

const svgPathPoints = computed<SvgPoint[]>(() => {
  const points = referencePath.value

  if (points.length === 0) return []

  const xs = points.map(point => point.x)
  const ys = points.map(point => point.y)

  const minX = Math.min(...xs)
  const maxX = Math.max(...xs)
  const minY = Math.min(...ys)
  const maxY = Math.max(...ys)

  const worldWidth = maxX - minX || 1
  const worldHeight = maxY - minY || 1

  const availableWidth = SVG_WIDTH - SVG_PADDING * 2
  const availableHeight = SVG_HEIGHT - SVG_PADDING * 2

  const scale = Math.min(
    availableWidth / worldWidth,
    availableHeight / worldHeight,
  )

  const renderedWidth = worldWidth * scale
  const renderedHeight = worldHeight * scale

  const offsetX = (SVG_WIDTH - renderedWidth) / 2
  const offsetY = (SVG_HEIGHT - renderedHeight) / 2

  return points.map(point => ({
    x: offsetX + (point.x - minX) * scale,
    y: SVG_HEIGHT - offsetY - (point.y - minY) * scale,
  }))
})

const pathPolyline = computed(() =>
  svgPathPoints.value
    .map(point => `${point.x},${point.y}`)
    .join(' '),
)

const pathStart = computed<SvgPoint | null>(
  () => svgPathPoints.value[0] ?? null,
)

const pathEnd = computed<SvgPoint | null>(() => {
  const points = svgPathPoints.value
  return points.at(-1) ?? null
})

onMounted(() => {
  refreshPath()
  refreshTimer = setInterval(refreshPath, 5000)
})

onUnmounted(() => {
  if (refreshTimer !== undefined) {
    clearInterval(refreshTimer)
  }
})

defineExpose({
  refreshPath,
})
</script>

<template>
  <el-card class="panel-card field-card" shadow="never">
    <template #header>
      <div class="panel-header">
        <div class="panel-title">
          <el-icon><MapLocation /></el-icon>
          <span>农业参考路径</span>
        </div>

        <el-tag
          v-if="pathError"
          type="danger"
          size="small"
        >
          加载失败
        </el-tag>

        <el-tag
          v-else-if="pathLoading"
          type="info"
          size="small"
        >
          加载中
        </el-tag>

        <el-tag
          v-else-if="referencePath.length > 0"
          type="success"
          size="small"
        >
          已加载
        </el-tag>

        <el-tag v-else type="info" size="small">
          暂无路径
        </el-tag>
      </div>
    </template>

    <div class="path-canvas">
      <div
        v-if="pathLoading && referencePath.length === 0"
        class="path-empty"
      >
        <el-icon class="empty-icon"><Refresh /></el-icon>
        <span>正在加载参考路径...</span>
      </div>

      <div
        v-else-if="pathError && referencePath.length === 0"
        class="path-empty"
      >
        <el-icon class="empty-icon"><MapLocation /></el-icon>
        <span>{{ pathError }}</span>

        <el-button
          type="success"
          plain
          size="small"
          @click="refreshPath"
        >
          重新加载
        </el-button>
      </div>

      <div
        v-else-if="referencePath.length === 0"
        class="path-empty"
      >
        <el-icon class="empty-icon"><MapLocation /></el-icon>
        <span>暂无参考路径数据</span>
        <small>请先启动路径发布节点</small>
      </div>

      <svg
        v-else
        class="reference-path-svg"
        :viewBox="`0 0 ${SVG_WIDTH} ${SVG_HEIGHT}`"
        preserveAspectRatio="xMidYMid meet"
        role="img"
        aria-label="农业机器人参考路径"
      >
        <defs>
          <pattern
            id="path-grid"
            width="40"
            height="40"
            patternUnits="userSpaceOnUse"
          >
            <path
              d="M 40 0 L 0 0 0 40"
              fill="none"
              stroke="#e8eee9"
              stroke-width="1"
            />
          </pattern>
        </defs>

        <rect
          width="100%"
          height="100%"
          fill="url(#path-grid)"
        />

        <polyline
          :points="pathPolyline"
          fill="none"
          stroke="#25845c"
          stroke-width="3"
          stroke-linecap="round"
          stroke-linejoin="round"
        />

        <circle
          v-if="pathStart"
          :cx="pathStart.x"
          :cy="pathStart.y"
          r="7"
          fill="#36a66b"
          stroke="#ffffff"
          stroke-width="2"
        />

        <circle
          v-if="pathEnd"
          :cx="pathEnd.x"
          :cy="pathEnd.y"
          r="7"
          fill="#e45b5b"
          stroke="#ffffff"
          stroke-width="2"
        />
      </svg>
    </div>

    <div class="path-footer">
      <div class="path-info">
        <span>路径点数</span>
        <strong>{{ referencePath.length }}</strong>
      </div>

      <div class="path-info">
        <span>更新时间</span>
        <strong>{{ pathUpdatedAt || '--' }}</strong>
      </div>

      <el-button
        type="success"
        plain
        size="small"
        :loading="pathLoading"
        @click="refreshPath"
      >
        <el-icon><Refresh /></el-icon>
        刷新路径
      </el-button>
    </div>

    <div class="path-legend">
      <span>
        <i class="legend-dot start-dot" />
        起点
      </span>

      <span>
        <i class="legend-dot end-dot" />
        终点
      </span>

      <span>
        <i class="legend-line" />
        参考路径
      </span>
    </div>
  </el-card>
</template>

<style scoped>
.panel-card {
  height: 100%;
  border-radius: 10px;
  border-color: #e5eae7;
}

.panel-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 10px;
  color: #263b30;
  font-size: 15px;
  font-weight: 600;
}

.panel-title {
  display: flex;
  align-items: center;
  gap: 8px;
}

.panel-title .el-icon {
  color: #25845c;
}

.path-canvas {
  display: flex;
  min-height: 300px;
  align-items: center;
  justify-content: center;
  overflow: hidden;
  border: 1px solid #e5eae7;
  border-radius: 8px;
  background: #f7faf7;
}

.reference-path-svg {
  display: block;
  width: 100%;
  max-height: 450px;
}

.path-empty {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 12px;
  padding: 30px;
  color: #89958d;
  font-size: 13px;
  text-align: center;
  overflow-wrap: anywhere;
}

.empty-icon {
  color: #9bb6a5;
  font-size: 34px;
}

.path-empty small {
  color: #a1aaa4;
  font-size: 11px;
}

.path-footer {
  display: flex;
  align-items: center;
  flex-wrap: wrap;
  gap: 16px;
  margin-top: 18px;
}

.path-info {
  display: flex;
  flex-direction: column;
  gap: 5px;
  margin-right: 12px;
}

.path-info span {
  color: #89958d;
  font-size: 11px;
}

.path-info strong {
  color: #43564a;
  font-size: 13px;
}

.path-footer .el-button {
  margin-left: auto;
}

.path-legend {
  display: flex;
  flex-wrap: wrap;
  gap: 20px;
  margin-top: 20px;
  padding-top: 15px;
  border-top: 1px solid #edf1ee;
  color: #7b8980;
  font-size: 11px;
}

.path-legend span {
  display: inline-flex;
  align-items: center;
  gap: 7px;
}

.legend-dot {
  display: inline-block;
  width: 9px;
  height: 9px;
  border-radius: 50%;
}

.start-dot {
  background: #36a66b;
}

.end-dot {
  background: #e45b5b;
}

.legend-line {
  display: inline-block;
  width: 18px;
  height: 3px;
  border-radius: 2px;
  background: #25845c;
}

@media (max-width: 480px) {
  .path-canvas {
    min-height: 220px;
  }

  .path-footer {
    gap: 12px;
  }
}
</style>
