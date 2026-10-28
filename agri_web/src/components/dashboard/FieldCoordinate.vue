<template>
  <el-card class="coordinate-card" shadow="never">
    <template #header>
      <div class="card-header">
        <div>
          <span class="card-title">地块角点坐标</span>
          <div class="card-description">
            按照地块边界顺序输入经纬度，至少需要 3 个角点
          </div>
        </div>

        <el-tag type="info">
          {{ points.length }} 个角点
        </el-tag>
      </div>
    </template>

    <!-- 表头 -->
    <div class="coordinate-header">
      <span class="index-column">编号</span>
      <span>经度（Longitude）</span>
      <span>纬度（Latitude）</span>
      <span class="action-column">操作</span>
    </div>

    <!-- 动态角点列表 -->
    <div
      v-for="(point, index) in points"
      :key="point.id"
      class="coordinate-row"
    >
      <span class="point-index">
        {{ index + 1 }}
      </span>

      <el-input
        v-model="point.longitude"
        placeholder="请输入经度"
        clearable
      />

      <el-input
        v-model="point.latitude"
        placeholder="请输入纬度"
        clearable
      />

      <el-button
        type="danger"
        plain
        :disabled="points.length <= 3"
        @click="removePoint(index)"
      >
        删除
      </el-button>
    </div>

    <!-- 操作按钮 -->
    <div class="coordinate-actions">
      <el-button
        type="primary"
        plain
        @click="addPoint"
      >
        + 添加角点
      </el-button>

      <el-button @click="clearPoints">
        清空坐标
      </el-button>

      <el-button
        type="primary"
        :loading="submitting"
        @click="submitCoordinates"
      >
        提交地块坐标
      </el-button>
    </div>

    <!-- 提交结果 -->
    <el-alert
      v-if="resultMessage"
      :title="resultMessage"
      :type="resultType"
      :closable="true"
      show-icon
      class="result-alert"
      @close="resultMessage = ''"
    />
  </el-card>
</template>

<script setup lang="ts">
import { ref } from 'vue'
import { ElMessage } from 'element-plus'
import { apiUrl } from '../../api/http'

interface FieldPoint {
  id: number
  longitude: string
  latitude: string
}

interface Coordinate {
  longitude: number
  latitude: number
}

interface CoordinateResponse {
  success?: boolean
  message?: string
  detail?: string | Array<{
    msg?: string
  }>
  data?: {
    point_count?: number
    points?: Coordinate[]
  }
}

// ============================================================
// 角点数据
// ============================================================

let nextId = 1

function createPoint(
  longitude = '',
  latitude = '',
): FieldPoint {
  return {
    id: nextId++,
    longitude,
    latitude,
  }
}

// 使用现场坐标；不预填示例地块。
const points = ref<FieldPoint[]>([
  createPoint(), createPoint(), createPoint(), createPoint(),
])

const submitting = ref(false)

const resultMessage = ref('')
const resultType = ref<'success' | 'error'>('success')

// ============================================================
// 角点操作
// ============================================================

// 添加角点
function addPoint() {
  points.value.push(createPoint())
}

// 删除指定角点
function removePoint(index: number) {
  if (points.value.length <= 3) {
    ElMessage.warning('地块至少需要保留 3 个角点')
    return
  }

  points.value.splice(index, 1)
}

// 清空坐标，保留三行空白输入框
function clearPoints() {
  points.value = [
    createPoint(),
    createPoint(),
    createPoint(),
  ]

  resultMessage.value = ''
}

// ============================================================
// 坐标校验
// ============================================================

function collectCoordinates(): Coordinate[] {
  if (points.value.length < 3) {
    throw new Error('地块至少需要 3 个角点')
  }

  const coordinates: Coordinate[] = []

  for (let i = 0; i < points.value.length; i++) {
    const point = points.value[i]

    if (!point) {
      throw new Error(`第 ${i + 1} 个角点不存在`)
    }

    const lonText = point.longitude.trim()
    const latText = point.latitude.trim()

    if (!lonText || !latText) {
      throw new Error(
        `第 ${i + 1} 个角点的经纬度不能为空`,
      )
    }

    const longitude = Number(lonText)
    const latitude = Number(latText)

    if (
      !Number.isFinite(longitude) ||
      !Number.isFinite(latitude)
    ) {
      throw new Error(
        `第 ${i + 1} 个角点的经纬度必须是数字`,
      )
    }

    if (longitude < -180 || longitude > 180) {
      throw new Error(
        `第 ${i + 1} 个角点的经度超出范围`,
      )
    }

    if (latitude < -90 || latitude > 90) {
      throw new Error(
        `第 ${i + 1} 个角点的纬度超出范围`,
      )
    }

    coordinates.push({
      longitude,
      latitude,
    })
  }

  return coordinates
}

// ============================================================
// 提交地块坐标
// ============================================================

async function submitCoordinates() {
  if (submitting.value) {
    return
  }

  try {
    // 1. 校验坐标
    const coordinates = collectCoordinates()

    submitting.value = true
    resultMessage.value = ''

    // 2. 发送请求
    const response = await fetch(apiUrl('/api/field/coordinate'), {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Accept': 'application/json',
      },
      body: JSON.stringify({
        points: coordinates,
      }),
    })

    // 3. 只读取一次响应正文
    const responseText = await response.text()

    console.log('[地块坐标] 请求地址：', response.url)
    console.log('[地块坐标] HTTP 状态码：', response.status)
    console.log('[地块坐标] 响应类型：', response.headers.get('content-type'))
    console.log('[地块坐标] 响应内容：', responseText)

    // 4. 检查 HTTP 状态
    if (!response.ok) {
      let errorMessage = `请求失败，HTTP 状态码：${response.status}`

      // 尝试从错误响应中读取后端错误信息
      if (responseText.trim()) {
        try {
          const errorData = JSON.parse(responseText) as CoordinateResponse

          if (typeof errorData.detail === 'string') {
            errorMessage = errorData.detail
          } else if (Array.isArray(errorData.detail)) {
            errorMessage = errorData.detail
              .map(item => item.msg || '参数校验失败')
              .join('；')
          } else if (errorData.message) {
            errorMessage = errorData.message
          }
        } catch {
          errorMessage += `；${responseText.slice(0, 200)}`
        }
      }

      throw new Error(errorMessage)
    }

    // 5. 检查成功响应是否为空
    if (!responseText.trim()) {
      throw new Error(
        '服务器返回了空响应，但 HTTP 状态正常。请检查后端响应内容。',
      )
    }

    // 6. 解析 JSON，只解析一次
    let result: CoordinateResponse

    try {
      result = JSON.parse(responseText) as CoordinateResponse
    } catch {
      throw new Error(
        `服务器返回的不是合法 JSON：${responseText.slice(0, 200)}`,
      )
    }

    // 7. 检查业务处理结果
    if (result.success !== true) {
      throw new Error(
        result.message || '后端未能成功接收地块坐标',
      )
    }

    // 8. 显示成功结果
    resultType.value = 'success'
    resultMessage.value =
      result.message || `成功提交 ${coordinates.length} 个角点`

    ElMessage.success('地块坐标提交成功')

    console.log('[地块坐标] 提交成功：', result)
  } catch (error: unknown) {
    resultType.value = 'error'

    resultMessage.value =
      error instanceof Error
        ? error.message
        : '提交失败，请检查网络或后端服务'

    console.error('[地块坐标] 提交失败：', error)

    ElMessage.error(resultMessage.value)
  } finally {
    submitting.value = false
  }
}
</script>

<style scoped>
.coordinate-card {
  margin-bottom: 24px;
  border-radius: 10px;
}

.card-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 16px;
}

.card-title {
  font-size: 16px;
  font-weight: 600;
}

.card-description {
  margin-top: 6px;
  color: var(--el-text-color-secondary);
  font-size: 13px;
}

.coordinate-header,
.coordinate-row {
  display: grid;
  grid-template-columns: 48px minmax(0, 1fr) minmax(0, 1fr) 64px;
  align-items: center;
  gap: 12px;
}

.coordinate-header {
  margin-bottom: 8px;
  padding: 0 4px;
  color: var(--el-text-color-secondary);
  font-size: 13px;
}

.coordinate-row {
  margin-bottom: 10px;
}

.point-index {
  text-align: center;
  color: var(--el-text-color-regular);
}

.index-column,
.action-column {
  text-align: center;
}

.coordinate-actions {
  display: flex;
  flex-wrap: wrap;
  gap: 10px;
  margin-top: 20px;
}

.result-alert {
  margin-top: 16px;
}

@media (max-width: 600px) {
  .coordinate-header,
  .coordinate-row {
    grid-template-columns: 32px minmax(0, 1fr) minmax(0, 1fr) 56px;
    gap: 6px;
  }

  .coordinate-header {
    font-size: 11px;
  }

  .coordinate-row :deep(.el-button) {
    padding: 8px;
  }
}
</style>
