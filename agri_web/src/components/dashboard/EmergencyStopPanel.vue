<script setup lang="ts">
import { onMounted, onUnmounted, ref } from 'vue'
import { ElMessage, ElMessageBox } from 'element-plus'

const API_BASE_URL = 'http://127.0.0.1:8000'

const emergencyActive = ref(true)
const rosConnected = ref(false)
const loading = ref(false)
const errorMessage = ref('')

let pollingTimer: ReturnType<typeof setInterval> | undefined

// ============================================================
// 获取急停状态
// ============================================================
async function fetchEmergencyStatus() {
  try {
    const response = await fetch(
      `${API_BASE_URL}/api/robot/emergency-stop/status`,
    )

    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`)
    }

    const data = await response.json()

    emergencyActive.value = data.emergency_stop
    rosConnected.value = data.ros_connected
    errorMessage.value = ''
  } catch {
    rosConnected.value = false
    errorMessage.value = '无法连接后端，请检查 FastAPI 服务'
  }
}

// ============================================================
// 立即急停：不弹出确认窗口
// ============================================================
async function triggerEmergencyStop() {
  // 防止重复点击
  if (loading.value) {
    return
  }

  // 控制链路未确认时，不发送请求
  if (!rosConnected.value) {
    ElMessage.error('控制链路未确认，无法发送急停指令')
    return
  }

  loading.value = true

  try {
    const response = await fetch(
      `${API_BASE_URL}/api/robot/emergency-stop`,
      {
        method: 'POST',
      },
    )

    const data = await response.json()

    if (!response.ok) {
      throw new Error(data.detail || '急停指令发送失败')
    }

    emergencyActive.value = true

    ElMessage.success('急停指令已发送')

    // 刷新后端状态
    await fetchEmergencyStatus()
  } catch (error) {
    errorMessage.value =
      error instanceof Error
        ? error.message
        : '急停指令发送失败'

    ElMessage.error(errorMessage.value)
  } finally {
    loading.value = false
  }
}

// ============================================================
// 解除急停：保留确认窗口
// ============================================================
async function resetEmergencyStop() {
  if (loading.value) {
    return
  }

  // 仅解除急停时需要确认
  try {
    await ElMessageBox.confirm(
      '解除急停后，机器人可能重新具备运动条件。请确认周围环境安全。',
      '解除急停确认',
      {
        confirmButtonText: '确认解除',
        cancelButtonText: '取消',
        type: 'warning',
      },
    )
  } catch {
    // 用户取消操作
    return
  }

  loading.value = true

  try {
    const response = await fetch(
      `${API_BASE_URL}/api/robot/emergency-stop/reset`,
      {
        method: 'POST',
      },
    )

    const data = await response.json()

    if (!response.ok) {
      throw new Error(data.detail || '解除急停指令发送失败')
    }

    emergencyActive.value = false

    ElMessage.success('解除急停指令已发送')

    // 刷新后端状态
    await fetchEmergencyStatus()
  } catch (error) {
    errorMessage.value =
      error instanceof Error
        ? error.message
        : '解除急停指令发送失败'

    ElMessage.error(errorMessage.value)
  } finally {
    loading.value = false
  }
}

// ============================================================
// 页面生命周期
// ============================================================
onMounted(() => {
  fetchEmergencyStatus()

  // 每秒刷新急停状态
  pollingTimer = setInterval(() => {
    fetchEmergencyStatus()
  }, 1000)
})

onUnmounted(() => {
  if (pollingTimer !== undefined) {
    clearInterval(pollingTimer)
  }
})
</script>

<template>
  <el-card class="emergency-panel" shadow="never">
    <div class="emergency-row">
      <!-- 急停状态 -->
      <div
        class="status-indicator"
        :class="{
          'status-stop': emergencyActive || !rosConnected,
          'status-ready': !emergencyActive && rosConnected,
        }"
      >
        <span class="status-dot"></span>

        <span v-if="!rosConnected">控制链路未确认</span>
        <span v-else-if="emergencyActive">急停锁定</span>
        <span v-else>急停指令已解除</span>
      </div>

      <!-- 急停操作按钮 -->
      <div class="button-group">
        <el-button
          type="danger"
          :loading="loading"
          :disabled="!rosConnected || loading"
          @click="triggerEmergencyStop"
        >
          立即急停
        </el-button>

        <el-button
          type="success"
          :loading="loading"
          :disabled="!rosConnected || !emergencyActive || loading"
          @click="resetEmergencyStop"
        >
          解除急停
        </el-button>
      </div>
    </div>

    <!-- 错误提示 -->
    <el-alert
      v-if="errorMessage"
      class="error-alert"
      :title="errorMessage"
      type="error"
      :closable="false"
      show-icon
    />
  </el-card>
</template>

<style scoped>
.emergency-panel {
  margin-bottom: 18px;
  border-radius: 12px;
}

.emergency-row {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 16px;
  flex-wrap: wrap;
}

/* 急停状态 */
.status-indicator {
  display: flex;
  align-items: center;
  gap: 10px;
  font-size: 14px;
  font-weight: 600;
}

.status-dot {
  width: 10px;
  height: 10px;
  flex-shrink: 0;
  border-radius: 50%;
}

/* 急停锁定或控制链路未确认 */
.status-stop {
  color: var(--el-color-danger);
}

.status-stop .status-dot {
  background: var(--el-color-danger);
}

/* 急停已解除 */
.status-ready {
  color: var(--el-color-success);
}

.status-ready .status-dot {
  background: var(--el-color-success);
}

/* 按钮区域 */
.button-group {
  display: flex;
  align-items: center;
  gap: 10px;
}

.button-group .el-button + .el-button {
  margin-left: 0;
}

/* 错误提示 */
.error-alert {
  margin-top: 12px;
}
</style>