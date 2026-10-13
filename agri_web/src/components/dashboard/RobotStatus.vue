<script setup lang="ts">
import { ref, onMounted, onUnmounted } from 'vue'
import {
  Cpu,
  Odometer,
  Compass,
  Lightning,
  Location,
} from '@element-plus/icons-vue'
import { getRobotStatus } from '../../api/robot'

const robot = ref({
  name: '履带式农业机器人 01',
  online: false,
  mode: '加载中',
  speed: 0,
  heading: 0,
  battery: 0,
  longitude: 120.101,
  latitude: 30.201,
  lastUpdated: '',
})

const loading = ref(false)

let refreshTimer: ReturnType<typeof setInterval> | undefined

/**
 * 获取机器人最新状态
 */
async function refreshData(): Promise<void> {
  // 避免上一次请求尚未完成时重复发起请求
  if (loading.value) return

  loading.value = true

  try {
    const data = await getRobotStatus()

    robot.value.online = data.online

    robot.value.mode =
      data.mode === 'standby'
        ? '待命'
        : data.mode

    robot.value.speed = data.speed ?? 0
    robot.value.heading = data.heading ?? 0

    robot.value.battery = Math.min(
      100,
      Math.max(0, data.battery ?? 0),
    )

    robot.value.lastUpdated = data.timestamp
  } catch (error) {
    console.error('获取机器人状态失败：', error)

    robot.value.online = false
    robot.value.mode = '连接异常'
  } finally {
    loading.value = false
  }
}

/**
 * 格式化最后更新时间
 */
function formatTime(value: string): string {
  if (!value) return '--'

  const date = new Date(value)

  if (Number.isNaN(date.getTime())) {
    return value
  }

  return date.toLocaleTimeString()
}

/**
 * 组件加载后立即获取状态，并每秒自动刷新
 */
onMounted(() => {
  void refreshData()

  refreshTimer = setInterval(() => {
    void refreshData()
  }, 1000)
})

/**
 * 组件销毁时清除定时器
 */
onUnmounted(() => {
  if (refreshTimer !== undefined) {
    clearInterval(refreshTimer)
    refreshTimer = undefined
  }
})

/**
 * 暴露刷新函数，兼容 DashboardView.vue 的手动刷新功能
 */
defineExpose({
  refreshData,
})

</script>

<template>
  <el-card class="panel-card robot-card" shadow="never">
    <template #header>
      <div class="panel-header">
        <span>机器人状态</span>

        <el-tag
          :type="robot.online ? 'success' : 'danger'"
          effect="light"
          size="small"
        >
          {{ robot.online ? '已连接' : '未连接' }}
        </el-tag>
      </div>
    </template>

    <div class="robot-profile">
      <div class="robot-avatar">
        <el-icon :size="26">
          <Cpu />
        </el-icon>
      </div>

      <div class="robot-identity">
        <div class="robot-name">
          {{ robot.name }}
        </div>

        <div class="robot-subtitle">
          履带式农业机器人
        </div>
      </div>

      <span
        class="online-dot"
        :class="{ offline: !robot.online }"
      />
    </div>

    <el-divider />

    <div class="status-grid">
      <div class="status-item">
        <div class="status-label">
          运行模式
        </div>
        <div class="status-value">
          {{ robot.mode }}
        </div>
      </div>

      <div class="status-item">
        <div class="status-label">
          <el-icon><Odometer /></el-icon>
          行驶速度
        </div>
        <div class="status-value">
          {{ robot.speed.toFixed(2) }}
          <span class="status-unit">m/s</span>
        </div>
      </div>

      <div class="status-item">
        <div class="status-label">
          <el-icon><Compass /></el-icon>
          航向角
        </div>
        <div class="status-value">
          {{ robot.heading.toFixed(1) }}°
        </div>
      </div>

      <div class="status-item">
        <div class="status-label">
          <el-icon><Lightning /></el-icon>
          电池电量
        </div>
        <div class="status-value">
          {{ robot.battery.toFixed(0) }}%
        </div>
      </div>
    </div>

    <div class="battery-section">
      <div class="battery-heading">
        <span>电量状态</span>
        <span>{{ robot.battery.toFixed(0) }}%</span>
      </div>

      <el-progress
        :percentage="robot.battery"
        :stroke-width="8"
        :show-text="false"
        status="success"
      />
    </div>

    <el-divider />

    <div class="location-heading">
      <el-icon><Location /></el-icon>
      GNSS 定位信息
    </div>

    <div class="location-grid">
      <div class="location-item">
        <span>经度</span>
        <strong>{{ robot.longitude.toFixed(6) }}</strong>
      </div>

      <div class="location-item">
        <span>纬度</span>
        <strong>{{ robot.latitude.toFixed(6) }}</strong>
      </div>
    </div>

    <el-alert
      class="update-alert"
      :title="
        robot.lastUpdated
          ? `最近更新时间：${formatTime(robot.lastUpdated)}`
          : '等待机器人状态数据'
      "
      :type="robot.online ? 'success' : 'warning'"
      :closable="false"
      show-icon
    />
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
  gap: 12px;
  color: #263b30;
  font-size: 15px;
  font-weight: 600;
}

.robot-profile {
  display: flex;
  align-items: center;
  gap: 12px;
}

.robot-avatar {
  display: flex;
  width: 52px;
  height: 52px;
  flex-shrink: 0;
  align-items: center;
  justify-content: center;
  border-radius: 12px;
  background: #e9f5ee;
  color: #25845c;
}

.robot-identity {
  flex: 1;
  min-width: 0;
}

.robot-name {
  color: #263b30;
  font-size: 14px;
  font-weight: 600;
}

.robot-subtitle {
  margin-top: 6px;
  color: #89958d;
  font-size: 12px;
}

.online-dot {
  width: 9px;
  height: 9px;
  flex-shrink: 0;
  border-radius: 50%;
  background: #36a66b;
}

.online-dot.offline {
  background: #d9534f;
}

.status-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 12px;
}

.status-item {
  padding: 14px;
  border-radius: 8px;
  background: #f7faf7;
}

.status-label {
  display: flex;
  align-items: center;
  gap: 5px;
  color: #7b8980;
  font-size: 12px;
}

.status-value {
  margin-top: 9px;
  color: #263b30;
  font-size: 20px;
  font-weight: 600;
  overflow-wrap: anywhere;
}

.status-unit {
  color: #89958d;
  font-size: 11px;
  font-weight: 400;
}

.battery-section {
  margin-top: 20px;
}

.battery-heading {
  display: flex;
  justify-content: space-between;
  margin-bottom: 10px;
  color: #65736a;
  font-size: 12px;
}

.location-heading {
  display: flex;
  align-items: center;
  gap: 6px;
  margin-bottom: 14px;
  color: #43564a;
  font-size: 13px;
  font-weight: 600;
}

.location-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 10px;
}

.location-item {
  display: flex;
  flex-direction: column;
  gap: 7px;
  padding: 12px;
  border: 1px solid #e8eee9;
  border-radius: 8px;
}

.location-item span {
  color: #89958d;
  font-size: 11px;
}

.location-item strong {
  color: #25845c;
  font-size: 13px;
  overflow-wrap: anywhere;
}

.update-alert {
  margin-top: 18px;
}

@media (max-width: 480px) {
  .status-value {
    font-size: 17px;
  }

  .location-grid {
    grid-template-columns: 1fr;
  }
}
</style>
