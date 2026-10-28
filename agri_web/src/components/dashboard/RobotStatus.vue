<script setup lang="ts">
import { computed } from 'vue'
import { Cpu, Odometer, Compass, Lightning, Location } from '@element-plus/icons-vue'
import { useRealtime, modeText, numberText } from '../../composables/useRealtime'

const { robot: state, devices, poseFresh, robotFresh, safetyFresh, isFresh, refresh } = useRealtime()
const robot = computed(() => {
  const pose = poseFresh.value ? state.value : null
  const gnss = isFresh(devices.value?.gnss) ? devices.value?.gnss : null
  return {
    name: state.value?.robot_id ?? '农业机器人',
    online: poseFresh.value,
    mode: safetyFresh.value ? modeText(devices.value?.safety.owner) : '控制模式未知',
    speed: pose?.speed, heading: pose?.heading, angularSpeed: pose?.angular_speed,
    battery: robotFresh.value ? state.value?.battery : null,
    longitude: gnss?.longitude, latitude: gnss?.latitude,
    x: pose?.position_x, y: pose?.position_y,
    lastUpdated: pose?.last_odom_time ?? '',
  }
})
function formatTime(value: string) {
  return value ? new Date(value).toLocaleTimeString() : '--'
}
defineExpose({ refreshData: refresh })
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
          {{ robot.online ? '位姿在线' : '位姿未更新' }}
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
          MGX23 定位 · 标准底盘接口
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
          {{ numberText(robot.speed) }}
          <span class="status-unit">m/s</span>
        </div>
      </div>

      <div class="status-item">
        <div class="status-label">
          <el-icon><Compass /></el-icon>
          ROS 航向角（东 0°，逆时针）
        </div>
        <div class="status-value">
          {{ numberText(robot.heading, 1) }}°
        </div>
      </div>

      <div class="status-item">
        <div class="status-label">
          <el-icon><Lightning /></el-icon>
          电池电量
        </div>
        <div class="status-value">
          {{ robot.battery == null ? '未接入' : numberText(robot.battery, 0) + '%' }}
        </div>
      </div>
    </div>

    <div v-if="robot.battery != null" class="battery-section">
      <div class="battery-heading">
        <span>电量状态</span>
        <span>{{ numberText(robot.battery, 0) }}%</span>
      </div>

      <el-progress
        :percentage="Math.min(100, Math.max(0, robot.battery))"
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
        <strong>{{ numberText(robot.longitude, 8) }}</strong>
      </div>

      <div class="location-item">
        <span>纬度</span>
        <strong>{{ numberText(robot.latitude, 8) }}</strong>
      </div>
    </div>

    <div class="location-grid">
      <div class="location-item"><span>平面 X / Y（m）</span><strong>{{ numberText(robot.x) }} / {{ numberText(robot.y) }}</strong></div>
      <div class="location-item"><span>角速度（rad/s）</span><strong>{{ numberText(robot.angularSpeed, 3) }}</strong></div>
    </div>

    <el-alert
      class="update-alert"
      :title="
        robot.lastUpdated
          ? `位姿接收时间：${formatTime(robot.lastUpdated)}`
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
