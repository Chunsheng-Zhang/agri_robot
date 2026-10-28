<script setup lang="ts">
import { computed } from 'vue'
import { useRealtime, modeText, numberText } from '../../composables/useRealtime'

const { devices, safetyFresh, isFresh, error } = useRealtime()
const safety = computed(() => safetyFresh.value ? devices.value?.safety : null)
const obstacles = computed(() => isFresh(devices.value?.obstacles) && devices.value?.obstacles.valid ? devices.value.obstacles : null)
const cards = computed(() => [
  { label: '当前控制模式', text: safety.value ? modeText(safety.value.owner) : '未知', bad: !safety.value },
  { label: '手柄安全通道', text: safety.value ? (safety.value.joy_connected ? '在线' : '未连接') : '未知', bad: !safety.value?.joy_connected },
  { label: '雷达数据', text: safety.value ? (safety.value.lidar_connected ? '有效' : '无效 / 超时') : '未知', bad: !safety.value?.lidar_connected },
  { label: '定位状态', text: safety.value ? (safety.value.localization_valid ? '有效' : '无效 / 超时') : '未知', bad: !safety.value?.localization_valid },
  { label: '软件急停锁定', text: typeof safety.value?.emergency_stop === 'boolean' ? (safety.value.emergency_stop ? '已锁定' : '已解除') : '未知', bad: safety.value?.emergency_stop !== false },
  { label: '障碍物停车', text: typeof safety.value?.obstacle_stop === 'boolean' ? (safety.value.obstacle_stop ? '已锁定' : '未锁定') : '未知', bad: safety.value?.obstacle_stop !== false },
])
</script>

<template>
  <el-card shadow="never" class="safety-panel">
    <template #header><div class="heading"><span>实时设备与安全状态</span><el-tag :type="safetyFresh ? 'success' : 'warning'">{{ safetyFresh ? '状态新鲜' : '等待有效状态' }}</el-tag></div></template>
    <div class="safety-grid">
      <div v-for="card in cards" :key="card.label" class="safety-item">
        <span>{{ card.label }}</span><el-tag :type="card.bad ? 'warning' : 'success'">{{ card.text }}</el-tag>
      </div>
    </div>
    <div class="distances">
      <span>全局最近：<strong>{{ numberText(obstacles?.global_range) }} m</strong></span>
      <span>前方：<strong>{{ numberText(obstacles?.front_valid ? obstacles.front_distance : null) }} m</strong></span>
      <span>后方：<strong>{{ numberText(obstacles?.rear_valid ? obstacles.rear_distance : null) }} m</strong></span>
    </div>
    <el-alert :title="error || safety?.reason || '等待 ROS 2 安全模块'" :type="safety?.motion_inhibited === false ? 'info' : 'warning'" :closable="false" />
    <p class="hint">最近距离用于观察，停车依据由安全模块按运动方向和车体净空判定。定位有效不代表已确认 RTK 固定解。</p>
  </el-card>
</template>

<style scoped>
.safety-panel { border: 1px solid #e8eee9; border-radius: 10px; margin-bottom: 20px; }
.heading { display: flex; justify-content: space-between; align-items: center; gap: 10px; }
.safety-grid { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 12px; }
.safety-item { display: flex; justify-content: space-between; align-items: center; gap: 8px; padding: 12px; background: #f6f9f7; border-radius: 8px; font-size: 13px; }
.distances { display: flex; gap: 24px; flex-wrap: wrap; padding: 18px 0; font-size: 14px; }
.hint { color: #7b8980; font-size: 12px; line-height: 1.6; margin: 12px 0 0; }
@media (max-width: 900px) { .safety-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); } }
@media (max-width: 500px) { .safety-grid { grid-template-columns: 1fr; } }
</style>
