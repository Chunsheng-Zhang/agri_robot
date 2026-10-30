<script setup lang="ts">
import { computed, ref, watch } from 'vue'
import { ElMessage } from 'element-plus'
import { resetObstacle, setLidarProtection } from '../../api/workspace'
import { useRealtime, modeText, numberText } from '../../composables/useRealtime'

const { devices, safetyFresh, isFresh, error, refresh } = useRealtime()
const resetting = ref(false)
const changingLidar = ref(false)
const lidarSelection = ref(false)
async function changeLidar(value: boolean | string | number) {
  changingLidar.value = true
  try {
    const result = await setLidarProtection(value === true)
    ElMessage.success(`${result.message}；切换后需重新取得控制权或启动任务`)
  } catch (value) {
    ElMessage.error(value instanceof Error ? value.message : '雷达保护切换失败，请查看实际状态')
  } finally {
    await refresh()
    lidarSelection.value = safety.value?.lidar_required === true
    changingLidar.value = false
  }
}
async function clearObstacle() {
  resetting.value = true
  try { await resetObstacle(); ElMessage.success('障碍锁定已解除，任务仍需明确继续') }
  catch (value) { ElMessage.error(value instanceof Error ? value.message : '复位被拒绝') }
  finally { resetting.value = false; await refresh() }
}
const safety = computed(() => safetyFresh.value ? devices.value?.safety : null)
watch(() => safety.value?.lidar_required, value => {
  if (!changingLidar.value) lidarSelection.value = value === true
}, { immediate: true })
const obstacles = computed(() => isFresh(devices.value?.obstacles) && devices.value?.obstacles.valid ? devices.value.obstacles : null)
const cards = computed(() => [
  { label: '当前控制模式', text: safety.value ? modeText(safety.value.owner) : '未知', bad: !safety.value },
  { label: '手柄安全通道', text: safety.value ? (safety.value.joy_connected ? '在线' : '未连接') : '未知', bad: !safety.value?.joy_connected },
  { label: '雷达障碍停车', text: typeof safety.value?.lidar_required === 'boolean' ? (safety.value.lidar_required === false ? '已关闭' : safety.value.lidar_connected ? '已启用' : '已启用 / 等待有效雷达') : '未知', bad: !safety.value || (safety.value.lidar_required !== false && !safety.value.lidar_connected) },
  { label: '定位状态', text: safety.value ? (safety.value.localization_valid ? '有效' : '无效 / 超时') : '未知', bad: !safety.value?.localization_valid },
  { label: '软件急停锁定', text: typeof safety.value?.emergency_stop === 'boolean' ? (safety.value.emergency_stop ? '已锁定' : '已解除') : '未知', bad: safety.value?.emergency_stop !== false },
  { label: '障碍物停车', text: typeof safety.value?.obstacle_stop === 'boolean' ? (safety.value.obstacle_stop ? '已锁定' : '未锁定') : '未知', bad: safety.value?.obstacle_stop !== false },
])
</script>

<template>
  <el-card shadow="never" class="safety-panel">
    <template #header><div class="heading"><span>实时设备与安全状态</span><el-tag :type="safetyFresh ? 'success' : 'warning'">{{ safetyFresh ? '状态新鲜' : '等待有效状态' }}</el-tag></div></template>
    <div class="lidar-control">
      <el-checkbox v-model="lidarSelection"
        :disabled="changingLidar || typeof safety?.lidar_required !== 'boolean'" @change="changeLidar">
        启用雷达障碍停车
      </el-checkbox>
      <span>{{ changingLidar ? '正在切换，请等待安全节点反馈…' : '默认关闭；切换会停车并释放控制权，任务需重新启动。' }}</span>
    </div>
    <el-alert v-if="safety?.lidar_front_only" title="雷达仅检测车头前方 180°；允许导航和转向，后方与侧后方无雷达保护。" type="warning" :closable="false" />
    <div class="safety-grid">
      <div v-for="card in cards" :key="card.label" class="safety-item">
        <span>{{ card.label }}</span><el-tag :type="card.bad ? 'warning' : 'success'">{{ card.text }}</el-tag>
      </div>
    </div>
    <div class="distances">
      <span>{{ safety?.lidar_front_only ? '检测范围内最近' : '全局最近' }}：<strong>{{ numberText(obstacles?.global_range) }} m</strong></span>
      <span>前方：<strong>{{ numberText(obstacles?.front_valid ? obstacles.front_distance : null) }} m</strong></span>
      <span v-if="safety?.lidar_front_only">后方：<strong>未检测</strong></span>
      <span v-else>后方：<strong>{{ numberText(obstacles?.rear_valid ? obstacles.rear_distance : null) }} m</strong></span>
    </div>
    <p v-if="typeof safety?.obstacle_fraction === 'number'" class="hint">
      停车区域障碍占比：{{ typeof obstacles?.stop_fraction === 'number' ? numberText(obstacles.stop_fraction * 100) + '%' : '未更新' }}
      （{{ obstacles?.stop_points ?? '—' }}/{{ obstacles?.total_beams ?? '—' }} 点）；
      达到 {{ numberText(safety.obstacle_fraction * 100) }}% 停车。零值和无返回不计为障碍，细小障碍可能低于门槛。
    </p>
    <el-button v-if="safety?.obstacle_stop" type="warning" plain :loading="resetting" :disabled="!obstacles?.recovery_ready" @click="clearObstacle">{{ obstacles?.recovery_ready ? '人工解除障碍停车' : '等待持续有效的恢复净空' }}</el-button>
    <el-alert :title="error || safety?.reason || '等待 ROS 2 安全模块'" :type="safety?.motion_inhibited === false ? 'info' : 'warning'" :closable="false" />
    <p class="hint">最近距离用于观察，停车依据由安全模块按运动方向和车体净空判定。定位有效不代表已确认 RTK 固定解。</p>
  </el-card>
</template>

<style scoped>
.safety-panel { border: 1px solid #e8eee9; border-radius: 10px; margin-bottom: 20px; }
.heading { display: flex; justify-content: space-between; align-items: center; gap: 10px; }
.lidar-control { display: flex; align-items: center; flex-wrap: wrap; gap: 4px 16px; margin-bottom: 14px; }
.lidar-control span { color: #7b8980; font-size: 12px; line-height: 1.6; }
.safety-grid { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 12px; }
.safety-item { display: flex; justify-content: space-between; align-items: center; gap: 8px; padding: 12px; background: #f6f9f7; border-radius: 8px; font-size: 13px; }
.distances { display: flex; gap: 24px; flex-wrap: wrap; padding: 18px 0; font-size: 14px; }
.hint { color: #7b8980; font-size: 12px; line-height: 1.6; margin: 12px 0 0; }
@media (max-width: 900px) { .safety-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); } }
@media (max-width: 500px) { .safety-grid { grid-template-columns: 1fr; } }
</style>
