<script setup lang="ts">
import { computed, ref } from 'vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import { emergencyStop } from '../api/robot'
import { useRealtime } from '../composables/useRealtime'

const { devices, safetyFresh, refresh } = useRealtime()
const busy = ref(false)
const active = computed(() => safetyFresh.value ? devices.value?.safety.emergency_stop : null)
const known = computed(() => typeof active.value === 'boolean')
async function stop() {
  if (busy.value) return
  busy.value = true
  try {
    await emergencyStop(true)
    ElMessage.success('安全服务已确认急停请求')
  } catch (error) {
    ElMessage.error(error instanceof Error ? error.message : '急停请求失败')
  } finally {
    busy.value = false
    await refresh()
  }
}
async function reset() {
  if (busy.value || !known.value || active.value !== true) return
  try {
    await ElMessageBox.confirm('确认现场安全、急停按钮松开且手柄回中。复位后不会自动开始任务。', '解除软件急停', {
      type: 'warning', confirmButtonText: '确认复位', cancelButtonText: '取消',
    })
  } catch { return }
  if (!safetyFresh.value || active.value !== true) {
    ElMessage.warning('安全状态已过期，请等待刷新')
    return
  }
  busy.value = true
  try {
    await emergencyStop(false)
    ElMessage.success('复位服务已确认，请核对实时安全状态')
  } catch (error) {
    ElMessage.error(error instanceof Error ? error.message : '复位请求被拒绝')
  } finally {
    busy.value = false
    await refresh()
  }
}
</script>

<template>
  <div class="emergency-controls">
    <el-tag :type="!known ? 'warning' : active ? 'danger' : 'success'" role="status">
      {{ !known ? '安全状态未知' : active ? '急停锁定' : '急停已解除' }}
    </el-tag>
    <el-button type="danger" size="small" :loading="busy" :disabled="busy" @click="stop">立即急停</el-button>
    <el-button type="success" size="small" :disabled="busy || !known || active !== true" @click="reset">解除急停</el-button>
  </div>
</template>

<style scoped>
.emergency-controls { display: flex; align-items: center; justify-content: center; gap: 8px; flex-wrap: wrap; }
</style>
