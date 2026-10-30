<script setup lang="ts">
import { computed, ref } from 'vue'
import { ElMessage } from 'element-plus'
import { useRealtime, taskText, numberText } from '../../composables/useRealtime'
import { useWorkspace } from '../../composables/useWorkspace'
import { taskAction, cancelPlanning } from '../../api/workspace'

const { devices, safetyFresh, isFresh, refresh } = useRealtime()
const { context, refresh: refreshWorkspace } = useWorkspace()
const busy = ref(false)
const source = computed(() => devices.value?.safety.mode === 'waypoint' ? 'waypoint' : 'tracking')
const status = computed(() => devices.value?.[source.value])
const state = computed(() => isFresh(status.value) ? status.value?.state : undefined)
const planning = computed(() => isFresh(devices.value?.planning) && devices.value?.planning.state === 'PLANNING')
const displayState = computed(() => planning.value ? 'PLANNING' : context.value?.active_run?.stage === 'planning'
  ? context.value.active_run.state : state.value)
const coveragePhase = computed(() => source.value === 'tracking' && isFresh(status.value)
  && ['RUNNING', 'PAUSED', 'ACQUIRING'].includes(state.value || '') ? status.value?.phase : undefined)
const progress = computed(() => {
  if (!isFresh(status.value)) return null
  if (status.value?.state === 'COMPLETED') return 100
  if (source.value === 'tracking' && status.value?.length) return Math.min(100, 100 * (status.value.progress || 0) / status.value.length)
  if (source.value === 'waypoint' && status.value?.count) return Math.min(100, 100 * (status.value.index || 0) / status.value.count)
  return null
})
async function operate(action: 'pause' | 'resume' | 'cancel' | 'cancel_plan') {
  busy.value = true
  try {
    if (action === 'cancel_plan') await cancelPlanning()
    else await taskAction(source.value, action)
    ElMessage.success('操作已确认，请查看实际任务状态')
  } catch (error) { ElMessage.error(error instanceof Error ? error.message : '任务操作失败') }
  finally { busy.value = false; await Promise.all([refresh(), refreshWorkspace()]) }
}
</script>
<template>
  <div class="task-controls">
    <div><strong>{{ context?.active_run?.name || '当前任务' }}<span v-if="context?.active_run"> · v{{ context.active_run.mission_revision }}</span></strong><el-tag type="info">{{ taskText(displayState) }}</el-tag><small>{{ context?.active_run?.reason || status?.reason || '任务开始后在此查看反馈' }}</small><small v-if="state && state !== 'IDLE'">用时 {{ numberText(status?.elapsed_seconds, 1) }} s（含暂停）<span v-if="progress !== null"> · {{ source === 'tracking' ? '路径任务进度' : '目标点完成进度' }} {{ numberText(progress, 1) }}%</span></small></div>
    <div v-if="coveragePhase" class="coverage-phase" role="status">
      <strong>{{ coveragePhase === 'APPROACHING' ? '前往覆盖起点（单点导航）' : '覆盖路径跟踪' }}</strong>
      <small v-if="coveragePhase === 'APPROACHING'">距起点 {{ numberText(status?.approach_distance, 2) }} m · 到达后自动开始覆盖路径跟踪</small>
    </div>
    <div class="buttons">
      <el-button :disabled="busy || state !== 'RUNNING'" @click="operate('pause')">暂停任务</el-button>
      <el-button type="success" :disabled="busy || state !== 'PAUSED' || !safetyFresh || devices?.safety.motion_inhibited !== false" @click="operate('resume')">继续任务</el-button>
      <el-button type="danger" plain :disabled="busy || !['RUNNING', 'PAUSED', 'ACQUIRING'].includes(state || '')" @click="operate('cancel')">取消任务</el-button>
      <el-button v-if="planning" type="warning" :disabled="busy" @click="operate('cancel_plan')">取消规划</el-button>
    </div>
  </div>
</template>
<style scoped>
.task-controls { display: flex; align-items: center; justify-content: space-between; flex-wrap: wrap; gap: 14px; padding: 18px; background: #edf5ef; border: 1px solid #dce9df; border-radius: 10px; margin-bottom: 20px; }
.task-controls { position: sticky; top: 0; z-index: 4; box-shadow: 0 3px 12px #193d2410; }
.task-controls strong { margin-right: 12px; }.task-controls small { display: block; margin-top: 8px; color: #758379; }.buttons { display: flex; flex-wrap: wrap; gap: 8px; }.buttons .el-button { margin: 0; }
</style>
