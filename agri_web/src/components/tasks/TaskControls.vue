<script setup lang="ts">
import { computed, ref } from 'vue'
import { ElMessage } from 'element-plus'
import { useRealtime, taskText, numberText } from '../../composables/useRealtime'
import { useWorkspace } from '../../composables/useWorkspace'
import { taskAction, cancelPlanning } from '../../api/workspace'
import type { MissionMode } from '../../api/workspace'

const props = defineProps<{ kind: MissionMode }>()
const { devices, safetyFresh, isFresh, refresh } = useRealtime()
const { context, refresh: refreshWorkspace } = useWorkspace()
const busy = ref(false)
const source = computed(() => props.kind === 'waypoint' ? 'waypoint' : 'tracking')
const activeRun = computed(() => context.value?.active_run?.mode === props.kind ? context.value.active_run : undefined)
const status = computed(() => devices.value?.[source.value])
const state = computed(() => isFresh(status.value) ? status.value?.state : undefined)
const planning = computed(() => props.kind === 'coverage' && isFresh(devices.value?.planning) && devices.value?.planning.state === 'PLANNING')
const planningProgress = computed(() => planning.value ? devices.value?.planning.search_progress : null)
const planningPhases: Record<string,string> = { INITIALIZING: '准备地块与评价网格', COARSE: '作业角度粗搜索',
  REFINE: '较优角度细化搜索', FINALIZING: '汇总评分与校验路径', COMPLETE: '搜索完成，等待最终校验' }
const displayState = computed(() => planning.value ? 'PLANNING' : activeRun.value?.stage === 'planning'
  ? activeRun.value.state : state.value)
const canPause = computed(() => !busy.value && state.value === 'RUNNING')
const canResume = computed(() => !busy.value && state.value === 'PAUSED' && safetyFresh.value
  && devices.value?.safety.mode === props.kind && devices.value?.safety.motion_inhibited === false)
const canCancel = computed(() => !busy.value && ['RUNNING', 'PAUSED', 'ACQUIRING'].includes(state.value || ''))
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
  if (busy.value || (action === 'pause' && !canPause.value) || (action === 'resume' && !canResume.value)
    || (action === 'cancel' && !canCancel.value) || (action === 'cancel_plan' && !planning.value)) return
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
    <div><strong>{{ activeRun?.name || (kind === 'waypoint' ? '多点导航任务' : '覆盖作业任务') }}<span v-if="activeRun"> · v{{ activeRun.mission_revision }}</span></strong><el-tag type="info">{{ taskText(displayState) }}</el-tag><small>{{ activeRun?.reason || status?.reason || '任务开始后在此查看反馈' }}</small><small v-if="state && state !== 'IDLE'">用时 {{ numberText(status?.elapsed_seconds, 1) }} s（含暂停）<span v-if="progress !== null"> · {{ source === 'tracking' ? '路径任务进度' : '目标点完成进度' }} {{ numberText(progress, 1) }}%</span></small></div>
    <div v-if="planningProgress" class="planning-progress" role="status" aria-live="polite">
      <strong>智能规划进度：{{ planningPhases[planningProgress.phase] || planningProgress.phase }}</strong>
      <small>已比较 {{ planningProgress.evaluated_count }} 套候选 · 可执行 {{ planningProgress.feasible_count }} 套 · 已用时 {{ numberText(planningProgress.elapsed_seconds, 1) }} s</small>
      <template v-if="planningProgress.stage_total > 0">
        <small>本阶段方向进度 {{ planningProgress.stage_completed }} / {{ planningProgress.stage_total }}<span v-if="planningProgress.angle_rad != null"> · 当前方向 {{ numberText(planningProgress.angle_rad * 180 / Math.PI, 2) }}°</span></small>
        <progress :value="planningProgress.stage_completed" :max="planningProgress.stage_total" aria-label="本阶段作业方向比较进度" />
      </template>
    </div>
    <div v-if="coveragePhase" class="coverage-phase" role="status">
      <strong>{{ coveragePhase === 'APPROACHING' ? '前往覆盖起点（单点导航）' : coveragePhase === 'ALIGNING' ? '航向对齐中（原地转向）' : '覆盖路径跟踪' }}</strong>
      <small v-if="coveragePhase === 'APPROACHING'">距起点 {{ numberText(status?.approach_distance, 2) }} m · 到达后先对齐航向，再开始覆盖跟踪</small>
      <small v-if="coveragePhase === 'ALIGNING'">{{ status?.alignment_next_phase === 'APPROACHING' ? '接近起点方向' : '覆盖路径方向' }} · 航向误差 {{ numberText(status?.alignment_error == null ? null : status.alignment_error * 180 / Math.PI, 1) }}° · 对齐期间不前进</small>
    </div>
    <div class="buttons">
      <el-button :disabled="!canPause" @click="operate('pause')">暂停任务</el-button>
      <el-button type="success" :disabled="!canResume" @click="operate('resume')">继续任务</el-button>
      <el-button type="danger" plain :disabled="!canCancel" @click="operate('cancel')">取消任务</el-button>
      <el-button v-if="planning" type="warning" :disabled="busy" @click="operate('cancel_plan')">取消规划</el-button>
    </div>
  </div>
</template>
<style scoped>
.task-controls { display: flex; align-items: center; justify-content: space-between; flex-wrap: wrap; gap: 14px; padding: 18px; background: #edf5ef; border: 1px solid #dce9df; border-radius: 10px; margin-bottom: 20px; }
.task-controls { position: sticky; top: 0; z-index: 4; box-shadow: 0 3px 12px #193d2410; }
.task-controls strong { margin-right: 12px; }.task-controls small { display: block; margin-top: 8px; color: #758379; }.buttons { display: flex; flex-wrap: wrap; gap: 8px; }.buttons .el-button { margin: 0; }
.planning-progress { flex-basis: 100%; } .planning-progress progress { width: min(100%, 460px); margin-top: 10px; accent-color: #40916c; }
</style>
