<script setup lang="ts">
import { computed, ref, onUnmounted } from 'vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import { useRealtime } from '../composables/useRealtime'
import { useWorkspace } from '../composables/useWorkspace'
import { modeLabels, selectMode, workModes, workModeText } from '../api/workspace'
import type { WorkMode } from '../api/workspace'

const { devices, safetyFresh, isFresh, refresh } = useRealtime()
const { context, refresh: refreshWorkspace } = useWorkspace()
const props = withDefaults(defineProps<{ modes?: readonly WorkMode[] }>(), {
  modes: () => workModes,
})
const availableModes = computed(() => props.modes.map(mode => ({ mode, label: modeLabels[mode] })))
const busy = ref(false)
let disposed = false
onUnmounted(() => { disposed = true })
const current = computed(() => safetyFresh.value ? devices.value?.safety.mode : undefined)
async function change(mode: WorkMode) {
  if (disposed || busy.value || current.value === mode || !props.modes.includes(mode) || !safetyFresh.value) return
  busy.value = true
  try {
    const executing = ['waypoint', 'tracking', 'following', 'planning'].some(source => {
      const status = devices.value?.[source as 'waypoint' | 'tracking' | 'following' | 'planning']
      return isFresh(status) && ['ACQUIRING', 'RUNNING', 'PAUSED', 'PLANNING'].includes(status?.state || '')
    })
    const teaching = context.value?.teaching?.state === 'RECORDING'
    const processing = context.value?.teaching?.state === 'PROCESSING'
    if (devices.value?.safety.owner || executing || teaching || processing || ['ACQUIRING', 'RUNNING', 'PAUSED', 'PLANNING'].includes(context.value?.active_run?.state || '')) {
      await ElMessageBox.confirm(teaching
        ? '切换会终止正在录制的示教轨迹，原始记录保留。新模式保持待命，不会自动行驶。'
        : processing ? '示教采样已结束，后台路径处理继续。切换将结束当前运动任务，新模式保持待命。'
        : '切换将停止并结束当前任务。新模式保持待命，不会自动行驶。', '切换工作模式', {
        type: 'warning', confirmButtonText: '停止并切换', cancelButtonText: '取消',
      })
    }
    if (disposed || !safetyFresh.value || current.value === mode) return
    await selectMode(mode); ElMessage.success('切换请求已确认，请核对当前模式')
  } catch (error) { if (error !== 'cancel' && error !== 'close') ElMessage.error(error instanceof Error ? error.message : '切换失败') }
  finally { busy.value = false; if (!disposed) await Promise.all([refresh(), refreshWorkspace()]) }
}
</script>

<template>
  <section class="mode-selector">
    <div><span class="eyebrow">运行模式</span><strong>{{ current ? workModeText(current) : '等待 ROS 核心' }}</strong><span class="mode-note">切换后待命，任务需明确开始；手柄 / Web 遥控与示教录制使用基础待命模式</span></div>
    <div class="mode-buttons">
      <el-tag v-if="context?.synthetic" type="warning">软件联调 · 模拟定位</el-tag>
      <el-button v-for="{ label, mode } in availableModes" :key="mode" :type="current === mode ? 'success' : 'default'"
        :plain="current !== mode" :disabled="busy || !safetyFresh" :loading="busy && current !== mode" @click="change(mode)">{{ label }}</el-button>
    </div>
  </section>
</template>

<style scoped>
.mode-selector { background: white; border: 1px solid #e1e9e4; border-radius: 10px; padding: 16px 20px; display: flex; align-items: center; justify-content: space-between; gap: 16px; flex-wrap: wrap; margin-bottom: 22px; }
.mode-selector strong { margin-left: 12px; color: #23754f; font-size: 18px; }
.eyebrow { color: #718178; font-size: 13px; }.mode-note { display: block; font-size: 12px; color: #819087; margin-top: 7px; }
.mode-buttons { display: flex; flex-wrap: wrap; gap: 8px; }.mode-buttons .el-button { margin: 0; }
</style>
