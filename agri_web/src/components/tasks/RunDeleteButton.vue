<script setup lang="ts">
import { computed, ref } from 'vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import { deleteRun } from '../../api/workspace'
import type { Run } from '../../api/workspace'
import { useWorkspace } from '../../composables/useWorkspace'

const props = defineProps<{ run: Run }>()
const { refresh } = useWorkspace()
const busy = ref(false)
const canDelete = computed(() => ['COMPLETED', 'CANCELED', 'FAILED', 'FAULT', 'READY'].includes(props.run.state))
async function remove() {
  if (busy.value || !canDelete.value) return
  busy.value = true
  try {
    const run = props.run
    const readyNote = run.state === 'READY' ? '删除此规划记录后，开始覆盖前需重新生成路径。' : ''
    try {
      await ElMessageBox.confirm(
        `删除“${run.name}”（v${run.mission_revision}，${new Date(run.created_at).toLocaleString('zh-CN')}）这条运行记录？原始实验数据和已保存的任务配置会保留。${readyNote}`,
        '删除运行记录', { type: 'warning', confirmButtonText: '删除记录', cancelButtonText: '取消' })
    } catch { return }
    if (!canDelete.value || props.run.id !== run.id) return
    await deleteRun(run.id)
    ElMessage.success('记录已删除，原始实验数据已保留')
    await refresh()
  } catch (error) {
    ElMessage.error(error instanceof Error ? error.message : '删除记录失败')
  } finally { busy.value = false }
}
</script>
<template>
  <el-button type="danger" link :disabled="busy || !canDelete" :loading="busy"
    :title="canDelete ? '删除记录条目，保留原始实验数据' : '任务结束或取消后可删除记录'" @click="remove">删除</el-button>
</template>
