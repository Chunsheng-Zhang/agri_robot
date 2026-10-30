<script setup lang="ts">
import { computed } from 'vue'
import { useWorkspace } from '../../composables/useWorkspace'
import { taskText } from '../../composables/useRealtime'
import { runDownloadUrl } from '../../api/workspace'
import type { MissionMode } from '../../api/workspace'
import RunDeleteButton from './RunDeleteButton.vue'

const props = defineProps<{ kind: MissionMode }>()
const { context } = useWorkspace()
const runs = computed(() => context.value?.runs.filter(run => run.mode === props.kind) || [])
const inProgress = (state: string) => ['PREPARING', 'PLANNING', 'ACQUIRING', 'RUNNING', 'PAUSED'].includes(state)
</script>
<template>
  <el-card shadow="never" class="task-history">
    <template #header><strong>{{ kind === 'waypoint' ? '多点导航运行记录' : '覆盖规划与跟踪记录' }}</strong></template>
    <el-table :data="runs" max-height="360" empty-text="该类任务暂无运行记录">
      <el-table-column prop="name" label="任务" min-width="160" />
      <el-table-column label="版本" width="75"><template #default="{ row }">v{{ row.mission_revision }}</template></el-table-column>
      <el-table-column label="状态" width="120"><template #default="{ row }">{{ taskText(row.state) }}</template></el-table-column>
      <el-table-column label="创建时间" min-width="185"><template #default="{ row }">{{ new Date(row.created_at).toLocaleString('zh-CN') }}</template></el-table-column>
      <el-table-column prop="reason" label="说明" min-width="200" />
      <el-table-column label="归档" width="140"><template #default="{ row }"><el-button tag="a" :href="runDownloadUrl(row.id)" :disabled="inProgress(row.state)" type="success" link>下载任务数据</el-button></template></el-table-column>
      <el-table-column label="操作" width="85" fixed="right"><template #default="{ row }"><RunDeleteButton :run="row" /></template></el-table-column>
    </el-table>
    <p>显示最近 50 次 Web 运行中属于本栏目的记录。任务结束后可下载配置、规划和轨迹数据；全部记录也可在“系统设置”查看。删除仅移除记录条目，原始实验数据保留。</p>
  </el-card>
</template>
<style scoped>
.task-history { margin-top: 22px; min-width: 0; }p { color: #819087; font-size: 12px; line-height: 1.8; margin-bottom: 0; }
</style>
