<script setup lang="ts">
import { computed, ref, watch } from 'vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import { useWorkspace } from '../../composables/useWorkspace'
import { deleteMission } from '../../api/workspace'
import type { MissionDocument, MissionMode } from '../../api/workspace'
import TaskEditor from './TaskEditor.vue'
import TaskControls from './TaskControls.vue'
import TaskRunHistory from './TaskRunHistory.vue'
import ModeSelector from '../ModeSelector.vue'

const props = defineProps<{ kind: MissionMode }>()
const { context, error, loading, refresh } = useWorkspace()
const kind = computed(() => props.kind), selectedId = ref<string>()
const title = computed(() => kind.value === 'waypoint' ? '多点导航' : '全覆盖规划与跟踪')
const workflow = computed(() => kind.value === 'waypoint'
  ? ['配置目标点与导航参数', '检查并保存任务', '开始导航与执行控制']
  : ['编辑边界与禁入区域', '保存任务与生成路径', '接近起点并跟踪覆盖路径'])
watch(kind, () => { selectedId.value = undefined })
const missions = computed(() => context.value?.missions.filter(p => p.mode === kind.value) || [])
const selected = computed<MissionDocument | null>(() => {
  const value = missions.value.find(p => p.id === selectedId.value)
  return value ? JSON.parse(JSON.stringify(value)) : null
})
async function remove(id: string) {
  try {
    await ElMessageBox.confirm('删除任务配置不会删除已有实验记录。', '删除任务', { confirmButtonText: '删除', cancelButtonText: '取消', type: 'warning' })
  } catch { return }
  try { await deleteMission(id); if (id === selectedId.value) selectedId.value = undefined; await refresh() }
  catch (value) { ElMessage.error(value instanceof Error ? value.message : '删除失败') }
}
function saved(value: MissionDocument) { if (value.mode === kind.value) selectedId.value = value.id }
</script>
<template>
  <div class="task-workspace">
    <div class="page-title"><div><h1>{{ title }}</h1><p>{{ kind === 'waypoint' ? '在此完成航点编辑、导航参数设置与任务执行。' : '在此完成农田边界编辑、覆盖路径生成与路径跟踪。' }}</p></div><el-button :loading="loading" @click="refresh">刷新任务</el-button></div>
    <div class="workflow"><span v-for="(step, index) in workflow" :key="step"><b>{{ index + 1 }}</b>{{ step }}</span></div>
    <ModeSelector :modes="['base', kind]" />
    <TaskControls :kind="kind" />
    <el-alert v-if="error" :title="error" type="warning" :closable="false" />
    <div v-if="context" class="workspace-grid">
      <el-card shadow="never" class="mission-list">
        <template #header><strong>{{ kind === 'waypoint' ? '多点导航任务' : '覆盖作业任务' }}</strong></template>
        <el-button class="new-task" type="success" plain @click="selectedId = undefined">＋ {{ kind === 'waypoint' ? '新建多点导航任务' : '新建覆盖任务' }}</el-button>
        <div v-if="!missions.length" class="empty-list">{{ kind === 'waypoint' ? '尚无多点任务。填写目标点，或导入 JSON / CSV。' : '尚无覆盖任务。填写地块边界，或导入 JSON / GeoJSON。' }}</div>
        <div v-for="mission in missions" :key="mission.id" class="mission-row" :class="{ selected: selectedId === mission.id }">
          <button class="mission-select" @click="selectedId = mission.id"><strong>{{ mission.name }}</strong><small>v{{ mission.revision }} · {{ mission.mode === 'waypoint' ? `${mission.waypoints.length} 个目标点` : `${mission.preview.area?.toFixed(1)} m²` }}</small><small>{{ new Date(mission.updated_at).toLocaleString('zh-CN') }}</small></button>
          <el-button link type="danger" @click="remove(mission.id)">删除</el-button>
        </div>
      </el-card>
      <TaskEditor :key="kind" :mission="selected" :kind="kind" :defaults="context.defaults[kind]" @saved="saved" />
    </div>
    <el-empty v-else-if="!loading" description="任务服务未连接，请启动 FastAPI 后端" />
    <TaskRunHistory :kind="kind" />
  </div>
</template>
<style scoped>
.page-title { display: flex; justify-content: space-between; gap: 12px; align-items: center; margin-bottom: 20px; }.page-title h1 { font-size: 24px; margin: 0 0 9px; }.page-title p { color: #7e8d83; font-size: 13px; margin: 0; line-height: 1.8; }
.workflow { display: flex; flex-wrap: wrap; gap: 12px 24px; margin-bottom: 20px; color: #5f7567; font-size: 13px; }.workflow span { display: flex; align-items: center; gap: 8px; }.workflow b { display: grid; place-items: center; width: 23px; height: 23px; border-radius: 50%; background: #e4efe8; color: #28784c; }
.workspace-grid { display: grid; grid-template-columns: 260px minmax(0, 1fr); gap: 20px; margin-top: 20px; }.mission-list { align-self: start; }.new-task { width: 100%; margin: 16px 0; }.empty-list { padding: 20px 0; color: #829087; font-size: 13px; line-height: 1.8; }
.mission-row { display: flex; justify-content: space-between; gap: 8px; padding: 12px 8px; border-bottom: 1px solid #e6ece8; border-radius: 6px; }.mission-row.selected { background: #eaf4ec; }.mission-select { background: none; border: 0; text-align: left; cursor: pointer; color: #344c3b; overflow-wrap: anywhere; }.mission-select small { display: block; color: #839087; font-size: 11px; margin-top: 7px; }
@media (max-width: 1200px) { .workspace-grid { grid-template-columns: 1fr; }.mission-list :deep(.el-card__body) { max-height: 260px; overflow-y: auto; } }
</style>
