<script setup lang="ts">
import { computed, ref } from 'vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import { useWorkspace } from '../../composables/useWorkspace'
import { deleteMission, modeLabels } from '../../api/workspace'
import type { MissionDocument, MissionMode } from '../../api/workspace'
import TaskEditor from './TaskEditor.vue'
import TaskControls from './TaskControls.vue'

const props = defineProps<{ kind?: MissionMode }>()
const { context, error, loading, refresh } = useWorkspace()
const kind = ref<MissionMode>(props.kind || 'waypoint'), selectedId = ref<string>()
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
function saved(value: MissionDocument) { selectedId.value = value.id; kind.value = value.mode }
</script>
<template>
  <div class="task-workspace">
    <div class="page-title"><div><h1>{{ props.kind ? '农田全覆盖规划' : '任务管理' }}</h1><p>配置、检查、保存，再明确开始任务。实时控制始终经过 ROS 安全模块。</p></div><el-button :loading="loading" @click="refresh">刷新任务</el-button></div>
    <TaskControls />
    <el-alert v-if="error" :title="error" type="warning" :closable="false" />
    <div v-if="context" class="workspace-grid">
      <el-card shadow="never" class="mission-list">
        <template #header><strong>已保存任务</strong></template>
        <el-radio-group v-if="!props.kind" v-model="kind" @change="selectedId = undefined"><el-radio-button value="waypoint">多点导航</el-radio-button><el-radio-button value="coverage">全覆盖</el-radio-button></el-radio-group>
        <el-button class="new-task" type="success" plain @click="selectedId = undefined">＋ 新建{{ modeLabels[kind] }}任务</el-button>
        <div v-if="!missions.length" class="empty-list">尚无任务。可以填写现场坐标，或导入 JSON / GeoJSON；多点任务也支持 CSV。</div>
        <div v-for="mission in missions" :key="mission.id" class="mission-row" :class="{ selected: selectedId === mission.id }">
          <button class="mission-select" @click="selectedId = mission.id"><strong>{{ mission.name }}</strong><small>v{{ mission.revision }} · {{ mission.mode === 'waypoint' ? `${mission.waypoints.length} 个目标点` : `${mission.preview.area?.toFixed(1)} m²` }}</small><small>{{ new Date(mission.updated_at).toLocaleString('zh-CN') }}</small></button>
          <el-button link type="danger" @click="remove(mission.id)">删除</el-button>
        </div>
      </el-card>
      <TaskEditor :mission="selected" :kind="kind" :defaults="context.defaults[kind]" @saved="saved" />
    </div>
    <el-empty v-else-if="!loading" description="任务服务未连接，请启动 FastAPI 后端" />
  </div>
</template>
<style scoped>
.page-title { display: flex; justify-content: space-between; gap: 12px; align-items: center; margin-bottom: 20px; }.page-title h1 { font-size: 24px; margin: 0 0 9px; }.page-title p { color: #7e8d83; font-size: 13px; margin: 0; line-height: 1.8; }
.workspace-grid { display: grid; grid-template-columns: 260px minmax(0, 1fr); gap: 20px; margin-top: 20px; }.mission-list { align-self: start; }.new-task { width: 100%; margin: 16px 0; }.empty-list { padding: 20px 0; color: #829087; font-size: 13px; line-height: 1.8; }
.mission-row { display: flex; justify-content: space-between; gap: 8px; padding: 12px 8px; border-bottom: 1px solid #e6ece8; border-radius: 6px; }.mission-row.selected { background: #eaf4ec; }.mission-select { background: none; border: 0; text-align: left; cursor: pointer; color: #344c3b; overflow-wrap: anywhere; }.mission-select small { display: block; color: #839087; font-size: 11px; margin-top: 7px; }
@media (max-width: 1200px) { .workspace-grid { grid-template-columns: 1fr; }.mission-list :deep(.el-card__body) { max-height: 260px; overflow-y: auto; } }
</style>
