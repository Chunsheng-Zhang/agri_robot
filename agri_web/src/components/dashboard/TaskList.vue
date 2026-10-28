<script setup lang="ts">
import { computed } from 'vue'
import { useRealtime, taskText, numberText } from '../../composables/useRealtime'
import type { TaskSnapshot } from '../../api/robot'

const { devices, isFresh, safetyFresh } = useRealtime()
const rows = computed(() => {
  const entries: [string, string, TaskSnapshot | undefined][] = [
    ['waypoint', '多目标点导航', devices.value?.waypoint],
    ['tracking', 'Pure Pursuit 跟踪', devices.value?.tracking],
    ['planning', '全覆盖路径规划', devices.value?.planning],
  ]
  return entries.map(([key, name, task]) => {
    const fresh = isFresh(task)
    let progress = '--'
    if (fresh && task && key === 'waypoint' && task.count) {
      progress = task.state === 'COMPLETED' ? '全部目标点已完成'
        : `目标 ${Math.min((task.index ?? 0) + 1, task.count)} / ${task.count}`
    } else if (fresh && task && key === 'tracking' && task.length) {
      progress = `${numberText(task.progress, 1)} / ${numberText(task.length, 1)} m`
    }
    return { key, name, state: fresh ? taskText(task?.state) : '节点未更新',
      active: safetyFresh.value && devices.value?.safety.owner === key,
      progress, reason: fresh ? (task?.reason || '—') : '等待实时状态' }
  })
})
</script>

<template>
  <el-card class="panel-card" shadow="never">
    <template #header><div class="task-heading"><span>实时任务状态</span><el-tag type="info">ROS 2 状态反馈</el-tag></div></template>
    <el-table :data="rows" row-key="key">
      <el-table-column prop="name" label="功能" min-width="170" />
      <el-table-column label="状态" min-width="140">
        <template #default="{ row }"><el-tag :type="row.active ? 'success' : 'info'">{{ row.state }}</el-tag></template>
      </el-table-column>
      <el-table-column prop="progress" label="任务进度" min-width="180" />
      <el-table-column label="控制权" width="110">
        <template #default="{ row }">{{ row.active ? '已取得' : '未取得' }}</template>
      </el-table-column>
      <el-table-column prop="reason" label="原因 / 说明" min-width="220" show-overflow-tooltip />
    </el-table>
  </el-card>
</template>

<style scoped>
.task-heading { display: flex; align-items: center; justify-content: space-between; gap: 12px; }
.panel-card { border: 1px solid #e8eee9; border-radius: 10px; }
</style>

