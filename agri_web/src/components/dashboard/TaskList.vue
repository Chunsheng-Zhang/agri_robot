<script setup lang="ts">
import { ref } from 'vue'
import { ElMessage } from 'element-plus'

interface Task {
  id: string
  name: string
  area: string
  progress: number
  status: '进行中' | '待执行' | '已完成'
}

const tasks = ref<Task[]>([
  {
    id: 'TASK-001',
    name: '东侧地块耕整作业',
    area: '8.5 亩',
    progress: 76,
    status: '进行中',
  },
  {
    id: 'TASK-002',
    name: '西侧地块覆盖作业',
    area: '6.2 亩',
    progress: 0,
    status: '待执行',
  },
  {
    id: 'TASK-003',
    name: '北侧地块路径测试',
    area: '2.8 亩',
    progress: 100,
    status: '已完成',
  },
])

function statusType(status: Task['status']) {
  const types: Record<Task['status'], 'success' | 'warning' | 'info'> = {
    进行中: 'success',
    待执行: 'warning',
    已完成: 'info',
  }

  return types[status]
}

function viewTask(task: Task) {
  ElMessage.info(`任务 ${task.id} 的详情功能将在后续开发`)
}
</script>

<template>
  <el-card class="panel-card task-card" shadow="never">
    <template #header>
      <div class="panel-header">
        <span>作业任务</span>

        <el-tag type="info" effect="light" size="small">
          {{ tasks.length }} 项任务
        </el-tag>
      </div>
    </template>

    <el-table
      :data="tasks"
      row-key="id"
      style="width: 100%"
    >
      <el-table-column
        prop="id"
        label="任务编号"
        min-width="120"
      />

      <el-table-column
        prop="name"
        label="任务名称"
        min-width="170"
        show-overflow-tooltip
      />

      <el-table-column
        prop="area"
        label="作业面积"
        width="100"
      />

      <el-table-column
        label="作业进度"
        min-width="150"
      >
        <template #default="{ row }">
          <div class="progress-cell">
            <el-progress
              :percentage="row.progress"
              :stroke-width="7"
              :show-text="false"
              :status="row.progress === 100 ? 'success' : undefined"
            />

            <span>{{ row.progress }}%</span>
          </div>
        </template>
      </el-table-column>

      <el-table-column
        label="状态"
        width="100"
      >
        <template #default="{ row }">
          <el-tag
            :type="statusType(row.status)"
            effect="light"
            size="small"
          >
            {{ row.status }}
          </el-tag>
        </template>
      </el-table-column>

      <el-table-column
        label="操作"
        width="90"
        fixed="right"
      >
        <template #default="{ row }">
          <el-button
            type="success"
            link
            @click="viewTask(row)"
          >
            查看
          </el-button>
        </template>
      </el-table-column>
    </el-table>
  </el-card>
</template>

<style scoped>
.panel-card {
  border-radius: 10px;
  border-color: #e5eae7;
}

.panel-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 12px;
  color: #263b30;
  font-size: 15px;
  font-weight: 600;
}

.progress-cell {
  display: flex;
  align-items: center;
  gap: 10px;
}

.progress-cell .el-progress {
  flex: 1;
  min-width: 40px;
}

.progress-cell span {
  width: 35px;
  flex-shrink: 0;
  color: #65736a;
  font-size: 12px;
  text-align: right;
}
</style>

