<script setup lang="ts">
import { ref } from 'vue'
import { Refresh } from '@element-plus/icons-vue'

import ModeSelector from '../components/ModeSelector.vue'
import WelcomeBanner from '../components/dashboard/WelcomeBanner.vue'
import StatisticsCards from '../components/dashboard/StatisticsCards.vue'
import RobotStatus from '../components/dashboard/RobotStatus.vue'
import LocationMap from '../components/dashboard/LocationMap.vue'
import TaskList from '../components/dashboard/TaskList.vue'
import SafetyStatus from '../components/dashboard/SafetyStatus.vue'
import CameraMonitor from '../components/dashboard/CameraMonitor.vue'

const robotStatusRef = ref<{
  refreshData: () => Promise<void>
} | null>(null)

const refreshing = ref(false)

async function refreshDashboard() {
  if (refreshing.value) return

  refreshing.value = true

  try {
    await robotStatusRef.value?.refreshData()
  } finally {
    refreshing.value = false
  }
}
</script>

<template>
  <div class="dashboard-page">
    <div class="page-heading">
      <div>
        <h1>系统总览</h1>
        <p>实时了解农业机器人运行状态与作业情况</p>
      </div>

      <el-button
        type="success"
        plain
        :loading="refreshing"
        @click="refreshDashboard"
      >
        <el-icon><Refresh /></el-icon>
        刷新数据
      </el-button>
    </div>

    <ModeSelector />
    <WelcomeBanner />

    <StatisticsCards />

    <SafetyStatus />

    <div class="quick-actions"><router-link to="/tasks">多点导航配置与执行 →</router-link><router-link to="/planning">全覆盖规划与跟踪 →</router-link><router-link to="/following">路径跟随与手柄示教 →</router-link><router-link to="/monitor">实时监控与人工遥控 →</router-link></div>

    <div class="dashboard-grid">
      <div class="dashboard-left">
        <LocationMap />
      </div>

      <div class="dashboard-right">
        <RobotStatus ref="robotStatusRef" />
      </div>
    </div>

    <div class="task-section">
      <TaskList />
    </div>
    <CameraMonitor />
  </div>
</template>

<style scoped>

.coordinate-section {
  min-width: 0;
  margin-bottom: 24px;
}
.quick-actions { display: flex; flex-wrap: wrap; gap: 18px; margin-bottom: 22px; }.quick-actions a { color: #27815a; font-size: 13px; text-decoration: none; }

.dashboard-page {
  width: 100%;
  min-width: 0;
  color: #263b30;
}

.page-heading {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 20px;
  margin-bottom: 24px;
}

.page-heading h1 {
  margin: 0;
  color: #263b30;
  font-size: 24px;
  font-weight: 600;
}

.page-heading p {
  margin: 9px 0 0;
  color: #89958d;
  font-size: 13px;
}

.page-heading .el-button {
  flex-shrink: 0;
}

.dashboard-grid {
  display: grid;
  grid-template-columns: minmax(0, 1.6fr) minmax(300px, 1fr);
  align-items: stretch;
  gap: 20px;
  margin-bottom: 24px;
}

.dashboard-left,
.dashboard-right {
  min-width: 0;
}

.dashboard-left {
  display: flex;
  flex-direction: column;
}

.dashboard-left > .location-card:not(.expanded) {
  flex: 1;
}

.task-section {
  min-width: 0;
}

@media (max-width: 1000px) {
  .dashboard-grid {
    grid-template-columns: minmax(0, 1fr);
  }
}

@media (max-width: 600px) {
  .page-heading {
    align-items: flex-start;
  }

  .page-heading h1 {
    font-size: 21px;
  }

  .page-heading p {
    line-height: 1.6;
  }

  .page-heading .el-button {
    padding: 9px 10px;
  }
}
</style>
