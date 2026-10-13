<script setup lang="ts">
import { useRoute } from 'vue-router'
import { computed } from 'vue'

const route = useRoute()

const title = computed(() => String(route.meta.title ?? '功能页面'))

const descriptions: Record<string, string> = {
  monitor: '查看农机实时位置、运行状态和农田作业轨迹。',
  tasks: '创建和管理农机作业任务，查看任务执行进度。',
  planning: '配置农田边界、覆盖路径和路径规划参数。',
  devices: '查看农机设备、GNSS 定位和底盘通信状态。',
  collaboration: '规划多台农业机器人的任务分配与协同作业。',
  settings: '管理平台配置和系统参数。',
}

const description = computed(
  () => descriptions[String(route.name)] ?? '此页面将在后续开发阶段逐步完善。',
)
</script>

<template>
  <div class="placeholder-page">
    <h1>{{ title }}</h1>
    <p>{{ description }}</p>
    <el-tag type="info" effect="plain">开发中</el-tag>
  </div>
</template>

<style scoped>
.placeholder-page {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  min-height: 420px;
  padding: 40px;
  border: 1px solid #e9edf3;
  border-radius: 12px;
  background: white;
  text-align: center;
}

h1 {
  margin-bottom: 12px;
  color: #29394c;
  font-size: 24px;
}

p {
  margin-bottom: 22px;
  color: #8290a2;
  font-size: 14px;
  line-height: 1.8;
}
</style>
