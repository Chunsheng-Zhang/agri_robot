<script setup lang="ts">
import {
  MapLocation,
  Clock,
  Odometer,
  DataLine,
} from '@element-plus/icons-vue'

interface StatisticItem {
  title: string
  value: string
  unit: string
  icon: typeof MapLocation
}

const statistics: StatisticItem[] = [
  {
    title: '今日作业面积',
    value: '12.6',
    unit: '亩',
    icon: MapLocation,
  },
  {
    title: '累计作业时长',
    value: '03:28',
    unit: 'h',
    icon: Clock,
  },
  {
    title: '平均作业速度',
    value: '4.2',
    unit: 'km/h',
    icon: Odometer,
  },
  {
    title: '路径覆盖率',
    value: '96.8',
    unit: '%',
    icon: DataLine,
  },
]
</script>

<template>
  <div class="statistics-grid">
    <el-card
      v-for="item in statistics"
      :key="item.title"
      class="stat-card"
      shadow="never"
    >
      <div class="stat-content">
        <div class="stat-info">
          <div class="stat-title">
            {{ item.title }}
          </div>

          <div class="stat-value">
            {{ item.value }}
            <span class="stat-unit">{{ item.unit }}</span>
          </div>
        </div>

        <div class="stat-icon">
          <el-icon :size="25">
            <component :is="item.icon" />
          </el-icon>
        </div>
      </div>
    </el-card>
  </div>
</template>

<style scoped>
.statistics-grid {
  display: grid;
  grid-template-columns: repeat(4, minmax(0, 1fr));
  gap: 18px;
  margin-bottom: 24px;
}

.stat-card {
  border: 1px solid #e8eee9;
  border-radius: 10px;
  background: #ffffff;
}

.stat-content {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 12px;
}

.stat-info {
  min-width: 0;
}

.stat-title {
  margin-bottom: 12px;
  color: #7b8980;
  font-size: 12px;
}

.stat-value {
  color: #263b30;
  font-size: 27px;
  font-weight: 600;
  line-height: 1.2;
}

.stat-unit {
  margin-left: 3px;
  color: #7b8980;
  font-size: 12px;
  font-weight: 400;
}

.stat-icon {
  display: flex;
  width: 48px;
  height: 48px;
  flex-shrink: 0;
  align-items: center;
  justify-content: center;
  border-radius: 12px;
  background: #e9f5ee;
  color: #25845c;
}

@media (max-width: 1000px) {
  .statistics-grid {
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 14px;
  }
}

@media (max-width: 480px) {
  .statistics-grid {
    grid-template-columns: 1fr;
  }

  .stat-value {
    font-size: 24px;
  }
}
</style>
