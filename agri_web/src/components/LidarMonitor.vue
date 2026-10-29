<script setup lang="ts">
import { computed } from 'vue'
import { useRealtime, numberText } from '../composables/useRealtime'
const { devices, isFresh } = useRealtime()
const scan = computed(() => isFresh(devices.value?.scan) ? devices.value?.scan : null)
const radius = computed(() => Math.min(8, Math.max(2, scan.value?.range_max || 5)))
const points = computed(() => (scan.value?.points || []).filter(p => Math.hypot(p.x, p.y) <= radius.value)
  .map(p => ({ x: 200 + p.y * -170 / radius.value, y: 200 - p.x * 170 / radius.value })))
</script>
<template>
  <el-card shadow="never" class="lidar-panel">
    <template #header><div class="heading"><strong>雷达实时扫描</strong><el-tag :type="scan ? 'success' : 'info'">{{ scan ? `${scan.points.length} 个有效采样点` : '未连接 / 数据过期' }}</el-tag></div></template>
    <svg viewBox="0 0 400 400" role="img" aria-label="雷达扫描预览">
      <rect width="400" height="400" fill="#14271e" />
      <circle v-for="r in [42.5, 85, 127.5, 170]" :key="r" cx="200" cy="200" :r="r" fill="none" stroke="#345343" />
      <path d="M 200 25 V 375 M 25 200 H 375" stroke="#345343" />
      <circle v-for="(point, index) in points" :key="index" :cx="point.x" :cy="point.y" r="1.6" fill="#85e3ac" />
      <path d="M 200 186 L 193 210 L 207 210 Z" fill="#e8b359" />
      <text x="15" y="23" fill="#b1d2bd" font-size="12">{{ scan?.frame_id || 'laser' }} · X ↑ · {{ numberText(radius, 0) }} m</text>
    </svg>
    <p>显示雷达自身坐标系的有效点；车体净空和停车依据以 ROS 安全模块为准。</p>
  </el-card>
</template>
<style scoped>.heading { display: flex; justify-content: space-between; flex-wrap: wrap; gap: 8px; }.lidar-panel svg { width: 100%; max-height: 400px; border-radius: 8px; }.lidar-panel p { color: #819087; font-size: 12px; line-height: 1.7; }</style>
