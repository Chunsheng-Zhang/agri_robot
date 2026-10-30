<script setup lang="ts">
import { computed } from 'vue'
import { useRealtime, numberText } from '../composables/useRealtime'
import SafetyStatus from '../components/dashboard/SafetyStatus.vue'
import LidarMonitor from '../components/LidarMonitor.vue'
import CameraMonitor from '../components/dashboard/CameraMonitor.vue'
const { devices, robot, robotFresh, isFresh } = useRealtime()
const rows = computed(() => [
  { name: 'ROS 桥接', online: robotFresh.value && robot.value?.ros_connected, detail: robot.value?.ros_error || 'FastAPI 与 ROS 2 通信节点', age: null },
  { name: 'MGX23 GNSS', online: isFresh(devices.value?.gnss), detail: '经纬度输入；定位有效不等于已确认 RTK 固定解', age: devices.value?.gnss.age_seconds },
  { name: 'Jetson 手柄适配', online: isFresh(devices.value?.manual) && devices.value?.manual.connected, detail: `映射 ${devices.value?.manual.profile || '--'}，线速度档位 ${devices.value?.manual.linear_gear ?? '--'}`, age: devices.value?.manual.age_seconds },
  { name: '4ROS 激光雷达', online: isFresh(devices.value?.obstacles) && devices.value?.obstacles.valid, detail: '方向净空、距离检测与安全停车', age: devices.value?.obstacles.age_seconds },
  { name: 'Astra Pro Plus', online: isFresh(devices.value?.camera) && devices.value?.camera.connected, detail: `${devices.value?.camera.width || '--'} × ${devices.value?.camera.height || '--'} · ${devices.value?.camera.encoding || '--'}`, age: devices.value?.camera.age_seconds },
  { name: '底盘电压反馈', online: isFresh(devices.value?.voltage, 3000), detail: `${numberText(isFresh(devices.value?.voltage, 3000) ? devices.value?.voltage?.volts : null)} V；不换算未知电池的百分比`, age: devices.value?.voltage?.age_seconds },
])
</script>
<template><div><h1>设备与安全</h1><SafetyStatus /><el-card shadow="never"><el-table :data="rows"><el-table-column prop="name" label="设备 / 通道" min-width="150" /><el-table-column label="数据状态" width="170"><template #default="{ row }"><el-tag :type="row.online ? 'success' : 'warning'">{{ row.online ? '数据在线' : '未连接 / 数据过期' }}</el-tag></template></el-table-column><el-table-column prop="detail" label="信息" min-width="280" /><el-table-column label="接收年龄" width="130"><template #default="{ row }">{{ numberText(row.age, 2) }} s</template></el-table-column></el-table></el-card><p class="hint">设备驱动由 ROS 核心统一启动。页面显示实际接收状态，不重复启动驱动；相机 Topic 在线与浏览器视频可用分别判断。</p><div class="devices-grid"><LidarMonitor /><CameraMonitor /></div></div></template>
<style scoped>h1 { font-size: 24px; margin: 0 0 22px; }.hint { color: #7c8c81; line-height: 1.8; font-size: 13px; }.devices-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 20px; margin-top: 22px; }@media (max-width: 950px) { .devices-grid { grid-template-columns: 1fr; } }</style>
