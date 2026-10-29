<script setup lang="ts">
import FieldMap from '../components/dashboard/FieldMap.vue'
import RobotStatus from '../components/dashboard/RobotStatus.vue'
import SafetyStatus from '../components/dashboard/SafetyStatus.vue'
import CameraMonitor from '../components/dashboard/CameraMonitor.vue'
import LidarMonitor from '../components/LidarMonitor.vue'
import ManualControl from '../components/ManualControl.vue'
import TaskControls from '../components/tasks/TaskControls.vue'
</script>
<template><div><h1>作业监控</h1><TaskControls /><SafetyStatus /><div class="monitor-grid"><FieldMap /><RobotStatus /></div><ManualControl /><div class="monitor-grid"><CameraMonitor /><LidarMonitor /></div></div></template>
<style scoped>h1 { font-size: 24px; margin: 0 0 22px; }.monitor-grid { display: grid; grid-template-columns: minmax(0, 1.5fr) minmax(280px, 1fr); gap: 20px; align-items: start; margin: 20px 0; }@media (max-width: 1050px) { .monitor-grid { grid-template-columns: 1fr; } }</style>
