<script setup lang="ts">
import { computed, onMounted, onUnmounted, ref, watch } from 'vue'
import { ElMessage } from 'element-plus'
import { useRealtime } from '../composables/useRealtime'
import { armManual, stopManual } from '../api/workspace'
import { apiUrl } from '../api/http'
import { ManualChannel, manualTiming } from '../control/manualChannel'

const { devices, safetyFresh } = useRealtime()
const token = ref<string | null>(null), busy = ref(false), linear = ref(0.15), angular = ref(0.3)
const direction = ref({ v: 0, w: 0 }), limits = ref({ linear: 0.3, angular: 0.6 })
const eligible = computed(() => safetyFresh.value && devices.value?.safety.mode === 'base'
  && devices.value.safety.motion_inhibited === false)
let timer: ReturnType<typeof setInterval> | undefined, seq = 0
let channel: ManualChannel | null = null, openingChannel: ManualChannel | null = null
let clockAnchor = 0, monotonicAnchor = 0
const expiry = () => clockAnchor + (performance.now() - monotonicAnchor) / 1000 + manualTiming.lifetime
let lastSentAt = 0
let armGeneration = 0
const keys = new Set<string>()
async function heartbeat() {
  const session = token.value, connection = channel
  if (!session || !connection) return
  const now = performance.now()
  if (now - lastSentAt > manualTiming.maxGap) {
    stop(); ElMessage.error('遥控心跳调度中断，已退出遥控，请重新启用'); return
  }
  lastSentAt = now
  const sequence = ++seq
  try {
    await connection.send({ token: session, seq: sequence, linear: direction.value.v * linear.value,
      angular: direction.value.w * angular.value, expires_at: expiry() })
  } catch (value) {
    if (token.value === session && channel === connection) {
      stop(); ElMessage.error(value instanceof Error ? value.message : '遥控通信异常，已停止')
    }
  }
}
async function arm() {
  const generation = ++armGeneration
  let candidate: string | null = null
  busy.value = true
  let connection: ManualChannel | null = null
  try {
    connection = new ManualChannel(apiUrl('/api/control/channel'))
    openingChannel = connection
    await connection.open()
    if (generation !== armGeneration || !eligible.value || document.visibilityState !== 'visible') return
    const started = performance.now(), response = await armManual(), ended = performance.now()
    candidate = response.token
    if (generation !== armGeneration || !eligible.value || document.visibilityState !== 'visible') return
    if (response.heartbeat_protocol !== 4) throw new Error('后端遥控协议尚未更新，请先加载新版后端')
    if (ended - started > 300) throw new Error('遥控响应延迟过高，请检查局域网或使用手柄')
    clockAnchor = response.server_time
    monotonicAnchor = (started + ended) / 2
    // Verify the expiring command channel with zero velocity before enabling
    // direction controls. Never retry arm or a movement command automatically.
    await connection.send({ token: candidate, seq: 1, linear: 0, angular: 0,
      expires_at: expiry() })
    if (generation !== armGeneration || !eligible.value || document.visibilityState !== 'visible') return
    channel = connection; openingChannel = null
    const activeConnection = connection
    connection.onFailure = value => {
      if (channel === activeConnection) { stop(); ElMessage.error(value.message) }
    }
    token.value = candidate; candidate = null; seq = 1
    lastSentAt = performance.now()
    // The Web API caps commands at 1 m/s and 2 rad/s, independently of the
    // physical joystick's factory limits. Do not offer values it will reject.
    limits.value = { linear: Math.min(response.limits.linear, 1), angular: Math.min(response.limits.angular, 2) }
    linear.value = Math.min(linear.value, limits.value.linear); angular.value = Math.min(angular.value, limits.value.angular)
    timer = setInterval(() => void heartbeat(), manualTiming.heartbeat)
  } catch (value) {
    if (generation === armGeneration) ElMessage.error(value instanceof Error ? value.message : '无法取得遥控权限')
  } finally {
    if (candidate) void stopManual(candidate).catch(() => {})
    if (connection !== channel) connection?.close()
    if (openingChannel === connection) openingChannel = null
    if (generation === armGeneration) busy.value = false
  }
}
function requestDirection(v: number, w: number) {
  direction.value = { v, w }; void heartbeat()
}
function neutral() { direction.value = { v: 0, w: 0 }; void heartbeat() }
function stop(beacon = false) {
  armGeneration += 1  // Late arm/probe responses must never re-enable controls.
  const session = token.value
  token.value = null; clearInterval(timer); keys.clear(); direction.value = { v: 0, w: 0 }
  channel?.close(); channel = null
  openingChannel?.close(); openingChannel = null
  busy.value = false
  if (session) {
    if (beacon) navigator.sendBeacon(apiUrl('/api/control/stop'), new Blob([JSON.stringify({ token: session })], { type: 'application/json' }))
    else void stopManual(session).catch(() => {})
  }
}
function hold(event: PointerEvent, v: number, w: number) {
  if (!token.value) return
  (event.currentTarget as HTMLElement).setPointerCapture(event.pointerId)
  requestDirection(v, w)
}
function updateKeys() {
  if (!keys.has('Space')) return neutral()
  requestDirection(Number(keys.has('KeyW')) - Number(keys.has('KeyS')),
    Number(keys.has('KeyA')) - Number(keys.has('KeyD')))
}
function keyDown(event: KeyboardEvent) {
  if (!token.value || (event.target as HTMLElement).closest?.('input,textarea,select,[contenteditable=true]')) return
  if (!['Space', 'KeyW', 'KeyA', 'KeyS', 'KeyD'].includes(event.code)) return
  event.preventDefault(); if (!event.repeat) { keys.add(event.code); updateKeys() }
}
function keyUp(event: KeyboardEvent) {
  if (!['Space', 'KeyW', 'KeyA', 'KeyS', 'KeyD'].includes(event.code)) return
  if (keys.delete(event.code)) updateKeys()
}
function hidden() { if (document.visibilityState !== 'visible') stop(true) }
function blurred() { stop() }
function unloading() { stop(true) }
watch(eligible, valid => { if (!valid) stop() })
onMounted(() => {
  window.addEventListener('keydown', keyDown); window.addEventListener('keyup', keyUp)
  window.addEventListener('blur', blurred); window.addEventListener('pagehide', unloading)
  document.addEventListener('visibilitychange', hidden)
})
onUnmounted(() => {
  stop(); window.removeEventListener('keydown', keyDown); window.removeEventListener('keyup', keyUp)
  window.removeEventListener('blur', blurred); window.removeEventListener('pagehide', unloading)
  document.removeEventListener('visibilitychange', hidden)
})
</script>
<template>
  <el-card shadow="never" class="manual-panel">
    <template #header><div class="heading"><strong>人工遥控</strong><el-tag :type="token ? 'warning' : 'info'">{{ token ? 'Web 遥控已启用' : '未取得控制权' }}</el-tag></div></template>
    <p class="hint">先选择基础待命并解除安全锁定，再启用遥控。按住方向按钮行驶，松开停车；键盘需按住空格，再使用 W/A/S/D。离开页面、失焦或通信超时会停止。</p>
    <div class="manual-grid">
      <div class="direction-pad">
        <el-button class="forward" :disabled="!token" @pointerdown.prevent="hold($event, 1, 0)" @pointerup="neutral" @pointercancel="neutral" @pointerleave="neutral">↑ 前进</el-button>
        <el-button class="left" :disabled="!token" @pointerdown.prevent="hold($event, 0, 1)" @pointerup="neutral" @pointercancel="neutral" @pointerleave="neutral">↶ 左转</el-button>
        <el-button class="stop" type="danger" plain :disabled="!token" @click="stop()">停车并释放</el-button>
        <el-button class="right" :disabled="!token" @pointerdown.prevent="hold($event, 0, -1)" @pointerup="neutral" @pointercancel="neutral" @pointerleave="neutral">右转 ↷</el-button>
        <el-button class="back" :disabled="!token" @pointerdown.prevent="hold($event, -1, 0)" @pointerup="neutral" @pointercancel="neutral" @pointerleave="neutral">↓ 后退</el-button>
      </div>
      <el-form label-position="top">
        <el-form-item label="遥控线速度（m/s）"><el-input-number v-model="linear" :min="0.01" :max="limits.linear" :step="0.05" :precision="2" /></el-form-item>
        <el-form-item label="遥控角速度（rad/s）"><el-input-number v-model="angular" :min="0.01" :max="limits.angular" :step="0.05" :precision="2" /></el-form-item>
        <el-button type="success" :loading="busy" :disabled="!!token || !eligible" @click="arm">启用 Web 遥控</el-button>
      </el-form>
    </div>
  </el-card>
</template>
<style scoped>
.manual-panel { margin: 20px 0; }.heading { display: flex; justify-content: space-between; gap: 10px; }.hint { color: #7a8980; line-height: 1.8; font-size: 13px; }
.manual-grid { display: grid; grid-template-columns: minmax(240px, 1fr) 240px; gap: 24px; padding: 15px 0; }.direction-pad { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); grid-template-rows: repeat(3, 55px); gap: 8px; max-width: 450px; align-self: center; }.direction-pad .el-button { margin: 0; height: 100%; touch-action: none; }.forward { grid-column: 2; grid-row: 1; }.left { grid-column: 1; grid-row: 2; }.stop { grid-column: 2; grid-row: 2; }.right { grid-column: 3; grid-row: 2; }.back { grid-column: 2; grid-row: 3; }
.direction-pad .el-button { padding: 0 4px; font-size: 12px; }
@media (max-width: 750px) { .manual-grid { grid-template-columns: 1fr; }.direction-pad { width: 100%; } }
</style>
