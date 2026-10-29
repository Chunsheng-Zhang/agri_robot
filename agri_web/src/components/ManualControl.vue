<script setup lang="ts">
import { computed, onMounted, onUnmounted, ref, watch } from 'vue'
import { ElMessage } from 'element-plus'
import { useRealtime } from '../composables/useRealtime'
import { armManual, manualCommand, stopManual } from '../api/workspace'
import { apiUrl } from '../api/http'

const { devices, safetyFresh } = useRealtime()
const token = ref<string | null>(null), busy = ref(false), linear = ref(0.15), angular = ref(0.3)
const direction = ref({ v: 0, w: 0 }), limits = ref({ linear: 0.3, angular: 0.6 })
const eligible = computed(() => safetyFresh.value && devices.value?.safety.mode === 'base'
  && devices.value.safety.motion_inhibited === false)
let timer: ReturnType<typeof setInterval> | undefined, seq = 0, offset = 0, pending = false
const keys = new Set<string>()
async function heartbeat(force = false) {
  const session = token.value
  if (!session || (pending && !force)) return
  pending = true
  const sequence = ++seq
  try {
    await manualCommand({ token: session, seq: sequence, linear: direction.value.v * linear.value,
      angular: direction.value.w * angular.value, expires_at: Date.now() / 1000 + offset + 0.2 })
  } catch (value) {
    if (token.value === session && sequence === seq) {
      stop(); ElMessage.error(value instanceof Error ? value.message : '遥控通信异常，已停止')
    }
  } finally { pending = false }
}
async function arm() {
  busy.value = true
  try {
    const started = Date.now() / 1000, response = await armManual(), ended = Date.now() / 1000
    if (ended - started > 0.3) { void stopManual(response.token); throw new Error('遥控响应延迟过高，请检查局域网或使用手柄') }
    token.value = response.token; seq = 0; offset = response.server_time - (started + ended) / 2
    limits.value = response.limits
    linear.value = Math.min(linear.value, response.limits.linear); angular.value = Math.min(angular.value, response.limits.angular)
    timer = setInterval(() => void heartbeat(), 80)
  } catch (value) { ElMessage.error(value instanceof Error ? value.message : '无法取得遥控权限') }
  finally { busy.value = false }
}
function neutral() { direction.value = { v: 0, w: 0 }; void heartbeat(true) }
function stop(beacon = false) {
  const session = token.value
  token.value = null; clearInterval(timer); keys.clear(); direction.value = { v: 0, w: 0 }
  if (session) {
    if (beacon) navigator.sendBeacon(apiUrl('/api/control/stop'), new Blob([JSON.stringify({ token: session })], { type: 'application/json' }))
    else void stopManual(session).catch(() => {})
  }
}
function hold(event: PointerEvent, v: number, w: number) {
  if (!token.value) return
  (event.currentTarget as HTMLElement).setPointerCapture(event.pointerId)
  direction.value = { v, w }; void heartbeat(true)
}
function updateKeys() {
  if (!keys.has('Space')) return neutral()
  direction.value = { v: Number(keys.has('KeyW')) - Number(keys.has('KeyS')),
    w: Number(keys.has('KeyA')) - Number(keys.has('KeyD')) }
  void heartbeat(true)
}
function keyDown(event: KeyboardEvent) {
  if (!token.value || (event.target as HTMLElement).closest?.('input,textarea,select,[contenteditable=true]')) return
  if (!['Space', 'KeyW', 'KeyA', 'KeyS', 'KeyD'].includes(event.code)) return
  event.preventDefault(); if (!event.repeat) { keys.add(event.code); updateKeys() }
}
function keyUp(event: KeyboardEvent) { if (keys.delete(event.code)) updateKeys() }
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
