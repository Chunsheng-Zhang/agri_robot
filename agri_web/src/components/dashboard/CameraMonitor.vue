<script setup lang="ts">
import { computed, onMounted, onUnmounted, ref, watch } from 'vue'
import { getCameraStream } from '../../api/robot'
import { useRealtime } from '../../composables/useRealtime'

const { devices, isFresh } = useRealtime()
const camera = computed(() => isFresh(devices.value?.camera) ? devices.value?.camera : null)
const connected = computed(() => camera.value?.connected === true)
const stream = ref('')
const connection = ref(0)
const imageUrl = computed(() => stream.value ? `${stream.value}${stream.value.includes('?') ? '&' : '?'}_connect=${connection.value}` : '')
const failed = ref(false)
const streamError = ref('')
let poll: ReturnType<typeof setInterval> | undefined
let retry: ReturnType<typeof setTimeout> | undefined
let disposed = false
let pending = false
async function refreshStream() {
  if (pending) return
  pending = true
  try {
    const data = await getCameraStream()
    const url = new URL(data.stream_url, window.location.origin)
    if (!['http:', 'https:'].includes(url.protocol)) throw new Error('视频地址无效')
    if (!disposed) { stream.value = url.toString(); streamError.value = '' }
  } catch (error) {
    if (!disposed) { stream.value = ''; streamError.value = error instanceof Error ? error.message : '视频地址不可用' }
  } finally { pending = false }
}
function imageError() {
  failed.value = true
  clearTimeout(retry)
  retry = setTimeout(() => { if (!disposed && connected.value) { connection.value += 1; failed.value = false } }, 3000)
}
watch(connected, () => {
  clearTimeout(retry)
  failed.value = false
  if (connected.value) { connection.value += 1; void refreshStream() }
})
onMounted(() => { void refreshStream(); poll = setInterval(() => void refreshStream(), 10000) })
onUnmounted(() => { disposed = true; clearInterval(poll); clearTimeout(retry) })
</script>

<template>
  <el-card class="camera-panel" shadow="never">
    <template #header><div class="heading"><span>相机实时画面</span><el-tag :type="connected ? 'success' : 'info'">{{ connected ? '图像接收正常' : '相机未连接 / 状态过期' }}</el-tag></div></template>
    <div class="video-frame">
      <img v-if="connected && imageUrl && !failed" :src="imageUrl" alt="Astra Pro Plus 实时画面" @error="imageError" />
      <div v-else class="video-placeholder">{{ !connected ? (camera?.reason || '等待相机图像') : failed ? '视频连接失败，正在重试' : (streamError || '等待视频服务') }}</div>
    </div>
    <div class="camera-meta">{{ camera?.topic || '/camera/color/image_raw' }} · {{ camera?.width || '--' }} × {{ camera?.height || '--' }} · {{ camera?.encoding || '--' }}</div>
  </el-card>
</template>

<style scoped>
.camera-panel { border: 1px solid #e8eee9; border-radius: 10px; margin: 20px 0; }
.heading { display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 10px; }
.video-frame { min-height: 240px; background: #14251d; border-radius: 8px; display: grid; place-items: center; overflow: hidden; }
.video-frame img { width: 100%; max-height: 480px; object-fit: contain; }
.video-placeholder { color: #c0cdc5; padding: 40px 12px; text-align: center; }
.camera-meta { padding-top: 12px; color: #7b8980; font-size: 12px; overflow-wrap: anywhere; }
</style>
