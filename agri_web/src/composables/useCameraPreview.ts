import { computed, onBeforeUnmount, onMounted, onUnmounted, readonly, ref, watch } from 'vue'
import { getCameraStream } from '../api/robot'
import { useRealtime } from './useRealtime'

export const cameraPreview = Object.freeze({ width: 320, height: 240, quality: 40, interval: 500 })

export function useCameraPreview() {
  const { devices, isFresh } = useRealtime()
  const camera = computed(() => isFresh(devices.value?.camera) ? devices.value?.camera : null)
  const connected = computed(() => camera.value?.connected === true)
  const enabled = ref(false), visible = ref(document.visibilityState === 'visible')
  const imageUrl = ref(''), error = ref(''), loading = ref(false)
  const active = computed(() => enabled.value && visible.value && connected.value)
  let disposed = false, generation = 0, snapshotUrl = ''
  let controller: AbortController | null = null
  let timer: ReturnType<typeof setTimeout> | undefined

  function clearImage() {
    if (imageUrl.value) URL.revokeObjectURL(imageUrl.value)
    imageUrl.value = ''
  }
  function cancel() {
    generation += 1
    clearTimeout(timer); timer = undefined
    controller?.abort(); controller = null
    loading.value = false; snapshotUrl = ''; clearImage()
  }
  function snapshotAddress(stream: string) {
    const url = new URL(stream, window.location.origin)
    if (!['http:', 'https:'].includes(url.protocol) || !url.pathname.endsWith('/stream')) {
      throw new Error('当前视频服务没有可用的截图地址')
    }
    url.pathname = url.pathname.replace(/\/stream$/, '/snapshot')
    url.searchParams.delete('type')
    for (const name of ['width', 'height', 'quality'] as const) {
      url.searchParams.set(name, String(cameraPreview[name]))
    }
    return url.toString().replace(/%2F/gi, '/')
  }
  async function frame() {
    timer = undefined
    if (disposed || !active.value || controller) return
    const epoch = generation, request = new AbortController()
    controller = request; loading.value = true
    let deadline: ReturnType<typeof setTimeout> | undefined
    const current = () => !disposed && active.value && epoch === generation
    try {
      if (!snapshotUrl) {
        const info = await getCameraStream()
        if (!current()) return
        snapshotUrl = snapshotAddress(info.stream_url)
      }
      const url = new URL(snapshotUrl)
      url.searchParams.set('_frame', String(Date.now()))
      deadline = setTimeout(() => request.abort(), 1500)
      request.signal.addEventListener('abort', () => clearTimeout(deadline), { once: true })
      const response = await fetch(url.toString().replace(/%2F/gi, '/'), {
        signal: request.signal, cache: 'no-store',
      })
      if (!response.ok) throw new Error(`画面请求失败（HTTP ${response.status}）`)
      const blob = await response.blob()
      if (!current() || request.signal.aborted) return
      if (!blob.size || blob.type.split(';')[0] !== 'image/jpeg') throw new Error('相机未返回有效 JPEG 图片')
      const next = URL.createObjectURL(blob)
      clearImage(); imageUrl.value = next; error.value = ''
    } catch (reason) {
      if (current()) {
        clearImage()
        error.value = request.signal.aborted ? '画面请求超时，稍后重试'
          : reason instanceof Error ? reason.message : '画面连接失败，稍后重试'
      }
    } finally {
      clearTimeout(deadline)
      if (controller === request) { controller = null; loading.value = false }
      // Complete one image before scheduling the next: no overlap or backlog.
      if (current()) timer = setTimeout(() => void frame(), error.value ? 2000 : cameraPreview.interval)
    }
  }
  function toggle() { enabled.value = !enabled.value }
  function visibilityChanged() { visible.value = document.visibilityState === 'visible' }
  function imageError() {
    if (!imageUrl.value) return
    cancel(); error.value = '画面解码失败，稍后重试'
    if (active.value && !disposed) timer = setTimeout(() => void frame(), 2000)
  }
  watch(active, value => {
    cancel(); error.value = ''
    if (value && !disposed) void frame()
  }, { flush: 'sync' })
  onMounted(() => document.addEventListener('visibilitychange', visibilityChanged))
  onBeforeUnmount(() => { disposed = true; cancel() })
  onUnmounted(() => document.removeEventListener('visibilitychange', visibilityChanged))
  return { camera, connected, enabled: readonly(enabled), visible: readonly(visible),
    imageUrl: readonly(imageUrl), error: readonly(error), loading: readonly(loading), toggle, imageError }
}
