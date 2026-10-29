import { onMounted, onUnmounted, readonly, shallowRef, ref } from 'vue'
import { getWorkspace } from '../api/workspace'
import type { WorkspaceContext } from '../api/workspace'

const context = shallowRef<WorkspaceContext | null>(null)
const error = ref('')
const loading = ref(false)
let consumers = 0, timer: ReturnType<typeof setInterval> | undefined
async function refresh() {
  if (loading.value) return
  loading.value = true
  try { context.value = await getWorkspace(); error.value = '' }
  catch (value) { context.value = null; error.value = value instanceof Error ? value.message : '任务服务不可用' }
  finally { loading.value = false }
}
export function useWorkspace() {
  onMounted(() => { if (++consumers === 1) { void refresh(); timer = setInterval(() => void refresh(), 2000) } })
  onUnmounted(() => { if (--consumers === 0) { clearInterval(timer); context.value = null } })
  return { context: readonly(context), error: readonly(error), loading: readonly(loading), refresh }
}
