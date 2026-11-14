import { computed, onUnmounted, ref, watch } from 'vue'
import { getDiagnosticPage } from '../api/diagnostics'
import type { DiagnosticEntry, DiagnosticPage } from '../api/diagnostics'

export function useDiagnostics() {
  const active = ref<'logs' | 'serial' | null>(null), paused = ref(false)
  const entries = ref<DiagnosticEntry[]>([]), page = ref<DiagnosticPage | null>(null)
  const error = ref(''), skipped = ref(0), loading = ref(false)
  const fresh = ref(false)
  const visible = ref(typeof document === 'undefined' || !document.hidden)
  let timer: ReturnType<typeof setTimeout> | undefined
  let expiry: ReturnType<typeof setTimeout> | undefined
  let request: AbortController | undefined
  let generation = 0, after = 0, session = '', disposed = false, switching = false, clearedThrough = 0
  const running = computed(() => active.value !== null && !paused.value && visible.value)
  function stop() {
    generation++
    if (timer) clearTimeout(timer)
    if (expiry) clearTimeout(expiry)
    expiry = undefined; fresh.value = false
    timer = undefined; request?.abort(); request = undefined; loading.value = false
  }
  async function poll(expected: number) {
    const kind = active.value
    if (disposed || expected !== generation || !running.value || !kind) return
    const controller = new AbortController(); request = controller; loading.value = true
    const started = Date.now()
    const timeout = setTimeout(() => controller.abort(), 2000)
    try {
      const data = await getDiagnosticPage(kind, after, session, controller.signal)
      if (disposed || expected !== generation || !running.value) return
      if (session !== data.session || data.reset) clearedThrough = 0
      if (session !== data.session || data.reset || after === 0) entries.value = []
      session = data.session; after = data.cursor
      entries.value = [...entries.value, ...data.entries.filter(row => row.id > clearedThrough)].slice(-500)
      page.value = data; skipped.value += data.skipped; error.value = ''
      if (expiry) clearTimeout(expiry)
      const roundTrip = Math.max(0, Date.now() - started)
      let remaining = 1000 - roundTrip
      if (data.status?.observer_online)
        remaining = Math.min(remaining, 1000 - (data.status.status_age_seconds || 0) * 1000 - roundTrip)
      if (data.status?.rx_fresh)
        remaining = Math.min(remaining, 1000 - (data.status.observed_rx_age_seconds || 0) * 1000 - roundTrip)
      fresh.value = remaining > 0
      if (remaining > 0) expiry = setTimeout(() => { fresh.value = false }, remaining)
    } catch (value) {
      if (!disposed && expected === generation && running.value) {
        fresh.value = false
        error.value = controller.signal.aborted ? '诊断接口超过2秒未应答，当前数据未更新' : value instanceof Error ? value.message : '诊断数据未更新'
      }
    } finally {
      clearTimeout(timeout)
      if (expected === generation) {
        loading.value = false; request = undefined
        if (running.value) timer = setTimeout(() => { void poll(expected) }, 500)
      }
    }
  }
  function clearDisplay() { entries.value = []; skipped.value = 0; clearedThrough = after }
  function close() { active.value = null }
  function open(kind: 'logs' | 'serial') { active.value = active.value === kind ? null : kind }
  watch(active, () => {
    switching = true
    stop(); after = 0; session = ''; entries.value = []; page.value = null; skipped.value = 0
    error.value = ''; clearedThrough = 0; paused.value = false; switching = false
    if (running.value) void poll(generation)
  }, { flush: 'sync' })
  watch([paused, visible], () => {
    if (switching) return
    stop()
    if (running.value) { after = 0; void poll(generation) } // Resume at newest, without replaying a backlog.
  }, { flush: 'sync' })
  const visibility = () => { visible.value = !document.hidden }
  if (typeof document !== 'undefined') document.addEventListener('visibilitychange', visibility)
  onUnmounted(() => {
    disposed = true; stop()
    if (typeof document !== 'undefined') document.removeEventListener('visibilitychange', visibility)
  })
  return { active, paused, entries, page, error, skipped, loading, fresh, running, open, close, clearDisplay }
}
