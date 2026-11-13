import { computed, nextTick, onMounted, onUnmounted, ref, shallowRef, watch } from 'vue'
import { coverageMapImageUrl, getCoverageMap } from '../api/workspace'
import type { CoverageMap, CoverageMapLayer, CoverageMapSource, Run } from '../api/workspace'

const names: CoverageMapLayer[] = ['completed', 'remaining', 'overlap']
export function validateCoverageMap(value: CoverageMap, run: Run) {
  if (value.run_id !== run.id) throw new Error('作业区域不属于当前任务')
  if (value.state !== 'READY') return value
  if (value.frame_id !== 'map' || value.reference_fingerprint !== run.reference_fingerprint
    || !run.projection || !value.projection
    || ['epsg', 'latitude', 'longitude'].some(key =>
      value.projection?.[key as keyof typeof value.projection] !== run.projection?.[key as keyof typeof run.projection])) {
    throw new Error('作业区域与本轮路径或固定原点不一致')
  }
  if (!value.extent || value.extent.length !== 4 || !value.extent.every(Number.isFinite)
    || value.extent[2]! <= value.extent[0]! || value.extent[3]! <= value.extent[1]!) throw new Error('作业区域范围无效')
  for (const source of ['planned', 'actual'] as const) {
    if (!/^[0-9a-f]{20}$/.test(value.sources?.[source]?.version || '')) throw new Error('作业区域版本无效')
  }
  const points = [...(value.inset_polygons || []).flat(2), ...(value.field_rings || []).flat(), ...(value.sources?.actual.track_lines || []).flat()]
  if (!points.every(p => Number.isFinite(p.x) && Number.isFinite(p.y))) throw new Error('作业区域含无效坐标')
  return value
}

interface Raster {
  run: string; source: CoverageMapSource; version: string; extent: number[]
  images: Record<CoverageMapLayer, string>
}
export function useCoverageLayers(getRun: () => Run | null | undefined, getSource: () => CoverageMapSource) {
  const data = shallowRef<CoverageMap | null>(null), raster = shallowRef<Raster | null>(null)
  const error = ref(''), loading = ref(false), imagesLoading = ref(false)
  const clock = ref(Date.now()), receivedAt = ref(0)
  let mounted = false, disposed = false, generation = 0, imageGeneration = 0
  let timer: ReturnType<typeof setInterval> | undefined
  let imageAbort: AbortController | undefined
  const identity = computed(() => {
    const run = getRun()
    return run ? JSON.stringify([run.id, run.reference_fingerprint, run.projection]) : ''
  })
  const snapshot = computed(() => data.value?.run_id === getRun()?.id ? data.value : null)
  const visibleRaster = computed(() => {
    const value = raster.value
    return value && value.run === getRun()?.id && value.source === getSource() ? value : null
  })
  const stale = computed(() => snapshot.value !== null
    && (clock.value - receivedAt.value > 6000 || (snapshot.value.snapshot_age_seconds || 0) > 6))
  function release(value: Raster | null) { if (value) Object.values(value.images).forEach(url => URL.revokeObjectURL(url)) }
  function clear() {
    generation++; imageGeneration++; imageAbort?.abort()
    imagesLoading.value = false
    release(raster.value); raster.value = null; data.value = null; error.value = ''
  }
  async function loadImages(value: CoverageMap) {
    const source = getSource(), stage = value.sources?.[source]
    if (!stage || !value.extent || value.state !== 'READY') return
    if (visibleRaster.value?.version === stage.version || imagesLoading.value) return
    const epoch = ++imageGeneration, currentIdentity = identity.value
    imagesLoading.value = true
    const controller = new AbortController(); imageAbort = controller
    const timeout = setTimeout(() => controller.abort(), 4000)
    let next: Raster | null = null
    try {
      const blobs = await Promise.all(names.map(async name => {
        const response = await fetch(coverageMapImageUrl(value.run_id, source, name, stage.version),
          { signal: controller.signal, cache: 'force-cache' })
        if (!response.ok) throw new Error(`作业区域图层读取失败（HTTP ${response.status}）`)
        return response.blob()
      }))
      next = { run: value.run_id, source, version: stage.version, extent: value.extent,
        images: Object.fromEntries(names.map((name, i) => [name, URL.createObjectURL(blobs[i]!)])) as Record<CoverageMapLayer, string> }
      if (disposed || epoch !== imageGeneration || currentIdentity !== identity.value || source !== getSource()) { release(next); return }
      const previous = raster.value
      raster.value = next; error.value = ''
      await nextTick(); release(previous)
    } catch (reason) {
      controller.abort()
      if (!disposed && epoch === imageGeneration && currentIdentity === identity.value)
        error.value = reason instanceof Error ? reason.message : '作业区域图层读取失败'
    } finally { clearTimeout(timeout); if (epoch === imageGeneration) imagesLoading.value = false }
  }
  async function refresh() {
    const run = getRun()
    clock.value = Date.now()
    if (!mounted || disposed || !run || loading.value || document.visibilityState === 'hidden') return
    const epoch = generation, requested = Date.now()
    loading.value = true
    try {
      const value = validateCoverageMap(await getCoverageMap(run.id), run)
      if (disposed || epoch !== generation) return
      data.value = value; receivedAt.value = requested
      error.value = ['FAILED', 'UNAVAILABLE'].includes(value.state) ? value.reason || '作业区域暂不可用' : ''
      if (value.state === 'READY') void loadImages(value)
      else { release(raster.value); raster.value = null }
    } catch (reason) {
      if (!disposed && epoch === generation) error.value = reason instanceof Error ? reason.message : '作业区域未更新'
    } finally {
      loading.value = false
      if (mounted && !disposed && epoch !== generation) void refresh()
    }
  }
  watch(identity, () => { clear(); if (mounted) void refresh() })
  watch(getSource, () => {
    imageGeneration++; imageAbort?.abort(); imagesLoading.value = false
    if (snapshot.value) void loadImages(snapshot.value)
  })
  const onVisibility = () => { clock.value = Date.now(); if (document.visibilityState === 'visible') void refresh() }
  onMounted(() => {
    mounted = true; void refresh()
    timer = setInterval(() => void refresh(), 2000)
    document.addEventListener('visibilitychange', onVisibility)
  })
  onUnmounted(() => {
    disposed = true; mounted = false; clearTimeout(timer); clear()
    document.removeEventListener('visibilitychange', onVisibility)
  })
  return { snapshot, raster: visibleRaster, error, loading, imagesLoading, stale, refresh }
}
