import { computed, onMounted, onUnmounted, ref, watch } from 'vue'
import type { Ref } from 'vue'
import * as L from 'leaflet'
import { useRealtime } from './useRealtime'
import { useWorkspace } from './useWorkspace'
import type { MapPosition } from '../api/robot'

const storageKey = 'agri-robot.tianditu.web-key'
export type BasemapKind = 'satellite' | 'street'
export function validPosition(value?: MapPosition | null): value is MapPosition {
  return !!value && value.reference === 'base_link' && value.crs === 'EPSG:4326'
    && Number.isFinite(value.latitude) && Math.abs(value.latitude) <= 85
    && Number.isFinite(value.longitude) && Math.abs(value.longitude) <= 180
    && (value.bearing === null || Number.isFinite(value.bearing))
}
export function tileUrl(layer: 'img' | 'cia' | 'vec' | 'cva', key: string) {
  return `https://t{s}.tianditu.gov.cn/${layer}_w/wmts?SERVICE=WMTS&REQUEST=GetTile&VERSION=1.0.0&LAYER=${layer}&STYLE=default&TILEMATRIXSET=w&FORMAT=tiles&TILEMATRIX={z}&TILEROW={y}&TILECOL={x}&tk=${encodeURIComponent(key)}`
}

export function useLocationMap(container: Ref<HTMLElement | undefined>) {
  const { robot, poseFresh, refresh } = useRealtime()
  const { context } = useWorkspace()
  const key = ref(''), layerKind = ref<BasemapKind>('satellite'), following = ref(true)
  const lastPosition = ref<MapPosition | null>(null), updatedAt = ref('')
  const tileError = ref(''), loading = ref(false), visible = ref(true)
  const fresh = computed(() => poseFresh.value && validPosition(robot.value?.map_position))
  const configured = computed(() => !!key.value)
  let map: L.Map | undefined, marker: L.Marker | undefined
  let layers: L.TileLayer[] = [], epoch = 0, centered = false, disposed = false
  let originApplied = false, manualView = false, markerAppearance = ''
  let observer: ResizeObserver | undefined, intersection: IntersectionObserver | undefined
  let tileTimeout: ReturnType<typeof setTimeout> | undefined
  const pending = new Set<L.TileLayer>()
  let inViewport = true

  function detachTiles() {
    epoch += 1
    clearTimeout(tileTimeout); tileTimeout = undefined
    pending.clear(); loading.value = false
    for (const layer of layers) { layer.off(); layer.remove() }
    layers = []
  }
  function failTiles() {
    detachTiles()
    tileError.value = '底图加载失败或超时，请检查网络、地图 Key 和服务权限后重试。'
  }
  function attachTiles() {
    detachTiles()
    if (!map || !key.value || !visible.value || tileError.value || disposed) return
    const version = epoch
    const codes = layerKind.value === 'satellite' ? ['img', 'cia'] as const : ['vec', 'cva'] as const
    // Install both layers' handlers before starting either request batch.
    layers = codes.map(code => {
      const layer = L.tileLayer(tileUrl(code, key.value), {
        minZoom: 2, maxNativeZoom: 18, maxZoom: 20, noWrap: true,
        subdomains: ['0', '1'],
        updateWhenIdle: false, updateWhenZooming: true, updateInterval: 200,
        keepBuffer: 1, detectRetina: false,
        attribution: '<a href="https://www.tianditu.gov.cn/" target="_blank" rel="noopener noreferrer">天地图</a>',
      })
      layer.on('loading', () => {
        if (version !== epoch) return
        pending.add(layer); loading.value = true
        if (!tileTimeout) tileTimeout = setTimeout(() => { if (version === epoch) failTiles() }, 8000)
      })
      layer.on('load', () => {
        if (version !== epoch) return
        pending.delete(layer)
        if (!pending.size) { loading.value = false; clearTimeout(tileTimeout); tileTimeout = undefined }
      })
      layer.on('tileerror', () => { if (version === epoch) failTiles() })
      return layer
    })
    for (const layer of [...layers]) { if (version !== epoch) break; layer.addTo(map) }
  }
  function icon(position: MapPosition) {
    const arrow = position.bearing === null ? '' : '<path d="M 16 2 L 23 22 L 16 18 L 9 22 Z" fill="currentColor" stroke="white" stroke-width="1.5"/>'
    return L.divIcon({ className: 'robot-map-marker', iconSize: [34, 34], iconAnchor: [17, 17],
      html: `<div class="robot-map-pin ${fresh.value ? 'live' : 'stale'}"><svg viewBox="0 0 32 32" style="transform:rotate(${position.bearing ?? 0}deg)"><circle cx="16" cy="16" r="12" fill="white" stroke="currentColor" stroke-width="2"/>${arrow || '<circle cx="16" cy="16" r="5" fill="currentColor"/>'}</svg></div>` })
  }
  function followPosition() {
    if (!map || !lastPosition.value || !following.value || !fresh.value) return
    const latlng: L.LatLngTuple = [lastPosition.value.latitude, lastPosition.value.longitude]
    if (!centered) { map.setView(latlng, manualView ? map.getZoom() : 18, { animate: false }); centered = true; return }
    const point = map.latLngToContainerPoint(latlng), size = map.getSize()
    if (point.x < size.x * 0.2 || point.x > size.x * 0.8 || point.y < size.y * 0.2 || point.y > size.y * 0.8) {
      map.panTo(latlng, { animate: false })
    }
  }
  function renderPosition() {
    if (!map || !lastPosition.value || !visible.value) return
    const position = lastPosition.value
    const latlng: L.LatLngTuple = [position.latitude, position.longitude]
    const appearance = `${fresh.value}:${position.bearing}`
    if (!marker) marker = L.marker(latlng, { icon: icon(position), keyboard: false }).addTo(map).bindTooltip('')
    else {
      marker.setLatLng(latlng)
      if (appearance !== markerAppearance) marker.setIcon(icon(position))
    }
    if (appearance !== markerAppearance) marker.setTooltipContent(fresh.value ? '车辆当前位置' : '最后位置（定位未更新）')
    markerAppearance = appearance
    followPosition()
  }
  function locate() {
    if (!map || !lastPosition.value) return
    map.setView([lastPosition.value.latitude, lastPosition.value.longitude], Math.max(17, map.getZoom()), { animate: false })
    centered = true
  }
  function toggleFollow() { following.value = !following.value; if (following.value) locate() }
  function retryTiles() { tileError.value = ''; attachTiles() }
  function setKey(value: string) {
    const cleaned = value.trim()
    if (cleaned && !/^[a-zA-Z0-9_-]{16,128}$/.test(cleaned)) throw new Error('请填写有效的天地图服务 Key')
    try {
      if (cleaned) localStorage.setItem(storageKey, cleaned)
      else localStorage.removeItem(storageKey)
    } catch { throw new Error('浏览器不允许保存地图配置，请检查隐私或存储设置') }
    key.value = cleaned
    retryTiles()
  }
  function onVisibility() { visible.value = document.visibilityState === 'visible' && inViewport }
  const stops = [
    watch([fresh, () => robot.value?.map_position], () => {
      if (fresh.value && validPosition(robot.value?.map_position)) {
        lastPosition.value = { ...robot.value.map_position }
        updatedAt.value = robot.value.last_odom_time || new Date().toISOString()
      }
      renderPosition()
    }, { immediate: true }),
    watch(() => context.value?.origin, origin => {
      if (map && !originApplied && !manualView && !lastPosition.value && origin && Number.isFinite(origin.latitude) && Number.isFinite(origin.longitude)) {
        map.setView([origin.latitude, origin.longitude], 18, { animate: false })
        originApplied = true
      }
    }),
    watch(layerKind, retryTiles),
    watch(visible, value => {
      if (value) { map?.invalidateSize({ pan: false }); attachTiles(); renderPosition() }
      else detachTiles()
    }),
  ]
  onMounted(() => {
    if (!container.value) return
    try { key.value = localStorage.getItem(storageKey) || '' } catch { /* Still allow an unconfigured map. */ }
    const origin = context.value?.origin
    map = L.map(container.value, { zoomControl: true, minZoom: 2, maxZoom: 20,
      center: origin ? [origin.latitude, origin.longitude] : [0, 0], zoom: origin ? 18 : 2 })
    originApplied = !!origin
    L.control.scale({ imperial: false, maxWidth: 100 }).addTo(map)
    map.on('dragstart boxzoomstart', () => { following.value = false; manualView = true })
    map.on('zoomstart', () => { manualView = true })
    observer = new ResizeObserver(() => { map?.invalidateSize({ pan: false }) })
    observer.observe(container.value)
    if (typeof IntersectionObserver !== 'undefined') {
      intersection = new IntersectionObserver(entries => { inViewport = !!entries[0]?.isIntersecting; onVisibility() })
      intersection.observe(container.value)
    }
    document.addEventListener('visibilitychange', onVisibility)
    onVisibility(); attachTiles(); renderPosition()
  })
  onUnmounted(() => {
    disposed = true
    stops.forEach(stop => stop()); observer?.disconnect(); intersection?.disconnect()
    document.removeEventListener('visibilitychange', onVisibility)
    detachTiles(); map?.remove(); map = undefined; marker = undefined
  })
  return { configured, layerKind, following, lastPosition, updatedAt, fresh, tileError, loading, visible,
    setKey, retryTiles, locate, toggleFollow, refresh }
}
