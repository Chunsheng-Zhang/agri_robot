<script setup lang="ts">
import { computed, onMounted, onUnmounted, ref } from 'vue'
import 'leaflet/dist/leaflet.css'
import { ElMessage } from 'element-plus'
import { MapLocation } from '@element-plus/icons-vue'
import { useLocationMap } from '../../composables/useLocationMap'

const container = ref<HTMLElement>(), expanded = ref(false), configOpen = ref(false), keyInput = ref('')
const { configured, layerKind, following, lastPosition, updatedAt, fresh, tileError, loading, visible,
  setKey, retryTiles, locate, toggleFollow, refresh } = useLocationMap(container)
const viewport = ref({ top: 100, left: 244 })
const expandedStyle = computed(() => expanded.value ? { top: `${viewport.value.top}px`, left: `${viewport.value.left}px` } : {})
function resize() {
  viewport.value = { top: Math.max(12, (document.querySelector('.topbar')?.getBoundingClientRect().bottom || 84) + 12),
    left: Math.max(12, (document.querySelector('.sidebar')?.getBoundingClientRect().right || 0) + 12) }
}
function expand() { resize(); expanded.value = !expanded.value }
function escape(event: KeyboardEvent) { if (event.key === 'Escape') expanded.value = false }
function saveKey() {
  try { setKey(keyInput.value); keyInput.value = ''; configOpen.value = false; ElMessage.success('地图配置已保存在当前浏览器') }
  catch (value) { ElMessage.error(value instanceof Error ? value.message : '地图配置失败') }
}
function clearKey() { try { setKey(''); keyInput.value = ''; configOpen.value = false } catch (value) { ElMessage.error(String(value)) } }
onMounted(() => { window.addEventListener('resize', resize); document.addEventListener('keydown', escape) })
onUnmounted(() => { window.removeEventListener('resize', resize); document.removeEventListener('keydown', escape) })
defineExpose({ refreshData: refresh })
</script>
<template>
  <el-card class="location-card" :class="{ expanded }" :style="expandedStyle" shadow="never">
    <template #header>
      <div class="map-heading"><strong><el-icon><MapLocation /></el-icon> 机器人实时位置地图</strong><el-tag :type="fresh ? 'success' : 'info'">{{ fresh ? '定位更新中' : lastPosition ? '最后位置' : '等待定位' }}</el-tag></div>
    </template>
    <div class="map-toolbar">
      <el-radio-group v-model="layerKind" size="small" aria-label="地图底图"><el-radio-button value="satellite">卫星影像</el-radio-button><el-radio-button value="street">普通地图</el-radio-button></el-radio-group>
      <div class="map-buttons">
        <el-button size="small" :type="following ? 'success' : 'default'" :disabled="!lastPosition" :aria-pressed="following" @click="toggleFollow">{{ following ? '跟随车辆中' : '跟随车辆' }}</el-button>
        <el-button size="small" :disabled="!lastPosition" @click="locate">回到车辆</el-button>
        <el-button size="small" @click="expand">{{ expanded ? '收起地图' : '展开地图' }}</el-button>
        <el-button size="small" @click="configOpen = true">配置底图</el-button>
      </div>
    </div>
    <div class="map-area">
      <div ref="container" class="location-canvas" role="region" aria-label="车辆卫星位置地图" />
      <div v-if="!configured" class="map-notice" role="status">尚未配置天地图 Key。<el-button link type="primary" @click="configOpen = true">配置底图</el-button><small>车辆定位可独立显示，在线底图需要互联网连接。</small></div>
      <div v-else-if="tileError" class="map-notice error" role="status">{{ tileError }}<el-button link type="primary" @click="retryTiles">重新加载底图</el-button></div>
      <span v-else-if="loading && visible" class="map-loading" role="status">正在加载底图…</span>
      <span class="north-indicator" aria-label="正北方向">北 ↑</span>
    </div>
    <div class="map-coordinates" aria-live="polite">
      <span v-if="lastPosition">{{ fresh ? '当前位置' : '最后位置' }}：{{ lastPosition.longitude.toFixed(8) }}° E，{{ lastPosition.latitude.toFixed(8) }}° N</span>
      <span v-else>等待有效车辆位置，当前视野不代表车辆位置。</span>
      <span v-if="lastPosition">车头方位：{{ lastPosition.bearing === null ? '未更新' : `${lastPosition.bearing.toFixed(1)}°` }}</span>
    </div>
    <p class="map-note">定位时间：{{ updatedAt ? new Date(updatedAt).toLocaleString('zh-CN') : '未更新' }}。{{ lastPosition && !fresh ? '定位未更新，灰色标记仅表示最后位置。' : '箭头表示车头方向；拖动地图后可点击跟随车辆。' }}卫星影像为已有底图，实际清晰度取决于当地影像数据。</p>
    <el-dialog v-model="configOpen" title="配置地图底图" width="min(480px, 94vw)" append-to-body @closed="keyInput = ''">
      <p>填写你自己的天地图服务 Key。配置仅保存在当前浏览器，不发送到项目后端或写入 Git。</p>
      <el-input v-model="keyInput" type="password" show-password autocomplete="off" aria-label="天地图服务 Key" placeholder="请输入天地图服务 Key" @keyup.enter="saveKey" />
      <p class="config-note">当前状态：{{ configured ? '已配置（填写可替换）' : '尚未配置' }}。需要打开网页的电脑能访问天地图服务，并为本网页配置相应服务权限。更换浏览器或网页地址后需重新填写。</p>
      <template #footer><el-button v-if="configured" type="danger" plain @click="clearKey">清除配置</el-button><el-button @click="configOpen = false">取消</el-button><el-button type="primary" :disabled="!keyInput.trim()" @click="saveKey">保存并加载</el-button></template>
    </el-dialog>
  </el-card>
</template>
<style scoped>
.location-card { min-width: 0; border-radius: 10px; border-color: #e5eae7; }
.map-heading, .map-toolbar, .map-buttons { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }.map-heading { justify-content: space-between; }.map-heading strong { display: flex; align-items: center; gap: 7px; color: #263b30; }
.map-toolbar { justify-content: space-between; margin-bottom: 12px; }.map-buttons { gap: 6px; }.map-buttons .el-button { margin-left: 0; }
.map-area { position: relative; }.location-canvas { height: clamp(240px, 38vh, 360px); background: #edf2ed; border-radius: 8px; border: 1px solid #dce5dd; z-index: 0; }
.map-notice { position: absolute; z-index: 1; top: 12px; left: 50px; right: 44px; padding: 10px 14px; border: 1px solid #e1e8dd; border-radius: 8px; background: #fffffff0; color: #586d5b; font-size: 12px; line-height: 1.7; box-shadow: 0 2px 10px #263b3012; }.map-notice small { display: block; }.map-notice.error { color: #98652d; }
.map-loading { position: absolute; top: 12px; left: 50px; z-index: 1; background: #fffffff0; padding: 7px 10px; border-radius: 6px; font-size: 12px; color: #586d5b; }
.north-indicator { position: absolute; top: 12px; right: 10px; z-index: 1; background: #fffffff0; padding: 6px; border-radius: 5px; font-size: 12px; color: #536a58; pointer-events: none; }
.map-coordinates { display: flex; flex-wrap: wrap; gap: 8px 18px; padding-top: 12px; font-size: 12px; color: #5f7568; font-variant-numeric: tabular-nums; }.map-note, .config-note { color: #819087; font-size: 12px; line-height: 1.8; margin-bottom: 0; }
.location-card.expanded { position: fixed; right: 12px; bottom: 12px; z-index: 8; display: flex; flex-direction: column; box-shadow: 0 6px 40px #172b3c40; }.expanded :deep(.el-card__body) { display: flex; flex-direction: column; flex: 1; min-height: 0; }.expanded .map-area { flex: 1; min-height: 160px; }.expanded .location-canvas { height: 100%; }
:deep(.robot-map-marker) { background: transparent; border: none; }:deep(.robot-map-pin) { width: 34px; height: 34px; color: #237ac0; filter: drop-shadow(0 2px 4px #172b3c40); }:deep(.robot-map-pin.stale) { color: #89928e; }:deep(.robot-map-pin svg) { display: block; width: 100%; height: 100%; }
@media (max-width: 600px) { .map-toolbar { gap: 10px; }.map-notice { left: 46px; right: 36px; padding: 8px; }.map-coordinates { font-size: 11px; } }
</style>
