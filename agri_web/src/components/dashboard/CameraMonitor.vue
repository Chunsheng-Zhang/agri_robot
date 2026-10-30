<script setup lang="ts">
import { cameraPreview, useCameraPreview } from '../../composables/useCameraPreview'

const { camera, connected, enabled, visible, imageUrl, error, loading, toggle, imageError } = useCameraPreview()
</script>

<template>
  <el-card class="camera-panel" shadow="never">
    <template #header>
      <div class="heading">
        <span>相机辅助画面</span>
        <div class="actions">
          <el-tag :type="connected ? 'success' : 'info'">{{ connected ? '相机在线' : '相机未连接 / 状态过期' }}</el-tag>
          <el-button size="small" :disabled="!connected && !enabled" @click="toggle">{{ enabled ? '暂停画面' : '开启画面' }}</el-button>
        </div>
      </div>
    </template>
    <div class="video-frame">
      <img v-if="visible && enabled && connected && imageUrl" :src="imageUrl" alt="Astra Pro Plus 低带宽预览" @error="imageError" />
      <div v-else class="video-placeholder">{{ !enabled ? '画面默认暂停，点击“开启画面”查看' : !connected ? (camera?.reason || '等待相机图像') : error || (loading ? '正在获取画面…' : '等待画面更新') }}</div>
    </div>
    <div class="camera-meta">低带宽预览 {{ cameraPreview.width }} × {{ cameraPreview.height }} · 最多 2 帧/秒 · 切到后台自动暂停</div>
  </el-card>
</template>

<style scoped>
.camera-panel { border: 1px solid #e8eee9; border-radius: 10px; margin: 20px 0; }
.heading, .actions { display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 10px; }
.video-frame { min-height: 240px; background: #14251d; border-radius: 8px; display: grid; place-items: center; overflow: hidden; }
.video-frame img { width: 100%; max-height: 480px; object-fit: contain; }
.video-placeholder { color: #c0cdc5; padding: 40px 12px; text-align: center; }
.camera-meta { padding-top: 12px; color: #7b8980; font-size: 12px; line-height: 1.6; }
</style>
