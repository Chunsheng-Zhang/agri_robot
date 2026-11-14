<script setup lang="ts">
import { computed, nextTick, ref, watch } from 'vue'
import { Document, Connection, Close } from '@element-plus/icons-vue'
import { useDiagnostics } from '../composables/useDiagnostics'
const { active, paused, entries, page, error, skipped, loading, fresh, open, close, clearDisplay } = useDiagnostics()
const level = ref('ALL'), direction = ref('ALL'), search = ref(''), follow = ref(true)
const body = ref<HTMLElement>(), panel = ref<HTMLElement>()
const levels: Record<string,number> = { DEBUG: 10, INFO: 20, WARN: 30, ERROR: 40, FATAL: 50 }
const filtered = computed(() => entries.value.filter(row => {
  if (active.value === 'logs' && level.value !== 'ALL' && (levels[row.level || 'INFO'] || 20) < (levels[level.value] || 0)) return false
  if (active.value === 'serial' && direction.value !== 'ALL' && row.direction !== direction.value) return false
  const text = `${row.source} ${row.message || ''} ${row.summary || ''} ${row.function_name || ''} ${row.frame || ''}`
  return !search.value || text.toLowerCase().includes(search.value.toLowerCase())
}))
const status = computed(() => page.value?.status)
const timestamp = (value: number) => new Date(value / 1e6).toLocaleTimeString('zh-CN', {
  hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit', fractionalSecondDigits: 3, timeZone: 'Asia/Shanghai',
})
watch(active, async () => { search.value = ''; await nextTick(); panel.value?.focus() })
watch(filtered, async () => { if (follow.value) { await nextTick(); if (body.value) body.value.scrollTop = body.value.scrollHeight } })
</script>
<template>
  <div class="diagnostic-buttons">
    <el-button size="small" :icon="Document" :type="active === 'logs' ? 'primary' : 'default'" @click="open('logs')">日志</el-button>
    <el-button size="small" :icon="Connection" :type="active === 'serial' ? 'primary' : 'default'" @click="open('serial')">信号流</el-button>
  </div>
  <Teleport to="body">
    <section v-if="active" ref="panel" class="diagnostic-panel" tabindex="-1" role="region" :aria-label="active === 'logs' ? '实时运行日志' : 'Jetson与MCU串口信号流'" @keydown.esc="close">
      <header><strong>{{ active === 'logs' ? '实时运行日志' : 'Jetson ↔ MCU 串口信号流' }}</strong><el-button :icon="Close" circle text aria-label="关闭诊断面板" @click="close" /></header>
      <p v-if="active === 'logs'" class="hint">本次后端会话的ROS日志、任务异常、安全事件及当前一键启动服务输出；可按来源和错误信息搜索。</p>
      <template v-else>
        <p class="hint">TX：Jetson串口写入的数据；RX：从MCU串口读取的数据。TX写入成功不代表MCU已确认执行。这里显示实际观察帧，不用ROS速度话题代替。</p>
        <div class="serial-status">
          <span :class="{ good: status?.observer_online && fresh }">采集节点：{{ !fresh || !status?.observer_online ? '未更新 / 未接入' : '在线' }}</span>
          <span :class="{ good: status?.rx_fresh && fresh }">MCU接收：{{ fresh && status?.rx_fresh ? '有新鲜有效RX帧' : '未更新' }}</span>
          <span>串口：{{ status?.port || '--' }}</span><span>TX {{ status?.tx_frames ?? '--' }} / RX {{ status?.rx_frames ?? '--' }}</span>
        </div>
        <p v-if="!status?.observer_online" class="hint">需要由新版项目底盘包装入口启动采集；没有底盘或仅运行原厂独立命令时，不会生成串口信号数据。</p>
        <p v-if="status?.error" class="warning">{{ status.error }}</p>
        <p v-if="status?.observer_dropped || status?.sequence_gaps || status?.bad_frames || status?.malformed_batches" class="warning">采集队列丢弃 {{ status.observer_dropped || 0 }} · 观察序号缺口 {{ status.sequence_gaps }} · 校验异常 {{ status.bad_frames }} · 格式异常 {{ status.malformed_batches }}（不等于串口链路丢包率）</p>
      </template>
      <div class="tools">
        <el-select v-if="active === 'logs'" v-model="level" aria-label="日志等级" style="width: 130px"><el-option label="全部等级" value="ALL" /><el-option label="INFO及以上" value="INFO" /><el-option label="WARN及以上" value="WARN" /><el-option label="ERROR及以上" value="ERROR" /></el-select>
        <el-select v-else v-model="direction" aria-label="信号方向" style="width: 130px"><el-option label="TX与RX" value="ALL" /><el-option label="TX发送" value="TX" /><el-option label="RX接收" value="RX" /></el-select>
        <el-input v-model="search" placeholder="搜索来源、错误、功能字或原始HEX" clearable style="width: min(100%, 300px)" />
        <el-button size="small" @click="paused = !paused">{{ paused ? '继续实时显示' : '暂停显示' }}</el-button>
        <el-button size="small" @click="clearDisplay">清屏</el-button>
        <el-checkbox v-model="follow">自动滚动</el-checkbox>
      </div>
      <p v-if="error" class="warning" role="alert">{{ error }}。保留的内容为上次收到的数据。</p>
      <p v-if="paused" class="hint">显示已暂停，后台继续采集；继续后跳到最新记录。</p>
      <p v-if="skipped" class="warning">本次显示已跳过 {{ skipped }} 条缓冲记录；这不表示车辆通信丢包。</p>
      <div ref="body" class="rows">
        <p v-if="!filtered.length" class="hint">{{ loading ? '正在读取…' : '暂无匹配记录，等待实时数据' }}</p>
        <article v-for="row in filtered" :key="row.id" :class="[row.direction || row.level, { invalid: row.checksum_valid === false }]">
          <span class="stamp">{{ timestamp(row.unix_ns) }}</span>
          <template v-if="active === 'logs'"><b>{{ row.level }}</b><span class="source">{{ row.source }}</span><span class="message">{{ row.message }}</span></template>
          <template v-else><b>{{ row.direction }}</b><span>0x{{ row.function?.toString(16).padStart(2, '0').toUpperCase() }} {{ row.function_name }}</span><span>{{ row.summary }}</span><code>{{ row.frame }}</code></template>
        </article>
      </div>
      <footer>页面最多保留500条，清屏不删除文件。<span v-if="page?.archive_directory">采集目录：{{ page.archive_directory }}</span><span v-if="page?.archive_dropped || page?.archive_error" class="warning">归档丢弃 {{ page.archive_dropped }} · {{ page.archive_error }}</span></footer>
    </section>
  </Teleport>
</template>
<style scoped>
.diagnostic-buttons { display: flex; gap: 6px; margin-left: auto; flex-shrink: 0; }.diagnostic-buttons .el-button { margin-left: 0; }
.diagnostic-panel { position: fixed; z-index: 25; top: 76px; right: 12px; bottom: 14px; width: min(1060px, calc(100vw - 24px)); display: flex; flex-direction: column; background: white; border: 1px solid #ceddd2; border-radius: 10px; box-shadow: 0 10px 40px #142a3038; padding: 12px 16px; box-sizing: border-box; outline: none; }
header { display: flex; justify-content: space-between; align-items: center; } .hint,footer { font-size: 12px; color: #66756c; line-height: 1.6; margin: 6px 0; }.tools,.serial-status { display: flex; flex-wrap: wrap; gap: 8px 14px; align-items: center; }.serial-status { font-size: 12px; color: #a85a36; }.good { color: #238448; }.warning { color: #ba5d26; font-size: 12px; }
.rows { flex: 1; min-height: 0; overflow: auto; margin: 10px 0; border: 1px solid #e0e8e3; background: #f8faf9; padding: 8px; font-family: monospace; font-size: 12px; }
article { display: flex; flex-wrap: wrap; gap: 4px 10px; padding: 4px 0; border-bottom: 1px solid #e8eee9; align-items: baseline; }article .message { flex: 1; min-width: 150px; white-space: pre-wrap; overflow-wrap: anywhere; }.stamp { color: #78857e; }.source { color: #52685d; } .TX { color: #1a56c9; }.RX { color: #08753b; }.ERROR,.FATAL,.invalid { color: #c23535; }.WARN { color: #9b651c; }code { width: 100%; padding-left: 103px; overflow-wrap: anywhere; white-space: normal; }
footer { overflow-wrap: anywhere; } @media(max-width:760px) { .diagnostic-panel { padding: 8px; top: 112px; }.diagnostic-buttons { gap: 3px; }.diagnostic-buttons .el-button { padding: 6px; }code { padding-left: 0; } }
</style>
