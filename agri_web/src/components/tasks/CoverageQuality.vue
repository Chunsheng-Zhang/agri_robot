<script setup lang="ts">
import { computed, ref, watch } from 'vue'
import { coverageQualityUrl } from '../../api/workspace'
import type { Run } from '../../api/workspace'
import { numberText } from '../../composables/useRealtime'
import OptimizationResults from './OptimizationResults.vue'

const props = defineProps<{ run: Run | null }>()
const source = ref<'planned' | 'actual'>('planned')
const region = ref<'field' | 'interior' | 'headland'>('field')
const quality = computed(() => props.run?.coverage_quality)
const estimate = computed(() => quality.value?.[source.value])
const metrics = computed(() => estimate.value?.metrics?.[region.value])
const image = computed(() => props.run && estimate.value?.available
  ? coverageQualityUrl(props.run.id, source.value === 'planned' ? 'plan.png' : 'actual.png') : '')
const pending = computed(() => props.run?.coverage_quality_version === 1
  && !['SUCCEEDED', 'FAILED'].includes(quality.value?.state || ''))
watch(() => props.run?.id, () => { source.value = 'planned' })
watch(() => quality.value?.actual?.available, available => {
  if (!available && source.value === 'actual') source.value = 'planned'
})
</script>

<template>
  <el-card class="coverage-quality" shadow="never">
    <template #header>
      <div class="quality-heading"><strong>规划作业质量评估</strong><span v-if="run">{{ run.name }} · v{{ run.mission_revision }}</span></div>
    </template>
    <slot name="parameters" />
    <OptimizationResults v-if="run && (run.planning_strategy === 'SMART' || run.optimization)" :run="run" />
    <p v-if="!run || !run.coverage_quality_version">生成覆盖路径后显示本轮预测覆盖率和重叠率；历史记录不会自动回填。</p>
    <el-alert v-else-if="pending" title="正在等待规划结果或后台计算作业质量，完成后自动显示" type="info" :closable="false" />
    <el-alert v-else-if="quality?.state === 'FAILED'" :title="quality.reason || '覆盖质量评价未生成'" type="warning" :closable="false" />
    <template v-else>
      <div class="quality-toolbar">
        <el-radio-group v-model="source" size="small" aria-label="覆盖评价数据来源">
          <el-radio-button value="planned">规划预测</el-radio-button>
          <el-radio-button value="actual" :disabled="!quality?.actual?.available">实际轨迹估计</el-radio-button>
        </el-radio-group>
        <el-select v-model="region" aria-label="覆盖评价区域" size="small" style="width: 180px">
          <el-option value="field" label="整块田（扣除禁入区）" />
          <el-option value="interior" label="内部作业区" />
          <el-option value="headland" label="地头区" />
        </el-select>
      </div>
      <div v-if="metrics?.available" class="quality-metrics">
        <div><span>覆盖率</span><strong>{{ numberText(metrics.coverage_percent, 2) }}%</strong><el-progress :percentage="Math.max(0, Math.min(100, metrics.coverage_percent || 0))" :show-text="false" color="#38956c" /></div>
        <div><span>重复覆盖率（重叠率）</span><strong>{{ numberText(metrics.overlap_percent, 2) }}%</strong><small>至少两次经过的区域 / 评价区域</small></div>
        <div><span>漏覆盖率</span><strong>{{ numberText(metrics.miss_percent, 2) }}%</strong><small>{{ numberText(metrics.missed_m2, 2) }} m² 尚未覆盖</small></div>
        <div><span>评价区域面积</span><strong>{{ numberText(metrics.area_m2, 2) }} m²</strong><small>已覆盖 {{ numberText(metrics.covered_m2, 2) }} m² · 重叠 {{ numberText(metrics.overlap_m2, 2) }} m²</small></div>
      </div>
      <el-alert v-else title="所选区域为空或没有有效评价数据" type="info" :closable="false" />
      <p class="quality-note">{{ source === 'planned' ? '预测值以本轮保存的参考路径计算。' : '估计值只使用正式跟踪的已记录轨迹，空档不连线。' }}假设机具沿路径持续开启，包括连接转弯；整田指标包含保留地头。覆盖率与重叠率并非互补关系。</p>
      <p class="quality-note">评价幅宽 {{ numberText(estimate?.width_m, 3) }} m · 实际网格 {{ numberText(estimate?.resolution_m, 3) }} m · 两级网格覆盖率差 {{ numberText(estimate?.sensitivity?.coverage_difference_pp, 3) }} 个百分点。颜色：红色漏覆盖，绿色一次经过，橙色重复经过；图中区域不随上方统计区域切换裁剪。</p>
      <img v-if="image" :key="image" :src="image" alt="本轮规划或正式跟踪的覆盖、漏覆盖和重复覆盖分布图" class="quality-map" loading="lazy" />
      <p v-if="!quality?.actual?.available" class="quality-note">{{ quality?.actual?.reason || '任务结束后自动生成实际轨迹估计及完整作业质量报告。' }}</p>
      <el-alert v-if="quality?.actual_error" :title="quality.actual_error" type="warning" :closable="false" />
      <div v-if="run && quality?.actual" class="quality-links">
        <a :href="coverageQualityUrl(run.id, 'report.md')" target="_blank" rel="noopener">打开作业质量报告</a>
        <a :href="coverageQualityUrl(run.id, 'metrics.json')" target="_blank" rel="noopener">查看评价数据 JSON</a>
        <span>图表、CSV与评价口径随本轮数据归档保存。</span>
      </div>
    </template>
  </el-card>
</template>

<style scoped>
.coverage-quality { margin: 20px 0; border-radius: 10px; }
.quality-heading, .quality-toolbar, .quality-links { display: flex; align-items: center; flex-wrap: wrap; gap: 12px; }
.quality-heading { justify-content: space-between; }
.quality-heading span, .quality-note, .quality-links { font-size: 12px; color: #6c7e73; line-height: 1.8; }
.quality-metrics { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 12px; margin: 16px 0; }
.quality-metrics > div { padding: 14px; background: #f3f8f4; border-radius: 8px; min-width: 0; }
.quality-metrics span, .quality-metrics small { display: block; color: #6c7e73; font-size: 12px; line-height: 1.8; }
.quality-metrics strong { display: block; color: #28734f; font-size: 22px; margin: 8px 0; overflow-wrap: anywhere; }
.quality-map { display: block; max-width: 100%; width: 100%; max-height: 420px; object-fit: contain; }
.quality-links a { color: #28734f; }
@media (max-width: 1000px) { .quality-metrics { grid-template-columns: repeat(2, minmax(0, 1fr)); } }
@media (max-width: 480px) { .quality-metrics { grid-template-columns: 1fr; } }
</style>
