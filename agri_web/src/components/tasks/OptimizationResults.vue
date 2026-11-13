<script setup lang="ts">
import { computed } from 'vue'
import type { Run } from '../../api/workspace'
import { optimizationReportUrl } from '../../api/workspace'
import { numberText } from '../../composables/useRealtime'
const props = defineProps<{ run: Run }>()
const result = computed(() => props.run.optimization)
const selected = computed(() => result.value?.selected)
const pointTurn = computed(() => result.value?.turn_mode === 'POINT_TURN')
const names: Record<string,string> = {BOUSTROPHEDON:'牛耕',SNAKE:'蛇形',SPIRAL:'螺旋',CONTOUR:'回形',DUBINS:'Dubins',DUBINS_CC:'Dubins CC',REEDS_SHEPP:'Reeds–Shepp',REEDS_SHEPP_HC:'Reeds–Shepp HC'}
</script>
<template>
  <section class="optimization-results" aria-label="一键优化规划结果">
    <strong>{{ run.source_plan_id ? '已保存路径的原始优化结果' : '一键优化规划结果' }}</strong>
    <p v-if="run.source_plan_id">沿用源路径原始评分，本轮没有重新优化。</p>
    <el-alert v-if="!result" title="正在生成、校验并比较候选；完成后显示结果，规划不会自动开始行驶" type="info" :closable="false" />
    <template v-else>
      <template v-if="pointTurn">
        <el-alert v-if="result.state === 'FAILED'" :title="result.reason || '没有可执行方案'" type="warning" :closable="false" />
        <div v-if="selected" class="result-metrics">
          <div><span>推荐作业角度</span><b>{{ numberText(selected.angle_rad * 180 / Math.PI, 2) }}°</b></div>
          <div><span>最小安全地头</span><b>{{ numberText(selected.headland_width_m, 3) }} m</b></div>
          <div><span>路径总长度</span><b>{{ numberText(selected.length_m, 2) }} m</b></div>
          <div><span>原地转向点</span><b>{{ selected.point_turn_count ?? 0 }}</b></div>
        </div>
        <p v-if="selected">往复式{{ selected.order === 'LOCAL_2OPT' ? '（行顺序已局部优化）' : '（基础行顺序）' }} · 覆盖 {{ numberText(selected.coverage_percent, 2) }}% · 重复覆盖 {{ numberText(selected.overlap_percent, 2) }}% · 综合分 {{ numberText(selected.score, 4) }}（越低越好）。</p>
        <p>固定权重：覆盖率 0.5、重复覆盖率 0.3、总长度 0.2；在同一整田评价网格比较，机具有效幅宽保持不变。长度采用 L/(L+A/W) 归一化，含全部换行连接；不评价能耗或时间。</p>
        <p>车辆包络：长 {{ numberText(result.headland_search?.robot_length_m, 3) }} m、宽 {{ numberText(result.headland_search?.robot_width_m, 3) }} m、安全余量 {{ numberText(result.headland_search?.safety_margin_m, 3) }} m。转向点停车等待、原地对齐后继续，默认尺寸须实测核对。</p>
        <p>已比较 {{ result.evaluated_count }} 个候选 · 用时 {{ numberText(result.elapsed_seconds, 2) }} s · {{ result.state === 'FAILED' ? '没有可执行方案' : result.search_complete ? '已完成全部预定角度及行顺序比较' : result.budget_exhausted ? '历史结果：达到计算预算' : '完成本轮角度搜索' }}；不保证全局最优。实际评价网格 {{ numberText(result.final_resolution_m, 3) }} m。</p>
        <details><summary>角度与行顺序候选对比</summary>
          <div class="table-scroll"><table>
            <thead><tr><th>编号</th><th>角度 °</th><th>行顺序</th><th>覆盖 %</th><th>重复 %</th><th>总长度 m</th><th>综合评分</th><th>状态</th></tr></thead>
            <tbody><tr v-for="candidate in result.candidates" :key="candidate.id" :class="{ chosen: candidate.id === selected?.id }">
              <td>{{ candidate.id }}</td><td>{{ numberText(candidate.angle_rad * 180 / Math.PI, 2) }}</td>
              <td>{{ candidate.order === 'LOCAL_2OPT' ? '局部优化' : '往复式' }}</td>
              <td>{{ numberText(candidate.coverage_percent, 2) }}</td><td>{{ numberText(candidate.overlap_percent, 2) }}</td>
              <td>{{ numberText(candidate.length_m, 2) }}</td><td>{{ numberText(candidate.score, 4) }}</td>
              <td>{{ candidate.id === selected?.id ? '已选定' : candidate.reason || (candidate.executable ? '备选' : '不可用') }}</td>
            </tr></tbody>
          </table></div>
        </details>
        <a :href="optimizationReportUrl(run.id)" target="_blank" rel="noopener">查看本轮角度搜索与原地转向规划报告 JSON</a>
      </template>
      <template v-else>
      <el-alert v-if="result.state === 'FAILED'" :title="result.reason || '没有可执行的完整候选'" type="warning" :closable="false" />
      <div v-if="selected" class="result-metrics">
        <div><span>选定覆盖形式</span><b>{{ names[selected.mode] || selected.mode }}</b></div>
        <div><span>选定地头宽度</span><b>{{ numberText(selected.headland_width_m, 3) }} m</b></div>
        <div><span>路径总长度</span><b>{{ numberText(selected.length_m, 2) }} m</b></div>
        <div><span>能耗代理成本</span><b>{{ numberText(selected.energy_proxy, 2) }}</b><small>未标定的相对成本，非 Wh</small></div>
      </div>
      <p v-if="selected">方向 {{ numberText(selected.angle_rad * 180 / Math.PI, 1) }}° · 正式细网格覆盖 {{ numberText(selected.coverage_percent, 2) }}% · 重复覆盖 {{ numberText(selected.overlap_percent, 2) }}%</p>
      <p>已比较 {{ result.evaluated_count }} / {{ result.possible_count }} 个组合 · 搜索用时 {{ numberText(result.elapsed_seconds, 2) }} s · {{ result.budget_exhausted ? '达到候选或时间预算' : '完成本次组合搜索' }}。</p>
      <p>仅代表本次候选集合与评分下的选择，不保证全局最优。搜索网格 {{ numberText(result.search_resolution_m, 3) }} m，最终评价网格 {{ numberText(result.final_resolution_m, 3) }} m。</p>
      <p v-if="result.coverage_constraint_enabled === false">覆盖率仅作评价展示，不设最低门槛。</p>
      <p v-if="result.headland_search?.method === 'MANUAL'">地头宽度严格采用任务“规划与路径跟踪参数”中的人工值，不搜索其他宽度。</p>
      <details v-else-if="result.headland_search"><summary>自动地头搜索依据</summary>
        <p>系统估计搜索范围 {{ numberText(result.headland_search.range_min_m, 3) }}～{{ numberText(result.headland_search.range_max_m, 3) }} m，无需填写上下限。</p>
        <p>粗搜索 {{ result.headland_search.coarse_evaluated_count ?? 0 }} 套，细化搜索 {{ result.headland_search.refinement_evaluated_count ?? 0 }} 套；每套仍需完整路径净空和方向校验。</p>
        <p>{{ result.headland_search.limitations }}</p>
      </details>
      <details><summary>完整方案对比与拒绝原因</summary>
        <div class="table-scroll"><table>
          <thead><tr><th>编号</th><th>形式</th><th>地头 m</th><th>方向 °</th><th>搜索覆盖 %</th><th>搜索重叠 %</th><th>长度 m</th><th>能耗代理</th><th>搜索评分</th><th>结果</th></tr></thead>
          <tbody><tr v-for="candidate in result.candidates" :key="candidate.id" :class="{ chosen: candidate.id === selected?.id }">
            <td>{{ candidate.id }}</td><td>{{ names[candidate.mode] || candidate.mode }}</td><td>{{ numberText(candidate.headland_width_m, 3) }}</td><td>{{ numberText(candidate.angle_rad * 180 / Math.PI, 1) }}</td>
            <td>{{ numberText(candidate.coverage_percent, 2) }}</td><td>{{ numberText(candidate.overlap_percent, 2) }}</td><td>{{ numberText(candidate.length_m, 2) }}</td><td>{{ numberText(candidate.energy_proxy, 2) }}</td><td>{{ numberText(candidate.score, 4) }}</td>
            <td>{{ candidate.id === selected?.id ? '已选定' : candidate.reason || (candidate.executable ? '备选' : '不可用') }}</td>
          </tr></tbody>
        </table></div>
      </details>
      <details v-if="result.curve_type"><summary>Fields2Cover 原生转弯曲线（最多展示24段）</summary>
        <p>使用 {{ names[result.curve_type] || result.curve_type }}，按任务设定的最小半径生成。{{ result.allow_reverse ? '本轮允许倒车，换向点停车对齐后继续。' : '本轮不允许倒车，含倒车段的方案不会发布。' }}</p>
        <div class="table-scroll"><table><thead><tr><th>原生曲线</th><th>设定最小半径 m</th><th>长度 m</th><th>倒车 m</th><th>能耗代理</th><th>当前可执行</th><th>说明</th></tr></thead><tbody>
          <tr v-for="(turn,index) in result.turn_curves" :key="index"><td>{{ names[turn.type] || turn.type }}</td><td>{{ numberText(turn.radius_m, 2) }}</td><td>{{ numberText(turn.metrics.length_m, 2) }}</td><td>{{ numberText(turn.metrics.reverse_m, 2) }}</td><td>{{ numberText(turn.metrics.energy_proxy, 2) }}</td><td>{{ turn.executable ? '是' : '否' }}</td><td>{{ turn.reason || '方向与净空检查通过' }}</td></tr>
        </tbody></table></div>
      </details>
      <a :href="optimizationReportUrl(run.id)" target="_blank" rel="noopener">查看本轮优化报告与原生转弯曲线 JSON</a>
      </template>
    </template>
  </section>
</template>
<style scoped>
.optimization-results { margin: 16px 0; padding: 16px; background: #f3f8f4; border-radius: 8px; color: #42614c; font-size: 13px; }
.result-metrics { display: grid; grid-template-columns: repeat(4,minmax(0,1fr)); gap: 12px; margin-top: 14px; }
.result-metrics span,.result-metrics small { display: block; color: #6c7e73; }.result-metrics b { display: block; font-size: 20px; margin: 8px 0; }
summary { cursor: pointer; padding: 10px 0; }.table-scroll { overflow-x: auto; max-height: 400px; }
table { border-collapse: collapse; width: 100%; min-width: 740px; }th,td { border: 1px solid #dce7de; padding: 7px; text-align: left; }tr.chosen { background: #d8eddf; }
a { display: inline-block; margin-top: 10px; color: #28734f; }
@media(max-width:900px) { .result-metrics { grid-template-columns: repeat(2,minmax(0,1fr)); } }
</style>
