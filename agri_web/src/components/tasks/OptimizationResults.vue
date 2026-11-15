<script setup lang="ts">
import { computed } from 'vue'
import type { Run, PlanningBasis } from '../../api/workspace'
import { optimizationReportUrl } from '../../api/workspace'
import { numberText } from '../../composables/useRealtime'
const props = defineProps<{ run: Run }>()
const result = computed(() => props.run.optimization)
const selected = computed(() => result.value?.selected)
const pointTurn = computed(() => result.value?.turn_mode === 'POINT_TURN')
const nativeTsp = computed(() => result.value?.policy === 'POINT_TURN_NATIVE_TSP')
const fourObjectives = computed(() => result.value?.weights?.turns != null)
function basisName(basis?: PlanningBasis): string {
  if (!basis) return '历史布局'
  if (basis.kind === 'EDGE') return `${basis.ring === 'HOLE' ? '禁入区 ' + basis.ring_index : '外边界'} · 边 ${(basis.edge_index ?? 0) + 1}`
  return ({PCA:'PCA补充方向',COARSE:'全局粗搜方向',REFINE:'细化方向',NATIVE_WHOLE_FIELD:'原生整田不分解',NATIVE_PER_CELL:'原生分区独立方向'} as Record<string,string>)[basis.kind] || basis.kind
}
function angleText(candidate: NonNullable<typeof selected.value>): string {
  return candidate.direction_mode === 'PER_CELL' ? '各分区独立' : numberText(candidate.angle_rad * 180 / Math.PI, 2) + '°'
}
const names: Record<string,string> = {TSP:'原生TSP',BOUSTROPHEDON:'牛耕',SNAKE:'蛇形',SPIRAL:'螺旋',CONTOUR:'回形',DUBINS:'Dubins',DUBINS_CC:'Dubins CC',REEDS_SHEPP:'Reeds–Shepp',REEDS_SHEPP_HC:'Reeds–Shepp HC'}
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
          <div><span>推荐作业角度</span><b>{{ angleText(selected) }}</b></div>
          <div><span>最小安全地头</span><b>{{ numberText(selected.headland_width_m, 3) }} m</b></div>
          <div><span>{{ run.entry ? '计分长度（含入口）' : '路径总长度' }}</span><b>{{ numberText(selected.length_m, 2) }} m</b></div>
          <div><span>原地转向点</span><b>{{ selected.point_turn_count ?? 0 }}</b></div>
        </div>
        <p v-if="selected"><template v-if="nativeTsp">Fields2Cover 原生TSP</template><template v-else>往复式{{ selected.order === 'LOCAL_2OPT' ? (selected.cell_count ? '（分区顺序已局部优化）' : '（行顺序已局部优化）') : '（基础行顺序）' }}</template> · 覆盖 {{ numberText(selected.coverage_percent, 2) }}% · 重复覆盖 {{ numberText(selected.overlap_percent, 2) }}% · 综合分 {{ numberText(selected.score, 4) }}（越低越好）。</p>
        <p v-if="selected?.cell_count">Fields2Cover 分区 {{ selected.cell_count }} 个，{{ nativeTsp ? '允许跨分区安排作业行' : '各分区连续往复作业' }}；作业行 {{ numberText(selected.swath_length_m, 2) }} m，换行与换区连接 {{ numberText(selected.transition_length_m, 2) }} m。{{ nativeTsp ? '完整保留原生Route连接。' : selected.connection_policy === 'CELL_BOUNDARIES' ? '连接沿分区边界通道行驶。' : '相邻作业行直接连接。' }}</p>
        <p v-if="selected?.layout_family === 'NATIVE_WHOLE_FIELD'">选定布局：原生整田不分解；保留真实凹边界与禁入区，直接生成作业行并比较完整路径。</p>
        <p v-if="selected?.layout_family === 'NATIVE_PER_CELL'">选定布局：原生分区独立方向；各区分别搜索作业方向，再统一安排全部作业行。分区切线不额外预留地头，完整车辆净空检查保留。</p>
        <details v-if="selected?.direction_mode === 'PER_CELL' && selected.cell_directions?.length"><summary>各分区作业方向</summary>
          <p v-for="cell in selected.cell_directions" :key="cell.cell_id">分区 {{ cell.cell_id + 1 }}：{{ numberText(cell.angle_rad * 180 / Math.PI, 2) }}°，{{ cell.swath_count }} 条作业行。</p>
        </details>
        <p v-if="selected && run.entry">正式路径 {{ numberText(selected.working_length_m,2) }} m ＋ 入口参考距离 {{ numberText(selected.entry_length_m,2) }} m；{{ nativeTsp ? (selected.variant < 2 ? '原生自由路线' : '入口引导路线')+(selected.variant%2 === 0 ? '起端' : '另一端') : '起步方案 '+(selected.variant+1) }}。入口连接不计作业覆盖，田外示意通道未验证。</p>
        <p v-if="nativeTsp">TSP在全部作业行间优化访问顺序和方向，不要求分区连续完成。每次原生路由调用使用库默认 {{ selected?.native_tsp?.time_limit_seconds ?? 1 }} 秒启发式求解；不强制返回起点，不保证最短路由已被证明。</p>
        <p v-if="selected?.row_layout?.policy === 'EDGE_ALIGNED_FIXED_SPACING'">靠边排线：首行在安全允许时靠近原边界半幅宽，中间保持作业幅宽间距，末侧有余量时补行。作业方向继续自动搜索，行端原地转向净空保持不变。</p>
        <p v-if="selected?.row_layout?.policy === 'EDGE_ANCHORED_FIXED_SPACING'">沿边候选：{{ basisName(selected.basis) }}；中间保持作业幅宽间距，排线铺满整个地块。{{ selected.row_layout.last_side_row_added ? '本方案采用补边布局。' : '本方案采用原布局，不追加补边行。' }}方向与起排位置绑定，完整转向净空保持不变。</p>
        <p v-if="selected?.start && selected?.end">绿色起点 map（{{ numberText(selected.start.x,2) }}，{{ numberText(selected.start.y,2) }}），红色终点 map（{{ numberText(selected.end.x,2) }}，{{ numberText(selected.end.y,2) }}），单位 m。</p>
        <p>本轮权重：覆盖率 {{ result.weights?.coverage ?? '--' }}、重复覆盖率 {{ result.weights?.overlap ?? '--' }}、总长度 {{ result.weights?.length ?? '--' }}<template v-if="fourObjectives">、转弯次数 {{ result.weights.turns }}</template>；在同一整田评价网格比较，机具有效幅宽保持不变。长度采用 L/(L+A/W) 归一化，含全部换行连接；不评价能耗或时间。</p>
        <p v-if="fourObjectives">转弯次数按完整执行路径的停车转向标记统计，包含换行及绕障拐角，不含接近和首次对齐。采用 N/(N+N0) 归一化，N0=max(1,2√A/W)，本轮 N0={{ numberText(result.normalization?.turn_reference_count,2) }}；权重是本项目初始工程取值。</p>
        <p>车辆包络：长 {{ numberText(result.headland_search?.robot_length_m, 3) }} m、宽 {{ numberText(result.headland_search?.robot_width_m, 3) }} m、安全余量 {{ numberText(result.headland_search?.safety_margin_m, 3) }} m。转向点停车等待、原地对齐后继续，默认尺寸须实测核对。</p>
        <p v-if="result.layout_search">本轮自动比较：沿边统一方向、原生整田不分解、原生分区独立方向；完全相同的同类作业图去重，不因重复计算增加搜索范围。</p>
        <p>已比较 {{ result.evaluated_count }} 个候选 · 用时 {{ numberText(result.elapsed_seconds, 2) }} s · {{ result.state === 'FAILED' ? '没有可执行方案' : result.search_complete ? (nativeTsp ? (result.layout_search ? '已完成全部预定布局及方向的原生TSP求解和评价' : '已完成全部预定方向的原生TSP求解和评价') : '已完成全部预定角度及行顺序比较') : result.budget_exhausted ? '历史结果：达到计算预算' : '完成本轮角度搜索' }}；不保证全局最优。实际评价网格 {{ numberText(result.final_resolution_m, 3) }} m。</p>
        <details><summary>角度、排线与行顺序候选对比</summary>
          <p v-if="result.candidates_truncated">此处展示前128条及选定候选；全部候选均已参与比较，完整数据见下方报告 JSON。</p>
          <div class="table-scroll"><table>
            <thead><tr><th>编号</th><th>角度 °</th><th v-if="fourObjectives">基准／补行</th><th v-if="run.entry">起步方案</th><th>行顺序</th><th>覆盖 %</th><th>重复 %</th><th>{{ run.entry ? '计分长度 m' : '总长度 m' }}</th><th v-if="fourObjectives">转弯次数</th><th>综合评分</th><th>状态</th></tr></thead>
            <tbody><tr v-for="candidate in result.candidates" :key="candidate.id" :class="{ chosen: candidate.id === selected?.id }">
              <td>{{ candidate.id }}</td><td>{{ candidate.direction_mode === 'PER_CELL' ? '分区独立' : numberText(candidate.angle_rad * 180 / Math.PI, 2) }}</td>
              <td v-if="fourObjectives">{{ basisName(candidate.basis) }} · {{ candidate.row_layout?.last_side_row_added ? '补边' : '原布局' }}</td>
              <td v-if="run.entry">{{ candidate.variant+1 }}</td>
              <td>{{ candidate.cell_count ? candidate.cell_count + ' 分区 · ' : '' }}{{ candidate.order === 'F2C_TSP' ? '原生TSP' : candidate.order === 'LOCAL_2OPT' ? '局部优化' : '往复式' }}</td>
              <td>{{ numberText(candidate.coverage_percent, 2) }}</td><td>{{ numberText(candidate.overlap_percent, 2) }}</td>
              <td>{{ numberText(candidate.length_m, 2) }}</td><td v-if="fourObjectives">{{ candidate.point_turn_count ?? '--' }}</td><td>{{ numberText(candidate.score, 4) }}</td>
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
