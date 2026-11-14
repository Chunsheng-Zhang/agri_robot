import { apiRequest, apiUrl } from './http'
import type { PathPoint, ReferencePathResponse } from './robot'

export const workModes = ['base', 'waypoint', 'coverage', 'following'] as const
export type WorkMode = typeof workModes[number]
export type MissionMode = 'waypoint' | 'coverage'
export interface GeoPoint { longitude: number; latitude: number; dwell_time: number }
export interface Projection { epsg: number; latitude: number; longitude: number }
export interface FieldEntry { longitude:number; latitude:number; source:'coordinates'|'map' }
export interface EntryPlan {
  reference: FieldEntry & PathPoint; anchor:PathPoint; coverage_start:PathPoint; connection:readonly PathPoint[]
  distance_m:number; connection_length_m:number; reference_to_anchor_m:number; access_verified:boolean; note:string
}
export interface Preview {
  frame_id: string; points: PathPoint[]; rings: PathPoint[][]; area: number | null; length: number
  inset_polygons?: PathPoint[][][]; inset_distance_m?: number
  entry?: (FieldEntry & PathPoint) | null; entry_anchor?: PathPoint | null
}
export interface MissionInput {
  name: string; mode: MissionMode; waypoints: GeoPoint[]; boundary: object | null
  parameters: Record<string, number | string | boolean>; revision?: number; import_origin?: Projection
  entry?: FieldEntry | null
}
export interface MissionDocument extends MissionInput {
  id: string; revision: number; projection: Projection; preview: Preview; created_at: string; updated_at: string
  coverage_plans?: SavedPlan[]
}
export interface SavedPlan {
  id: string; mission_id: string; mission_revision: number; created_at: string
  planning_strategy?: 'MANUAL' | 'SMART' | 'REUSED'; reference_fingerprint?: string
}
export interface SavedPlanPath extends ReferencePathResponse {
  run: Run; projection: Projection; preview: Preview
}
export interface Run {
  id: string; mission_id: string; mission_revision: number; name: string; mode: MissionMode | 'following'
  stage: string; state: string; reason: string; created_at: string; updated_at: string
  coverage_quality_version?: number
  coverage_quality?: CoverageQuality | null
  projection?: Projection
  reference_fingerprint?: string
  planning_strategy?: 'MANUAL' | 'SMART' | 'REUSED'
  source_plan_id?: string
  saved_plan_version?: number
  plan_save_required?: boolean
  plan_saved_at?: string
  entry?: EntryPlan | null
  optimization?: OptimizationResult | null
}
export interface OptimizationCandidate {
  id: number; mode: string; headland_width_m: number; angle_rad: number; variant: number; state: string
  coverage_percent?: number; overlap_percent?: number; fine_coverage_percent?: number; fine_overlap_percent?: number
  length_m?: number; energy_proxy?: number; score?: number; reason?: string; executable?: boolean
  working_length_m?: number; entry_length_m?: number
  motion?: {forward_m:number;reverse_m:number;turning_m:number;switches:number;energy_proxy:number}
  headland_search_phase?: 'COARSE' | 'REFINE'
  order?: string; swath_count?: number; point_turn_count?: number
}
export interface OptimizationResult {
  schema_version?: number; policy?: string; turn_mode?: string; coverage_objective_enabled?: boolean
  state: string; reason?: string; selected?: OptimizationCandidate; coverage_constraint_enabled?: boolean
  candidates: readonly OptimizationCandidate[]; evaluated_count:number; possible_count:number; budget_exhausted?:boolean
  search_complete?:boolean; candidate_limit_enabled?:boolean; search_time_limit_enabled?:boolean
  search_resolution_m:number; final_resolution_m:number; elapsed_seconds:number; selection_scope:string
  weights: Record<string,number>; energy_model?:{calibrated:boolean;unit:string}
  curve_type?: string; reverse_execution_supported?: boolean; allow_reverse?: boolean
  headland_search?: {schema_version:number;method:string;range_min_m:number;range_max_m:number;
    selected_width_m?:number;initial_widths_m:readonly number[];refinement_widths_m:readonly number[];
    coarse_evaluated_count?:number;refinement_evaluated_count?:number;width_resolution_m?:number;limitations?:string;
    robot_length_m?:number;robot_width_m?:number;safety_margin_m?:number;rotation_radius_m?:number}
  turn_curves?:readonly {type:string;radius_m:number;metrics:{length_m:number;energy_proxy:number;reverse_m:number;switches:number};clearance_valid:boolean;executable:boolean;reason:string}[]
}
export type CoverageMapSource = 'planned' | 'actual'
export type CoverageMapLayer = 'completed' | 'remaining' | 'overlap'
export interface CoverageMap {
  state: string; run_id: string; reason?: string
  mission_id?: string; mission_revision?: number; frame_id?: string; projection?: Projection
  reference_fingerprint?: string; generated_at?: string; snapshot_age_seconds?: number
  width_m?: number; resolution_m?: number; extent?: number[]
  inset_polygons?: PathPoint[][][]; inset_distance_m?: number
  field_rings?: PathPoint[][]
  sources?: Record<CoverageMapSource, {
    version: string; status: string; samples?: number; last_stamp?: number | null
    splits?: number; invalid_rows?: number; track_lines?: PathPoint[][]
  }>
}
export interface CoverageAreaMetrics {
  available: boolean
  area_m2: number
  covered_m2?: number
  missed_m2?: number
  overlap_m2?: number
  coverage_percent: number | null
  overlap_percent: number | null
  miss_percent: number | null
  extra_pass_percent?: number
}
export interface CoverageEstimate {
  available: boolean
  reason?: string
  width_m?: number
  resolution_m?: number
  metrics?: Record<'field' | 'interior' | 'headland', CoverageAreaMetrics>
  sensitivity?: { coverage_difference_pp: number; overlap_difference_pp: number }
}
export interface CoverageQuality {
  state: string
  reason?: string
  actual_error?: string
  planned?: CoverageEstimate | null
  actual?: CoverageEstimate | null
}
export interface WorkspaceContext {
  teaching?: {state:string; reason:string; id:string|null; samples:number}
  synthetic?: boolean
  mode: WorkMode | null; origin: Projection; origin_source: string; config_file: string
  missions: MissionDocument[]; runs: Run[]; active_run: Run | null
  defaults: Record<MissionMode, Record<string, number | string | boolean>>
  events: { time?: string; stamp?: number; event: string; reason?: string; mode?: string; channel: string; mission_id?: string }[]
}
export const modeLabels: Record<WorkMode, string> = { base: '基础待命', waypoint: '多点导航', coverage: '全覆盖规划与跟踪', following: '路径跟随' }
export function workModeText(mode?: string | null): string {
  return workModes.includes(mode as WorkMode) ? modeLabels[mode as WorkMode] : '运行模式未更新'
}
export const request = <T>(path: string, data?: unknown, method = 'POST') => apiRequest<T>(path, {
  method, headers: { 'Content-Type': 'application/json' }, ...(data === undefined ? {} : { body: JSON.stringify(data) }),
})
export const getWorkspace = () => apiRequest<WorkspaceContext>('/api/workspace')
export const selectMode = (mode: WorkMode) => request('/api/system/mode', { mode })
export const mapToWgs84 = (x:number,y:number,projection:Projection) =>
  apiRequest<Pick<FieldEntry,'latitude'|'longitude'> & PathPoint & {projection:Projection}>('/api/coordinates/map-to-wgs84',{
    method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({x,y,projection})})
export const saveMission = (mission: MissionInput, id?: string) => request<MissionDocument>(
  id ? `/api/missions/${id}` : '/api/missions', mission, id ? 'PUT' : 'POST')
export const deleteMission = (id: string) => request(`/api/missions/${id}`, undefined, 'DELETE')
export const previewMission = (mission: MissionInput) => request<Preview>('/api/missions/preview', mission)
export const missionAction = (id: string, action: 'plan' | 'start' | 'optimize') => request(`/api/missions/${id}/${action}`)
export const getSavedPlanPath = (id: string, plan: string) => apiRequest<SavedPlanPath>(`/api/missions/${id}/plans/${plan}/path`)
export const loadSavedPlan = (id: string, plan: string) => request(`/api/missions/${id}/plans/${plan}/load`)
export const savePlannedPath = (id: string, plan: string) => request<{ success: boolean; already_saved: boolean; run: Run }>(`/api/missions/${id}/plans/${plan}/save`)
export const optimizationReportUrl = (id: string) => apiUrl(`/api/runs/${id}/optimization`)
export const taskAction = (source: 'waypoint' | 'tracking' | 'following', action: 'pause' | 'resume' | 'cancel') => request(`/api/tasks/${source}/${action}`)
export const cancelPlanning = () => request('/api/planning/cancel')
export const resetObstacle = () => request('/api/robot/obstacle-stop/reset')
export const setLidarProtection = (enabled: boolean) => request<{ success: boolean; enabled: boolean; message: string }>(
  '/api/robot/lidar-protection', { enabled })
export const runDownloadUrl = (id: string) => apiUrl(`/api/runs/${id}/download`)
export const coverageQualityUrl = (id: string, artifact: 'plan.png' | 'actual.png' | 'report.md' | 'metrics.json') =>
  apiUrl(`/api/runs/${id}/coverage-quality/${artifact}`)
export const getCoverageMap = (id: string) => apiRequest<CoverageMap>(`/api/runs/${id}/coverage-map`)
export const coverageMapImageUrl = (id: string, source: CoverageMapSource, layer: CoverageMapLayer, version: string) =>
  apiUrl(`/api/runs/${id}/coverage-map/${source}/${layer}.png?v=${version}`)
export const deleteRun = (id: string) => request<{ success: boolean; data_preserved: boolean }>(`/api/runs/${id}`, undefined, 'DELETE')
export const armManual = () => request<{ token: string; server_time: number; heartbeat_protocol: number; limits: { linear: number; angular: number } }>('/api/control/arm')
export const stopManual = (token: string) => request('/api/control/stop', { token })
export function downloadJson(name: string, data: unknown) {
  const url = URL.createObjectURL(new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' }))
  const anchor = document.createElement('a'); anchor.href = url; anchor.download = name; anchor.click()
  setTimeout(() => URL.revokeObjectURL(url), 1000)
}
