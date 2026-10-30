import { apiRequest, apiUrl } from './http'
import type { PathPoint } from './robot'

export type WorkMode = 'base' | 'waypoint' | 'coverage'
export type MissionMode = 'waypoint' | 'coverage'
export interface GeoPoint { longitude: number; latitude: number; dwell_time: number }
export interface Projection { epsg: number; latitude: number; longitude: number }
export interface Preview { frame_id: string; points: PathPoint[]; rings: PathPoint[][]; area: number | null; length: number }
export interface MissionInput {
  name: string; mode: MissionMode; waypoints: GeoPoint[]; boundary: object | null
  parameters: Record<string, number | string>; revision?: number; import_origin?: Projection
}
export interface MissionDocument extends MissionInput {
  id: string; revision: number; projection: Projection; preview: Preview; created_at: string; updated_at: string
}
export interface Run {
  id: string; mission_id: string; mission_revision: number; name: string; mode: MissionMode
  stage: string; state: string; reason: string; created_at: string; updated_at: string
}
export interface WorkspaceContext {
  synthetic?: boolean
  mode: WorkMode | null; origin: Projection; origin_source: string; config_file: string
  missions: MissionDocument[]; runs: Run[]; active_run: Run | null
  defaults: Record<MissionMode, Record<string, number | string>>
  events: { time?: string; stamp?: number; event: string; reason?: string; mode?: string; channel: string; mission_id?: string }[]
}
export const modeLabels: Record<WorkMode, string> = { base: '基础待命', waypoint: '多点导航', coverage: '全覆盖作业' }
export const request = <T>(path: string, data?: unknown, method = 'POST') => apiRequest<T>(path, {
  method, headers: { 'Content-Type': 'application/json' }, ...(data === undefined ? {} : { body: JSON.stringify(data) }),
})
export const getWorkspace = () => apiRequest<WorkspaceContext>('/api/workspace')
export const selectMode = (mode: WorkMode) => request('/api/system/mode', { mode })
export const saveMission = (mission: MissionInput, id?: string) => request<MissionDocument>(
  id ? `/api/missions/${id}` : '/api/missions', mission, id ? 'PUT' : 'POST')
export const deleteMission = (id: string) => request(`/api/missions/${id}`, undefined, 'DELETE')
export const previewMission = (mission: MissionInput) => request<Preview>('/api/missions/preview', mission)
export const missionAction = (id: string, action: 'plan' | 'start') => request(`/api/missions/${id}/${action}`)
export const taskAction = (source: 'waypoint' | 'tracking', action: 'pause' | 'resume' | 'cancel') => request(`/api/tasks/${source}/${action}`)
export const cancelPlanning = () => request('/api/planning/cancel')
export const resetObstacle = () => request('/api/robot/obstacle-stop/reset')
export const setLidarProtection = (enabled: boolean) => request<{ success: boolean; enabled: boolean; message: string }>(
  '/api/robot/lidar-protection', { enabled })
export const runDownloadUrl = (id: string) => apiUrl(`/api/runs/${id}/download`)
export const armManual = () => request<{ token: string; server_time: number; limits: { linear: number; angular: number } }>('/api/control/arm')
export const stopManual = (token: string) => request('/api/control/stop', { token })
export async function manualCommand(data: { token: string; seq: number; linear: number; angular: number; expires_at: number }) {
  const controller = new AbortController()
  const timer = setTimeout(() => controller.abort(), 180)
  try {
    const response = await fetch(apiUrl('/api/control/command'), {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(data), signal: controller.signal,
    })
    if (!response.ok) throw new Error((await response.json()).detail || '遥控指令被拒绝')
  } catch (error) {
    if (controller.signal.aborted) {
      throw new Error('遥控通信超过 180 毫秒，请检查网络或使用现场手柄')
    }
    throw error
  } finally { clearTimeout(timer) }
}
export function downloadJson(name: string, data: unknown) {
  const url = URL.createObjectURL(new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' }))
  const anchor = document.createElement('a'); anchor.href = url; anchor.download = name; anchor.click()
  setTimeout(() => URL.revokeObjectURL(url), 1000)
}
