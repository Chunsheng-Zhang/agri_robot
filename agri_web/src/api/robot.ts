import { apiRequest } from './http'
import type { WorkMode } from './workspace'

export interface PathPoint {
  x: number
  y: number
}

export interface ReferencePathResponse {
  count: number
  points: PathPoint[]
  frame_id?: string
  fingerprint?: string | null
}

export interface RobotStatusResponse {
  robot_id: string
  online: boolean
  mode: string
  speed: number | null
  speed_source?: 'mgx23_ned' | null
  velocity_age_seconds?: number | null
  heading: number | null
  battery: number | null
  battery_voltage?: number | null
  timestamp: string
  last_odom_time?: string | null
  latitude: number | null
  longitude: number | null
  map_position?: MapPosition | null
  position_x: number | null
  position_y: number | null
  angular_speed: number | null
  commanded_speed: number | null
  commanded_angular_speed: number | null
  control_mode: string | null
  work_mode?: WorkMode | null
  ros_connected: boolean
  ros_error: string | null
  odom_age_seconds: number | null
  reference_path_points: number
  tracked_path_points: number
}

export interface MapPosition {
  latitude: number
  longitude: number
  bearing: number | null
  reference: 'base_link'
  source: 'mgx23_localization'
  crs: 'EPSG:4326'
}

export interface Snapshot {
  fresh: boolean
  age_seconds: number | null
}

export interface SafetySnapshot extends Snapshot {
  mode?: WorkMode
  mode_revision?: number
  limits?: { linear: number; angular: number; path_error: number }
  owner?: string | null
  emergency_stop?: boolean | null
  estop_pressed?: boolean
  obstacle_stop?: boolean | null
  motion_inhibited?: boolean
  localization_valid?: boolean
  joy_connected?: boolean
  lidar_connected?: boolean
  lidar_required?: boolean
  lidar_front_only?: boolean
  obstacle_fraction?: number
  reason?: string
}

export interface TaskSnapshot extends Snapshot {
  search_progress?: {
    phase: string; stage_completed: number; stage_total: number
    evaluated_count: number; feasible_count: number; elapsed_seconds: number
    angle_rad?: number; order?: string
  } | null
  phase?: 'IDLE' | 'APPROACHING' | 'ALIGNING' | 'TRACKING'
  alignment_error?: number | null
  alignment_next_phase?: 'APPROACHING' | 'TRACKING' | null
  approach_distance?: number | null
  approach_target?: PathPoint | null
  elapsed_seconds?: number
  state?: string
  reason?: string
  index?: number
  count?: number
  progress?: number
  length?: number
  distance?: number | null
  heading_error?: number | null
  dwelling?: boolean
}

export interface DevicesResponse {
  safety: SafetySnapshot
  obstacles: Snapshot & {
    valid?: boolean
    global_range?: number | null
    front_distance?: number | null
    rear_distance?: number | null
    front_valid?: boolean
    rear_valid?: boolean
    recovery_ready?: boolean
    total_beams?: number | null
    stop_points?: number | null
    stop_fraction?: number | null
    resume_fraction?: number | null
    obstacle_fraction?: number
  }
  manual: Snapshot & { connected?: boolean; profile?: string; linear_gear?: number; angular_gear?: number }
  camera: Snapshot & { connected?: boolean; topic?: string; width?: number; height?: number; encoding?: string; reason?: string }
  waypoint: TaskSnapshot
  tracking: TaskSnapshot
  following: TaskSnapshot
  planning: TaskSnapshot
  gnss: Snapshot & { latitude?: number; longitude?: number; fix_status?: number }
  mgx23?: Snapshot & {
    status_word: number | null
    initialized: boolean | null
    flags: { name: string; label: string; mask: number; active: boolean | null }[]
    gnss_flags?: { name: string; label: string; mask: number; active: boolean | null }[]
  }
  voltage?: Snapshot & { volts?: number }
  scan?: Snapshot & { frame_id: string; points: PathPoint[]; range_max: number | null }
}

export function getRobotStatus(): Promise<RobotStatusResponse> {
  return apiRequest<RobotStatusResponse>('/api/robot/status')
}

export function getReferencePath(source: 'tracking' | 'following' | 'waypoint' = 'tracking'): Promise<ReferencePathResponse> {
  if (source === 'following') return apiRequest('/api/following/path/reference')
  if (source === 'waypoint') return Promise.resolve({ count:0,points:[],frame_id:'map' })
  return apiRequest<ReferencePathResponse>('/api/robot/path/reference')
}

export function getTrackedPath(source: 'tracking' | 'following' | 'waypoint' = 'tracking'): Promise<ReferencePathResponse> {
  return apiRequest(source === 'following' ? '/api/following/path/tracked' : `/api/robot/path/tracked?source=${source}`)
}

export function getWaypoints(): Promise<ReferencePathResponse> {
  return apiRequest('/api/robot/waypoints')
}

export function getDevices(): Promise<DevicesResponse> {
  return apiRequest('/api/devices')
}

export function emergencyStop(active: boolean): Promise<{ success: boolean; message: string }> {
  return apiRequest(active ? '/api/robot/emergency-stop' : '/api/robot/emergency-stop/reset', { method: 'POST' })
}

export function getCameraStream(): Promise<{ stream_url: string }> {
  return apiRequest('/api/camera/stream-info')
}
