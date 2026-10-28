import { apiRequest } from './http'

export interface PathPoint {
  x: number
  y: number
}

export interface ReferencePathResponse {
  count: number
  points: PathPoint[]
  frame_id?: string
}

export interface RobotStatusResponse {
  robot_id: string
  online: boolean
  mode: string
  speed: number | null
  heading: number | null
  battery: number | null
  timestamp: string
  last_odom_time?: string | null
  latitude: number | null
  longitude: number | null
  position_x: number | null
  position_y: number | null
  angular_speed: number | null
  commanded_speed: number | null
  commanded_angular_speed: number | null
  control_mode: string | null
  ros_connected: boolean
  ros_error: string | null
  odom_age_seconds: number | null
  reference_path_points: number
  tracked_path_points: number
}

export interface Snapshot {
  fresh: boolean
  age_seconds: number | null
}

export interface SafetySnapshot extends Snapshot {
  owner?: string | null
  emergency_stop?: boolean | null
  estop_pressed?: boolean
  obstacle_stop?: boolean | null
  motion_inhibited?: boolean
  localization_valid?: boolean
  joy_connected?: boolean
  lidar_connected?: boolean
  reason?: string
}

export interface TaskSnapshot extends Snapshot {
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
  }
  manual: Snapshot & { connected?: boolean; profile?: string; linear_gear?: number; angular_gear?: number }
  camera: Snapshot & { connected?: boolean; topic?: string; width?: number; height?: number; encoding?: string; reason?: string }
  waypoint: TaskSnapshot
  tracking: TaskSnapshot
  planning: TaskSnapshot
  gnss: Snapshot & { latitude?: number; longitude?: number; fix_status?: number }
}

export function getRobotStatus(): Promise<RobotStatusResponse> {
  return apiRequest<RobotStatusResponse>('/api/robot/status')
}

export function getReferencePath(): Promise<ReferencePathResponse> {
  return apiRequest<ReferencePathResponse>('/api/robot/path/reference')
}

export function getTrackedPath(): Promise<ReferencePathResponse> {
  return apiRequest('/api/robot/path/tracked')
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
