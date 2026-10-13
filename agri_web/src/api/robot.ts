const API_BASE_URL = 'http://127.0.0.1:8000'

export interface PathPoint {
  x: number
  y: number
}

export interface ReferencePathResponse {
  count: number
  points: PathPoint[]
}

export interface RobotStatusResponse {
  online: boolean
  mode: string
  speed: number | null
  heading: number | null
  battery: number | null
  timestamp: string
}

async function request<T>(url: string): Promise<T> {
  const response = await fetch(url)

  if (!response.ok) {
    throw new Error(`HTTP 错误：${response.status}`)
  }

  return response.json() as Promise<T>
}

export function getRobotStatus(): Promise<RobotStatusResponse> {
  return request<RobotStatusResponse>(
    `${API_BASE_URL}/api/robot/status`,
  )
}

export function getReferencePath(): Promise<ReferencePathResponse> {
  return request<ReferencePathResponse>(
    `${API_BASE_URL}/api/robot/path/reference`,
  )
}
