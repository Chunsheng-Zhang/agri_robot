import { computed, onMounted, onUnmounted, readonly, shallowRef, ref } from 'vue'
import { getDevices, getRobotStatus } from '../api/robot'
import type { DevicesResponse, RobotStatusResponse, Snapshot } from '../api/robot'

const robot = shallowRef<RobotStatusResponse | null>(null)
const devices = shallowRef<DevicesResponse | null>(null)
const now = ref(Date.now())
const robotReceived = ref(0)
const devicesReceived = ref(0)
const error = ref('等待后端数据')
const loading = ref(false)
let consumers = 0
let poll: ReturnType<typeof setInterval> | undefined
let clock: ReturnType<typeof setInterval> | undefined

const robotFresh = computed(() => robot.value !== null && now.value - robotReceived.value < 2500)
const devicesFresh = computed(() => devices.value !== null && now.value - devicesReceived.value < 2500)
const safetyFresh = computed(() => devicesFresh.value && devices.value?.safety.fresh === true)
const poseFresh = computed(() => robotFresh.value && robot.value?.ros_connected === true
  && robot.value.online && robot.value.odom_age_seconds !== null
  && robot.value.odom_age_seconds * 1000 + now.value - robotReceived.value < 1000)

async function refresh() {
  if (loading.value) return
  loading.value = true
  const results = await Promise.allSettled([getRobotStatus(), getDevices()])
  now.value = Date.now()
  const [status, snapshots] = results
  if (status.status === 'fulfilled') {
    robot.value = status.value
    robotReceived.value = now.value
  } else {
    robot.value = null
    robotReceived.value = 0
  }
  if (snapshots.status === 'fulfilled') {
    devices.value = snapshots.value
    devicesReceived.value = now.value
  } else {
    devices.value = null
    devicesReceived.value = 0
  }
  const failure = results.find(result => result.status === 'rejected')
  error.value = failure?.status === 'rejected'
    ? (failure.reason instanceof Error ? failure.reason.message : '后端连接失败') : ''
  loading.value = false
}

function onVisibility() {
  now.value = Date.now()
  if (document.visibilityState === 'visible') void refresh()
}

export function useRealtime() {
  onMounted(() => {
    consumers += 1
    if (consumers === 1) {
      void refresh()
      poll = setInterval(() => void refresh(), 1000)
      clock = setInterval(() => { now.value = Date.now() }, 250)
      document.addEventListener('visibilitychange', onVisibility)
    }
  })
  onUnmounted(() => {
    consumers -= 1
    if (consumers === 0) {
      clearInterval(poll)
      clearInterval(clock)
      document.removeEventListener('visibilitychange', onVisibility)
      robot.value = null
      devices.value = null
    }
  })
  const isFresh = (value?: Snapshot | null) => devicesFresh.value && value?.fresh === true
  return { robot: readonly(robot), devices: readonly(devices), error: readonly(error),
    loading: readonly(loading), robotFresh, devicesFresh, safetyFresh, poseFresh, isFresh, refresh }
}

export function numberText(value: number | null | undefined, digits = 2): string {
  return typeof value === 'number' && Number.isFinite(value) ? value.toFixed(digits) : '--'
}

export function modeText(value: string | null | undefined): string {
  return ({ manual: '手柄遥控', waypoint: '多点导航', tracking: '路径跟踪', web: 'Web 遥控' } as Record<string, string>)[value ?? '']
    ?? (value ? value : '无控制任务')
}

export function taskText(value?: string): string {
  return ({ IDLE: '待命', ACQUIRING: '申请控制权', RUNNING: '运行中', PAUSED: '已暂停',
    COMPLETED: '已完成', CANCELED: '已取消', FAULT: '故障停止', PLANNING: '规划中',
    SUCCEEDED: '规划成功', FAILED: '规划失败' } as Record<string, string>)[value ?? ''] ?? (value || '等待节点')
}
