import { computed, onMounted, onUnmounted, readonly, shallowRef, ref } from 'vue'
import { getDevices, getRobotStatus } from '../api/robot'
import type { DevicesResponse, RobotStatusResponse, Snapshot } from '../api/robot'

const robot = shallowRef<RobotStatusResponse | null>(null)
const devices = shallowRef<DevicesResponse | null>(null)
const now = ref(Date.now())
const robotReceived = ref(0)
const devicesReceived = ref(0)
const statusError = ref('等待后端数据'), devicesError = ref('')
const error = computed(() => [...new Set([statusError.value, devicesError.value].filter(Boolean))].join('；'))
const statusLoading = ref(false), devicesLoading = ref(false)
const loading = computed(() => statusLoading.value || devicesLoading.value)
let statusRequest: Promise<void> | undefined, devicesRequest: Promise<void> | undefined
let generation = 0
let consumers = 0
let poll: ReturnType<typeof setInterval> | undefined
let clock: ReturnType<typeof setInterval> | undefined

const robotFresh = computed(() => robot.value !== null && now.value - robotReceived.value < 2500)
const devicesFresh = computed(() => devices.value !== null && now.value - devicesReceived.value < 2500)
function isFresh(value?: Snapshot | null, maxAgeMs = 1000) {
  return devicesFresh.value && value?.fresh === true && value.age_seconds !== null
    && value.age_seconds * 1000 + now.value - devicesReceived.value < maxAgeMs
}
const safetyFresh = computed(() => isFresh(devices.value?.safety))
const poseFresh = computed(() => robotFresh.value && robot.value?.ros_connected === true
  && robot.value.online && robot.value.odom_age_seconds !== null
  && robot.value.odom_age_seconds * 1000 + now.value - robotReceived.value < 1000)
const velocityFresh = computed(() => robotFresh.value && robot.value?.ros_connected === true
  && robot.value.speed_source === 'mgx23_ned' && robot.value.velocity_age_seconds != null
  && robot.value.velocity_age_seconds * 1000 + now.value - robotReceived.value < 1500)

function refreshStatus() {
  if (statusRequest) return statusRequest
  const epoch = generation, started = Date.now()
  statusLoading.value = true
  const request = getRobotStatus().then(value => {
    if (epoch !== generation) return
    now.value = Date.now(); robot.value = value; robotReceived.value = started; statusError.value = ''
  }, reason => {
    if (epoch !== generation) return
    robot.value = null; robotReceived.value = 0
    statusError.value = reason instanceof Error ? reason.message : '机器人状态查询失败'
  }).finally(() => {
    if (statusRequest === request) { statusRequest = undefined; statusLoading.value = false }
  })
  statusRequest = request
  return request
}

function refreshDevices() {
  if (devicesRequest) return devicesRequest
  const epoch = generation, started = Date.now()
  devicesLoading.value = true
  const request = getDevices().then(value => {
    if (epoch !== generation) return
    now.value = Date.now(); devices.value = value; devicesReceived.value = started; devicesError.value = ''
  }, reason => {
    if (epoch !== generation) return
    devices.value = null; devicesReceived.value = 0
    devicesError.value = reason instanceof Error ? reason.message : '设备状态查询失败'
  }).finally(() => {
    if (devicesRequest === request) { devicesRequest = undefined; devicesLoading.value = false }
  })
  devicesRequest = request
  return request
}

async function refresh() {
  // Independent single-flight streams: a slow scan/device response must not
  // hold back a healthy robot/ROS response, or queue duplicate requests.
  await Promise.all([refreshStatus(), refreshDevices()])
}

function onVisibility() {
  now.value = Date.now()
  if (document.visibilityState === 'visible') void refresh()
}

export function useRealtime() {
  onMounted(() => {
    consumers += 1
    if (consumers === 1) {
      generation += 1
      void refresh()
      poll = setInterval(() => void refresh(), 500)
      clock = setInterval(() => { now.value = Date.now() }, 250)
      document.addEventListener('visibilitychange', onVisibility)
    }
  })
  onUnmounted(() => {
    consumers -= 1
    if (consumers === 0) {
      generation += 1
      clearInterval(poll)
      clearInterval(clock)
      document.removeEventListener('visibilitychange', onVisibility)
      robot.value = null
      devices.value = null
      robotReceived.value = devicesReceived.value = 0
      statusRequest = devicesRequest = undefined
      statusLoading.value = devicesLoading.value = false
      statusError.value = '等待后端数据'; devicesError.value = ''
    }
  })
  return { robot: readonly(robot), devices: readonly(devices), error: readonly(error),
    loading: readonly(loading), robotFresh, devicesFresh, safetyFresh, poseFresh, velocityFresh, isFresh, refresh }
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
    SUCCEEDED: '规划成功', FAILED: '规划失败', READY: '路径已就绪', WAITING_PATH: '等待参考路径', PREPARING: '准备中', UNKNOWN: '状态未知' } as Record<string, string>)[value ?? ''] ?? (value || '等待节点')
}
