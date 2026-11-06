export const manualTiming = Object.freeze({ acknowledgement: 500, heartbeat: 80, lifetime: 0.2, maxGap: 200 })
export interface ManualFrame { token: string; seq: number; linear: number; angular: number; expires_at: number }
export interface ManualAck { success: true; seq: number; applied?: boolean; reason?: string; overdue_ms?: number }

// One ordered connection, bounded outstanding frames, no reconnect or retries.
// Acknowledgements are cumulative; they never refresh a command's expiry.
export class ManualChannel {
  private socket: WebSocket
  private closed = false
  private opened = false
  private lastSent = 0
  private lastAcknowledged = 0
  private openingTimer: ReturnType<typeof setTimeout>
  private opening: Promise<void>
  private rejectOpening!: (reason: Error) => void
  private pending = new Map<number, { resolve: (reply: ManualAck) => void; reject: (reason: Error) => void; timer: ReturnType<typeof setTimeout> }>()
  onFailure: (reason: Error) => void = () => {}

  constructor(address: string) {
    const url = new URL(address, window.location.href)
    if (!['http:', 'https:', 'ws:', 'wss:'].includes(url.protocol)) throw new Error('遥控连接地址无效')
    url.protocol = ['https:', 'wss:'].includes(url.protocol) ? 'wss:' : 'ws:'
    this.socket = new WebSocket(url)
    this.opening = new Promise<void>((resolve, reject) => {
      this.rejectOpening = reject
      this.socket.onopen = () => {
        if (this.closed) return
        clearTimeout(this.openingTimer); this.opened = true; resolve()
      }
    })
    this.openingTimer = setTimeout(() => this.fail(new Error('遥控连接建立超过 500 毫秒，请检查网络')), manualTiming.acknowledgement)
    this.socket.onmessage = event => this.acknowledge(event.data)
    this.socket.onerror = () => this.fail(new Error('遥控连接异常，已退出遥控'))
    this.socket.onclose = () => this.fail(new Error('遥控连接已断开，请重新启用'))
  }

  open() { return this.opening }

  send(frame: ManualFrame): Promise<ManualAck> {
    if (this.closed || !this.opened || this.socket.readyState !== WebSocket.OPEN) {
      return Promise.reject(new Error('遥控连接不可用，请重新启用'))
    }
    if (frame.seq <= this.lastSent || this.pending.size >= 8 || this.socket.bufferedAmount > 2048) {
      const error = new Error('遥控发送积压或顺序异常，已退出遥控')
      this.fail(error); return Promise.reject(error)
    }
    this.lastSent = frame.seq
    return new Promise<ManualAck>((resolve, reject) => {
      const timer = setTimeout(() => this.fail(new Error('遥控应答超过 500 毫秒，已退出遥控')), manualTiming.acknowledgement)
      this.pending.set(frame.seq, { resolve, reject, timer })
      try { this.socket.send(JSON.stringify(frame)) }
      catch { this.fail(new Error('遥控指令发送失败，已退出遥控')) }
    })
  }

  private acknowledge(data: unknown) {
    if (this.closed) return
    try {
      if (typeof data !== 'string') throw new Error()
      const reply = JSON.parse(data)
      if (reply.success !== true) {
        this.fail(new Error(typeof reply.detail === 'string' ? reply.detail : '遥控指令被拒绝'))
        return
      }
      if (!Number.isSafeInteger(reply.seq) || reply.seq > this.lastSent || reply.seq < 1) throw new Error()
      if (reply.seq <= this.lastAcknowledged) return
      if (!this.pending.has(reply.seq)) throw new Error()
      this.lastAcknowledged = reply.seq
      for (const [seq, item] of this.pending) {
        if (seq <= reply.seq) { clearTimeout(item.timer); this.pending.delete(seq); item.resolve(reply) }
      }
    } catch { this.fail(new Error('遥控应答格式异常，已退出遥控')) }
  }

  private fail(error: Error) {
    if (this.closed) return
    this.close(error); this.onFailure(error)
  }

  close(reason = new Error('遥控会话已结束')) {
    if (this.closed) return
    this.closed = true
    clearTimeout(this.openingTimer); this.rejectOpening(reason)
    for (const item of this.pending.values()) { clearTimeout(item.timer); item.reject(reason) }
    this.pending.clear()
    this.socket.onopen = this.socket.onmessage = this.socket.onerror = this.socket.onclose = null
    try { this.socket.close() } catch { /* The backend independently expires commands. */ }
  }
}
