const base = (import.meta.env.VITE_API_BASE_URL ?? '').replace(/\/$/, '')

export function apiUrl(path: string): string {
  return `${base}${path}`
}

export async function apiRequest<T>(path: string, options: RequestInit = {}): Promise<T> {
  const controller = new AbortController()
  const mutation = options.method && options.method !== 'GET'
  const timeout = window.setTimeout(() => controller.abort(), mutation ? 20000 : 2000)
  try {
    const response = await fetch(apiUrl(path), { ...options, signal: controller.signal, cache: 'no-store' })
    const data = await response.json().catch(() => null)
    if (!response.ok) {
      const detail = data?.detail
      throw new Error(typeof detail === 'string' ? detail : `请求失败（HTTP ${response.status}）`)
    }
    if (data === null) throw new Error('后端没有返回有效数据，请检查 API 服务')
    return data as T
  } catch (error) {
    if (error instanceof DOMException && error.name === 'AbortError') {
      throw new Error(mutation ? '请求超时，请核对实际状态后再操作' : '后端响应超时')
    }
    throw error
  } finally {
    window.clearTimeout(timeout)
  }
}
