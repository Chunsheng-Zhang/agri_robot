import { apiUrl } from './http'

export interface DiagnosticEntry {
  id: number; unix_ns: number; level?: string; source: string; message?: string; channel?: string
  direction?: 'TX' | 'RX'; sequence?: number; frame?: string; bytes?: number
  function?: number; function_name?: string; summary?: string; checksum_valid?: boolean
}
export interface DiagnosticPage {
  session: string; started_unix_ns: number; entries: DiagnosticEntry[]; cursor: number
  reset: boolean; skipped: number; archive_directory: string | null
  archive_dropped: number; archive_error: string
  status?: {
    observer_online: boolean; rx_fresh: boolean; port?: string; session?: string; status_age_seconds?: number | null
    tx_frames?: number; rx_frames?: number; observer_dropped?: number; sequence_gaps: number
    bad_frames: number; malformed_batches: number; error?: string
    observed_tx_age_seconds?: number | null; observed_rx_age_seconds?: number | null
  }
}
export async function getDiagnosticPage(kind: 'logs' | 'serial', after: number, session: string, signal: AbortSignal): Promise<DiagnosticPage> {
  const query = new URLSearchParams({ after: String(after), limit: '200' })
  if (session) query.set('session', session)
  const response = await fetch(apiUrl(`/api/diagnostics/${kind}?${query}`), { signal, cache: 'no-store' })
  if (!response.ok) throw new Error(`诊断接口未更新（HTTP ${response.status}），请核对后端版本与连接`)
  return response.json() as Promise<DiagnosticPage>
}
