import { apiRequest } from './http'
import type { Projection, Run } from './workspace'

export interface TeachPoint { x: number; y: number; yaw?: number; stamp?: number }
export interface ProcessingParameters { allow_reverse:boolean; mode?:'original_points'; spacing?:number; min_spacing?:number; max_speed?:number; max_gap?:number; smoothing?:number }
export interface FollowingParameters { controller_algorithm:'pp'|'pp_speed'|'stanley'|'lqr'|'mpc'; lookahead:number; speed:number; max_angular:number; completion_tolerance:number; start_tolerance:number; search_window:number; max_path_error:number }
export interface TaughtPath {
  id:string; name:string; state:string; reason:string; created_at:string; projection:Projection
  processing:ProcessingParameters; fingerprint?:string; reference_fingerprint?:string; points?:TeachPoint[]
  source_kind?:'recording'|'import'
  import_source?:{filename:string;coordinate_system:string;yaw_supplied:boolean;timing_available:boolean;note:string}
  statistics?:{ raw_samples:number; retained_samples:number; path_points:number; length_m:number; point_turn_count:number; reverse_segments:number }
}
export interface FollowingContext {
  recording:{ state:string; reason:string; id:string|null; samples:number; elapsed_seconds:number }
  paths:TaughtPath[]; loaded_id:string|null; reference:TeachPoint[]; raw:TeachPoint[]; track:TeachPoint[]; pose:TeachPoint|null
  error:{lateral:number|null;heading:number;progress:number}|null; active_run:Run|null
  status:{controller_algorithm?:'pp'|'pp_speed'|'stanley'|'lqr'|'mpc';controller_label?:string;state?:string;reason?:string;phase?:string;fresh:boolean;age_seconds:number|null;progress?:number;length?:number;reference_fingerprint?:string;lateral_error?:number|null;heading_error?:number|null}
}
export const getFollowing = () => apiRequest<FollowingContext>('/api/following/context')
export const getTaughtPath = (id:string) => apiRequest<TaughtPath>(`/api/following/paths/${id}`)
export const importFollowingFile = (body:{name:string;filename:string;content:string;processing:ProcessingParameters}) =>
  apiRequest<TaughtPath>('/api/following/import', {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)})
export function followingRequest(path:string, body?:object, method='POST') {
  return apiRequest<{success?:boolean;id?:string;run_id?:string}>(`/api/following/${path}`, {
    method, headers:{'Content-Type':'application/json'}, ...(body ? {body:JSON.stringify(body)} : {}),
  })
}
