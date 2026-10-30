import type { GeoPoint } from '../../api/workspace'

// Shared initial waypoints and field corners, in the supplied boundary order.
export const defaultTaskPoints: readonly GeoPoint[] = [
  { latitude: 36.6468527633281, longitude: 117.021433066391, dwell_time: 2 },
  { latitude: 36.6467053079008, longitude: 117.021432014200, dwell_time: 2 },
  { latitude: 36.6467040785547, longitude: 117.021575725468, dwell_time: 2 },
  { latitude: 36.6468038113836, longitude: 117.021555217921, dwell_time: 2 },
  { latitude: 36.6468514201155, longitude: 117.021555675521, dwell_time: 2 },
]
