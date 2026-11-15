import type { PathPoint } from '../../api/robot'

// Viridis-style progression, shared by all pieces in traversal order.
const palette = ['#440154', '#443983', '#31688e', '#21918c', '#35b779', '#90d743', '#fde725']
export function sequenceColor(fraction: number): string {
  const value = Math.max(0, Math.min(1, fraction)) * (palette.length - 1)
  const index = Math.min(palette.length - 2, Math.floor(value)), t = value - index
  const channels = [1, 3, 5].map(offset => {
    const a = parseInt(palette[index]!.slice(offset, offset + 2), 16)
    const b = parseInt(palette[index + 1]!.slice(offset, offset + 2), 16)
    return Math.round(a + (b - a) * t).toString(16).padStart(2, '0')
  })
  return `#${channels.join('')}`
}

export function sequenceBands(points: readonly PathPoint[]) {
  // Remove stationary turn duplicates and same-direction collinear samples;
  // preserve every bend/reversal. Only display geometry, never execution data.
  const vertices: PathPoint[] = []
  for (const p of points) {
    if (!Number.isFinite(p.x) || !Number.isFinite(p.y)) return []
    const previous = vertices.at(-1)
    if (previous && Math.hypot(p.x - previous.x, p.y - previous.y) < 1e-9) continue
    while (vertices.length >= 2) {
      const a = vertices.at(-2)!, b = vertices.at(-1)!
      const ux = b.x - a.x, uy = b.y - a.y, vx = p.x - b.x, vy = p.y - b.y
      if (ux * vx + uy * vy > 0 && Math.abs(ux * vy - uy * vx) <= 1e-7 * Math.hypot(p.x - a.x, p.y - a.y)) vertices.pop()
      else break
    }
    vertices.push(p)
  }
  const size = Math.max(1, Math.ceil((vertices.length - 1) / 96))
  const bands: { points: PathPoint[]; color: string }[] = []
  for (let first = 0; first < vertices.length - 1; first += size) {
    bands.push({ points: vertices.slice(first, Math.min(vertices.length, first + size + 1)),
      color: sequenceColor(first / Math.max(1, vertices.length - 2)) })
  }
  return bands
}
