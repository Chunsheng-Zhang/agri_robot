"""Bounded planar swept-footprint evaluation; independent of ROS and motion control.

Each cell represents its clipped field area. A visit is a connected interval of
arc length during which the moving disk footprint contains the sample point.
Adjacent segments and repeated stationary samples do not create extra visits.
"""
import math

import numpy as np
from matplotlib.path import Path as PlotPath
from shapely.geometry import Point, box

MAX_CELLS = 160000
MAX_SEGMENTS = 200000
MAX_CANDIDATES = 100000000


class SweepState:
    """Incremental form of the visit-count model used in final reports."""
    def __init__(self, size):
        self.counts = np.zeros(size, dtype=np.uint32)
        self.last_exit = np.full(size, -math.inf)
        self.arc = 0.0
        self.segments = self.candidates = self.chunks = 0


def polygons(geometry):
    if geometry.is_empty:
        return []
    if geometry.geom_type == 'Polygon':
        return [geometry]
    return [p for g in geometry.geoms for p in polygons(g)]


class AreaGrid:
    def __init__(self, field, headland_width, requested_resolution):
        if not field.is_valid or field.is_empty or field.area <= 0:
            raise ValueError('Invalid evaluation field')
        if not .01 <= requested_resolution <= 10000 or not math.isfinite(requested_resolution):
            raise ValueError('Invalid quality resolution')
        self.field = field
        self.interior = field.buffer(-headland_width) if headland_width else field
        self.x0, self.y0, self.x1, self.y1 = field.bounds
        resolution = requested_resolution
        while True:
            nx = max(1, math.ceil((self.x1-self.x0)/resolution))
            ny = max(1, math.ceil((self.y1-self.y0)/resolution))
            if nx*ny <= MAX_CELLS:
                break
            resolution *= max(1.01, math.sqrt(nx*ny/MAX_CELLS))
        self.resolution, self.nx, self.ny = resolution, nx, ny
        x, y = np.meshgrid(self.x0+(np.arange(nx)+.5)*resolution,
                           self.y0+(np.arange(ny)+.5)*resolution)
        self.x, self.y = x.ravel(), y.ravel()
        self.weights = self.area_weights(field, adjust_samples=True)
        self.interior_weights = self.area_weights(self.interior)
        if not np.isclose(self.weights.sum(), field.area, rtol=1e-7, atol=1e-8):
            raise ValueError('Evaluation grid failed field-area conservation')

    def area_weights(self, geometry, adjust_samples=False):
        weights = np.zeros(self.nx*self.ny)
        centers = np.column_stack((self.x, self.y))
        boundary = np.zeros((self.ny, self.nx), dtype=bool)
        h = self.resolution
        inside = np.zeros(len(weights), dtype=bool)
        for polygon in polygons(geometry):
            mask = PlotPath(np.asarray(polygon.exterior.coords)).contains_points(centers)
            for hole in polygon.interiors:
                mask &= ~PlotPath(np.asarray(hole.coords)).contains_points(centers)
            inside |= mask
            for ring in [polygon.exterior, *polygon.interiors]:
                for a, b in zip(ring.coords, list(ring.coords)[1:]):
                    steps = max(1, math.ceil(math.dist(a, b)/h*2))
                    t = np.linspace(0, 1, steps+1)
                    ix = np.floor((a[0]+t*(b[0]-a[0])-self.x0)/h).astype(int)
                    iy = np.floor((a[1]+t*(b[1]-a[1])-self.y0)/h).astype(int)
                    for dx in (-1, 0, 1):
                        for dy in (-1, 0, 1):
                            xx, yy = ix+dx, iy+dy
                            keep = (xx>=0)&(xx<self.nx)&(yy>=0)&(yy<self.ny)
                            boundary[yy[keep], xx[keep]] = True
        weights[inside] = h*h
        for index in np.flatnonzero(boundary.ravel()):
            row, col = divmod(int(index), self.nx)
            cell = box(self.x0+col*h, self.y0+row*h, self.x0+(col+1)*h, self.y0+(row+1)*h)
            clipped = geometry.intersection(cell)
            weights[index] = clipped.area
            if adjust_samples and clipped.area > 0 and not geometry.covers(Point(self.x[index], self.y[index])):
                point = clipped.representative_point()
                self.x[index], self.y[index] = point.x, point.y
        return weights

    def visits(self, chunks, width, state=None, check=None):
        if not math.isfinite(width) or width <= 0:
            raise ValueError('Invalid implement width')
        radius = width/2
        state = state if state is not None else SweepState(len(self.weights))
        counts, last_exit = state.counts, state.last_exit
        arc, segments, candidates = state.arc, state.segments, state.candidates
        for points in chunks:
            for a, b in zip(points, points[1:]):
                if not all(math.isfinite(v) for v in (*a, *b)):
                    raise ValueError('Nonfinite quality trajectory')
                length = math.dist(a, b)
                if length <= 1e-9:
                    continue
                segments += 1
                if check is not None and segments%128==1:check()
                if segments > MAX_SEGMENTS:
                    raise ValueError('Evaluation segment budget exceeded')
                # Include one neighbouring cell for representative points in
                # clipped boundary cells which differ from regular centres.
                lo_x = max(0, math.floor((min(a[0],b[0])-radius-self.x0)/self.resolution)-1)
                hi_x = min(self.nx, math.floor((max(a[0],b[0])+radius-self.x0)/self.resolution)+2)
                lo_y = max(0, math.floor((min(a[1],b[1])-radius-self.y0)/self.resolution)-1)
                hi_y = min(self.ny, math.floor((max(a[1],b[1])+radius-self.y0)/self.resolution)+2)
                if lo_x >= hi_x or lo_y >= hi_y:
                    arc += length
                    continue
                indices = (np.arange(lo_y,hi_y)[:,None]*self.nx + np.arange(lo_x,hi_x)).ravel()
                indices = indices[self.weights[indices] > 0]
                candidates += len(indices)
                if candidates > MAX_CANDIDATES:
                    raise ValueError('Evaluation geometry budget exceeded')
                dx, dy = (b[0]-a[0])/length, (b[1]-a[1])/length
                rx, ry = self.x[indices]-a[0], self.y[indices]-a[1]
                along = rx*dx+ry*dy
                cross2 = (rx*dy-ry*dx)**2
                delta = np.sqrt(np.maximum(0, radius*radius-cross2))
                enter, leave = np.maximum(0,along-delta), np.minimum(length,along+delta)
                valid = (cross2 <= radius*radius+1e-12)&(leave >= enter-1e-10)
                indices, enter, leave = indices[valid], enter[valid]+arc, leave[valid]+arc
                counts[indices] += (enter > last_exit[indices]+1e-7).astype(np.uint32)
                last_exit[indices] = np.maximum(last_exit[indices], leave)
                arc += length
        state.arc, state.segments, state.candidates = arc, segments, candidates
        state.chunks += len(chunks)
        return counts, {'observed_length_m': arc, 'segments': segments,
                        'chunks': state.chunks, 'candidate_cell_checks': candidates}

    def metrics(self, counts):
        def region(weights):
            total = float(weights.sum())
            if total <= 1e-10:
                return {'available': False, 'area_m2': total, 'coverage_percent': None,
                        'overlap_percent': None, 'miss_percent': None}
            covered = float(weights[counts>=1].sum())
            overlap = float(weights[counts>=2].sum())
            extra = float(np.dot(weights, np.maximum(counts.astype(np.int64)-1,0)))
            return {'available': True, 'area_m2': total, 'covered_m2': covered,
                    'missed_m2': max(0,total-covered), 'overlap_m2': overlap,
                    'single_pass_m2': float(weights[counts==1].sum()),
                    'extra_pass_area_m2': extra,
                    'coverage_percent': 100*covered/total,
                    'miss_percent': 100*max(0,total-covered)/total,
                    'overlap_percent': 100*overlap/total,
                    'overlap_of_covered_percent': 100*overlap/covered if covered else None,
                    'extra_pass_percent': 100*extra/total}
        return {'field': region(self.weights), 'interior': region(self.interior_weights),
                'headland': region(np.maximum(0,self.weights-self.interior_weights))}
