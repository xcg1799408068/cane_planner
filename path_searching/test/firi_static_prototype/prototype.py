#!/usr/bin/env python3
"""Isolated 2D FIRI research prototype; NOT used by the ROS planner.

Adapted algorithmic foundation: GCOPTER firi.hpp, Zhepei Wang (2021),
https://github.com/ZJU-FAST-Lab/GCOPTER
pinned checkout e0444f6d47b84f972ced91746b05feb36ce1fd4f.

Differences: whole convex grid cells replace point obstacles; restrictive
inflation solves a small convex QP per retained cell, and MVIE uses SciPy
SLSQP with exact containment constraints instead of GCOPTER's 3D soft-penalty
L-BFGS solver. No upstream helper code is copied. This is a volume-seeking
alternating method, not a globally maximum-area guarantee. Geometry alone
cannot certify LFPC navigability. See NOTICE for upstream attribution.
"""
from dataclasses import dataclass
import time
import numpy as np
from scipy.optimize import minimize


class GeometryFailure(RuntimeError):
    """Fail closed: no partially successful corridor is returned."""


def polygon(A, b):
    """Bounded halfspace intersection, using all pairwise line intersections."""
    vertices = []
    for i in range(len(b)):
        for j in range(i):
            M = A[[i, j]]
            if abs(np.linalg.det(M)) > 1e-10:
                p = np.linalg.solve(M, b[[i, j]])
                if np.max(A @ p - b) <= 1e-8:
                    if not any(np.linalg.norm(p-q) < 1e-8 for q in vertices):
                        vertices.append(p)
    if len(vertices) < 3:
        raise GeometryFailure('DEGENERATE_POLYGON')
    v = np.array(vertices)
    center = v.mean(axis=0)
    return v[np.argsort(np.arctan2(v[:, 1]-center[1], v[:, 0]-center[0]))]


def area(v):
    # Translate before products: absolute-coordinate shoelace catastrophically
    # cancels even for small polygons at otherwise supported map coordinates.
    local = v-v[0]
    return abs(np.sum(local[:, 0]*np.roll(local[:, 1], -1)
                      - local[:, 1]*np.roll(local[:, 0], -1))) / 2


# Prototype numerical envelope (meters). Certificates are floating-point, not
# exact predicates. Large global frames must be rebased by the caller first.
MAX_COORDINATE = 1e4
MIN_RESOLUTION = .01
MAX_DOMAIN_SPAN = 100.


def supported_coordinates(points):
    return np.all(np.isfinite(points)) and np.max(np.abs(points)) <= MAX_COORDINATE



@dataclass
class Region:
    A: np.ndarray
    b: np.ndarray
    vertices: np.ndarray
    seed: np.ndarray
    ellipsoid_center: np.ndarray
    ellipsoid_L: np.ndarray
    iterations: int


@dataclass
class Grid:
    """Axis-aligned full cells: 0 free, 1 occupied, -1 unknown (blocked)."""
    values: np.ndarray
    resolution: float
    origin: np.ndarray

    def validate(self):
        if (self.values.ndim != 2 or 0 in self.values.shape
                or not np.isfinite(self.resolution) or self.resolution <= 0
                or np.shape(self.origin) != (2,) or not np.all(np.isfinite(self.origin))
                or not np.all(np.isin(self.values, [-1, 0, 1]))):
            raise GeometryFailure('INVALID_GRID')
        extent = np.array(self.values.shape[::-1])*self.resolution
        if (self.resolution < MIN_RESOLUTION or np.max(extent) > MAX_DOMAIN_SPAN
                or not supported_coordinates(self.origin)
                or not supported_coordinates(self.origin+extent)):
            raise GeometryFailure('UNSUPPORTED_NUMERICAL_SCALE')

    def local_geometry(self, seed, radius):
        self.validate()
        lo = np.maximum(self.origin, seed-radius)
        hi = np.minimum(self.origin + np.array(self.values.shape[::-1])*self.resolution,
                        seed+radius)
        if np.any(hi-lo <= 1e-6) or np.any(seed <= lo) or np.any(seed >= hi):
            raise GeometryFailure('SEED_OUT_OF_DOMAIN')
        A = np.array([[1., 0.], [-1., 0.], [0., 1.], [0., -1.]])
        b = np.array([hi[0], -lo[0], hi[1], -lo[1]])
        cells = []
        for iy, ix in np.argwhere(self.values != 0):
            a = self.origin + np.array([ix, iy])*self.resolution
            z = a+self.resolution
            if np.any(z < lo) or np.any(a > hi):
                continue
            cells.append(np.array([a, [z[0], a[1]], z, [a[0], z[1]]]))
        return A, b, cells


@dataclass
class Config:
    local_radius: float = 3.0
    clearance: float = 0.002
    iterations: int = 4
    optimizer_iterations: int = 150
    max_regions: int = 100
    min_progress: float = 0.005
    overlap_depth: float = 0.05
    overlap_area: float = 0.002
    max_seconds: float = 15.0

    def validate(self):
        floats = [self.local_radius, self.clearance, self.min_progress,
                  self.overlap_depth, self.overlap_area, self.max_seconds]
        if (not all(np.isfinite(x) and x > 0 for x in floats)
                or self.clearance >= self.overlap_depth
                or self.overlap_depth >= self.local_radius
                or any(not isinstance(x, int) or x < 1
                       for x in [self.iterations, self.optimizer_iterations, self.max_regions])):
            raise GeometryFailure('INVALID_CONFIG')
        if (self.local_radius > MAX_DOMAIN_SPAN or self.local_radius < .01
                or min(self.clearance, self.min_progress, self.overlap_depth) < 1e-4
                or self.overlap_area < 1e-8):
            raise GeometryFailure('UNSUPPORTED_NUMERICAL_SCALE')


def check_deadline(deadline):
    if time.monotonic() > deadline:
        raise GeometryFailure('TIME_BUDGET')


def mvie(A, b, center, L, cfg, deadline):
    """Maximize log det L with ||A_i L|| + A_i center <= b_i.

    Positive-diagonal lower triangular L parameterizes every 2D ellipse.
    The objective and feasible set are convex (minimization convention).
    """
    def unpack(x):
        return x[:2], np.array([[x[2], 0.], [x[3], x[4]]])

    def objective(x):
        check_deadline(deadline)
        return -np.log(x[2])-np.log(x[4])

    def constraints(x):
        c, l = unpack(x)
        return b-A@c-np.linalg.norm(A@l, axis=1)

    # Shrink the warm ellipse to a feasible strictly positive initial shape.
    slack = b-A@center
    if np.min(slack) <= 0:
        raise GeometryFailure('MVIE_NO_INTERIOR')
    scale = min(1., .9*np.min(slack/np.linalg.norm(A@L, axis=1)))
    L = L*scale
    x = np.array([*center, L[0, 0], L[1, 0], L[1, 1]])
    result = minimize(objective, x, method='SLSQP',
                      bounds=[(None, None), (None, None), (1e-7, None),
                              (None, None), (1e-7, None)],
                      constraints={'type': 'ineq', 'fun': constraints},
                      options={'maxiter': cfg.optimizer_iterations, 'ftol': 1e-9})
    if (not result.success or not np.all(np.isfinite(result.x))
            or np.min(constraints(result.x)) < -1e-7):
        raise GeometryFailure('MVIE_SOLVER_FAILURE: '+result.message)
    c, l = unpack(result.x)
    # Remove allowed solver residual before using this ellipse in another pass.
    l *= min(1., np.min((b-A@c)/np.linalg.norm(A@l, axis=1)))*(1-1e-8)
    return c, l


def separating_plane(cell, seed, center, L, cfg, deadline):
    """Restrictive inflation in whitened ellipse coordinates.

    min ||n||^2/2, with n.v >= 1 for ALL cell vertices and n.seed <= 1.
    The resulting halfspace contains the seed and excludes the entire convex
    cell, not just its center/vertices individually via unrelated planes.
    """
    check_deadline(deadline)
    inv = np.linalg.inv(L)
    V = (cell-center)@inv.T
    s = inv@(seed-center)
    C = np.vstack((V, -s))
    d = np.array([1., 1., 1., 1., -1.])
    guess = V.mean(axis=0)
    guess /= max(np.dot(guess, guess), 1e-8)
    result = minimize(lambda n: .5*np.dot(n, n), guess, jac=lambda n: n,
                      method='SLSQP', constraints={'type': 'ineq',
                      'fun': lambda n: C@n-d, 'jac': lambda n: C},
                      options={'maxiter': cfg.optimizer_iterations, 'ftol': 1e-10})
    if not result.success or not np.all(np.isfinite(result.x)):
        raise GeometryFailure('SEPARATION_SOLVER_FAILURE')
    n = inv.T@result.x
    norm = np.linalg.norm(n)
    if norm <= 1e-10 or np.min(C@result.x-d) < -1e-7:
        raise GeometryFailure('INVALID_SEPARATOR')
    n /= norm
    # Recompute support from whole obstacle, then add strict metric clearance.
    offset = np.min(cell@n)-cfg.clearance
    if n@seed > offset+1e-9:
        raise GeometryFailure('SEED_CLEARANCE')
    return n, offset, np.linalg.norm(result.x)


def inflate(grid, seed, cfg=None, deadline=None):
    cfg = cfg or Config()
    cfg.validate()
    seed = np.asarray(seed, dtype=float)
    if seed.shape != (2,) or not np.all(np.isfinite(seed)):
        raise GeometryFailure('INVALID_SEED')
    if not supported_coordinates(seed):
        raise GeometryFailure('UNSUPPORTED_NUMERICAL_SCALE')
    deadline = deadline or time.monotonic()+cfg.max_seconds
    boundA, boundb, cells = grid.local_geometry(seed, cfg.local_radius)
    # Domain is closed, but a small strict clearance excludes out-of-map contact.
    boundb = boundb-cfg.clearance
    if np.min(boundb-boundA@seed) <= 0:
        raise GeometryFailure('SEED_OUT_OF_DOMAIN')
    for cell in cells:
        if np.all(seed >= cell.min(axis=0)-cfg.clearance) and np.all(
                seed <= cell.max(axis=0)+cfg.clearance):
            raise GeometryFailure('SEED_BLOCKED')
    center, L = seed.copy(), np.eye(2)*.05
    for iteration in range(cfg.iterations):
        check_deadline(deadline)
        A, b = list(boundA), list(boundb)
        # Near cells first in current ellipsoid metric. Each retained separator
        # removes only cells certified wholly beyond that one plane.
        remaining = list(cells)
        while remaining:
            check_deadline(deadline)
            distances = [np.min(np.linalg.norm((v-center)@np.linalg.inv(L).T, axis=1))
                         for v in remaining]
            cell = remaining[int(np.argmin(distances))]
            n, offset, _ = separating_plane(cell, seed, center, L, cfg, deadline)
            A.append(n); b.append(offset)
            remaining = [v for v in remaining
                         if np.min(v@n)-offset < cfg.clearance-1e-8]
            if len(remaining) == len(distances):
                raise GeometryFailure('SEPARATION_NO_PROGRESS')
        A, b = np.array(A), np.array(b)
        center, L = mvie(A, b, center, L, cfg, deadline)
    vertices = polygon(A, b)
    if (not np.all(np.isfinite(vertices)) or area(vertices) < 1e-8
            or np.max(A@seed-b) > 1e-8
            or np.max(boundA@vertices.T-boundb[:, None]) > 1e-7
            or np.min(b-A@center-np.linalg.norm(A@L, axis=1)) < -1e-7):
        raise GeometryFailure('OUTPUT_CERTIFICATE')
    # A separating axis per blocked cell is a full polygon/cell certificate.
    for cell in cells:
        if not np.any(np.min(A@cell.T-b[:, None], axis=1) >= cfg.clearance-1e-7):
            raise GeometryFailure('CELL_INTERSECTION_CERTIFICATE')
    return Region(A, b, vertices, seed, center, L, iteration+1)


def point_at(path, arc, s):
    i = min(np.searchsorted(arc, s, side='right')-1, len(path)-2)
    return path[i]+(path[i+1]-path[i])*((s-arc[i])/(arc[i+1]-arc[i]))


def coverage(region, path, arc, start, inset=0.):
    """Forward contiguous prefix, clipping ORIGINAL edges without shortcuts."""
    i = min(np.searchsorted(arc, start, side='right')-1, len(path)-2)
    current = point_at(path, arc, start)
    if np.max(region.A@current-(region.b-inset)) > 1e-8:
        return start
    for j in range(i, len(path)-1):
        end = path[j+1]
        delta = end-current
        slack = region.b-inset-region.A@current
        rates = region.A@delta
        positive = rates > 1e-12
        t = min(1., np.min(slack[positive]/rates[positive]) if np.any(positive) else 1.)
        base = start if j == i else arc[j]
        if t < 1.-1e-10:
            return base+max(0., t)*(arc[j+1]-base)
        current = end
    return arc[-1]


def overlap_certificate(a, b, cfg):
    A, offsets = np.vstack((a.A, b.A)), np.r_[a.b, b.b]
    v = polygon(A, offsets)
    actual_area = area(v)
    # Eroding by requested depth gives a directly checked inscribed disk center.
    inner = polygon(A, offsets-cfg.overlap_depth)
    center = inner.mean(axis=0)
    depth = np.min(offsets-A@center)
    if actual_area < cfg.overlap_area or depth < cfg.overlap_depth-1e-8:
        raise GeometryFailure('INSUFFICIENT_OVERLAP')
    return actual_area, depth


def corridor(grid, path, cfg=None):
    cfg = cfg or Config()
    cfg.validate()
    path = np.asarray(path, dtype=float)
    if path.ndim != 2 or path.shape[1:] != (2,) or len(path) < 2 or not np.all(np.isfinite(path)):
        raise GeometryFailure('INVALID_PATH')
    if not supported_coordinates(path):
        raise GeometryFailure('UNSUPPORTED_NUMERICAL_SCALE')
    lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    if np.any(lengths < 1e-9):
        raise GeometryFailure('DEGENERATE_PATH')
    arc = np.r_[0., np.cumsum(lengths)]
    deadline = time.monotonic()+cfg.max_seconds
    regions, overlaps = [], []
    start, covered = 0., 0.
    for _ in range(cfg.max_regions):
        r = inflate(grid, point_at(path, arc, start), cfg, deadline)
        end = coverage(r, path, arc, start)
        if end <= covered+cfg.min_progress and end < arc[-1]-1e-8:
            raise GeometryFailure('NO_PROGRESS')
        if regions:
            try:
                overlaps.append(overlap_certificate(regions[-1], r, cfg))
            except GeometryFailure as exc:
                raise GeometryFailure('INSUFFICIENT_OVERLAP') from exc
        regions.append(r)
        if end >= arc[-1]-1e-8:
            return regions, overlaps
        # Interior point semantics only: no artificial shared-area seed constraint.
        next_start = coverage(r, path, arc, start, inset=2*cfg.overlap_depth)
        if next_start <= start+cfg.min_progress or next_start >= end:
            raise GeometryFailure('NO_INTERIOR_PROGRESS')
        start, covered = next_start, end
    raise GeometryFailure('REGION_BUDGET')
