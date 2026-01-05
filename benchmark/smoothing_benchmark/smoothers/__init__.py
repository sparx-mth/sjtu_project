"""
Trajectory Smoothers Package
============================
Standalone smoothing algorithms for path planning benchmarks.

Available smoothers:
- CubicSplineSmoother: Standard cubic spline interpolation
- CubicBezierSmoother: Cubic Hermite spline with heading continuity
- MinSnapSmoother: Minimum-snap optimization (requires minsnap-trajectories)
"""

from .cubic_spline_smoother import CubicSplineSmoother, smooth_waypoints as smooth_cubic_spline
from .cubic_bezier_smoother import CubicBezierSmoother, smooth_waypoints as smooth_cubic_bezier

# MinSnap may not be available if the package isn't installed
try:
    from .min_snap_smoother import MinSnapSmoother, smooth_waypoints as smooth_min_snap, MINSNAP_AVAILABLE
except ImportError:
    MINSNAP_AVAILABLE = False
    MinSnapSmoother = None
    smooth_min_snap = None

__all__ = [
    'CubicSplineSmoother',
    'CubicBezierSmoother',
    'MinSnapSmoother',
    'smooth_cubic_spline',
    'smooth_cubic_bezier',
    'smooth_min_snap',
    'MINSNAP_AVAILABLE',
]