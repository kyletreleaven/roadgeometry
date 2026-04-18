"""Piecewise-linear convex cost functions.

A PWL (piecewise-linear) function is represented as a sorted list of segments:
    [(left, slope, offset), ...]
where each segment covers [left, next_left) and evaluates as slope*x + offset.
The first segment always has left = -inf, covering (-inf, next_left).

Callable as f(x) -> float via bisect-based floor lookup (O(log n)).
"""

import bisect
import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import bintrees

# (left, slope, offset) — left is the inclusive left boundary of the segment
Segment = tuple[float, float, float]


class PWL:
    """Piecewise-linear convex function backed by a sorted segment list.

    The first segment's left is always -inf.  Evaluation uses bisect for an
    O(log n) floor lookup, matching the semantics of bintrees.RBTree.floor_item.
    """

    def __init__(self, segments: list[Segment]) -> None:
        assert segments, "PWL must have at least one segment"
        assert segments[0][0] == -math.inf, "first segment must have left = -inf"
        self._segments = segments
        # Pre-extract breakpoints for bisect (skip the -inf first entry).
        self._lefts = [s[0] for s in segments]

    def __call__(self, x: float) -> float:
        # O(log n) bisect floor lookup.
        # TODO: in the road-matching case all breakpoints are integers, so this
        # could be O(1) via direct array indexing after an integer floor.
        i = bisect.bisect_right(self._lefts, x) - 1
        if i < 0:
            i = 0
        _, slope, offset = self._segments[i]
        return slope * x + offset

    @property
    def segments(self) -> list[Segment]:
        return self._segments



class IntPWL:
    """Piecewise-linear function with contiguous integer breakpoints.

    Segments are indexed by integer i in [offset, offset + size), so evaluation
    is O(1): look up int(floor(x)) directly into the slope/intercept arrays.

    This is a special case of PWL arising in road matching, where flow levels
    are always integers.  Maps naturally to two parallel C++ std::vector<double>
    plus an int offset — no binary search needed.
    """

    def __init__(self, offset: int, slopes: list[float], intercepts: list[float]) -> None:
        assert len(slopes) == len(intercepts)
        self._offset = offset
        self._slopes = slopes
        self._intercepts = intercepts

    def __call__(self, x: float) -> float:
        i = max(0, min(len(self._slopes) - 1, int(math.floor(x)) - self._offset))
        return self._slopes[i] * x + self._intercepts[i]

    @property
    def segments(self) -> list[Segment]:
        """Convert to PWL segment format for use with negate/shift."""
        result = []
        for k, (slope, intercept) in enumerate(zip(self._slopes, self._intercepts)):
            left = -math.inf if k == 0 else float(self._offset + k)
            result.append((left, slope, intercept))
        return result

    @property
    def offset(self) -> int:
        return self._offset

    @property
    def size(self) -> int:
        return len(self._slopes)

    @property
    def slopes(self) -> list[float]:
        return self._slopes

    @property
    def intercepts(self) -> list[float]:
        return self._intercepts


def negate(pwl: PWL) -> PWL:
    """Return a PWL for g(x) = f(-x).

    Negating the argument reverses the segment order and negates both the
    breakpoints and the slopes.  The first segment of the result gets -inf.

        f(x) = alpha*x + kappa  =>  g(x) = f(-x) = -alpha*x + kappa
    """
    segs = pwl.segments
    n = len(segs)
    result: list[Segment] = []
    for i in range(n - 1, -1, -1):
        left, slope, offset = segs[i]
        # New left is the negation of the *next* segment's left boundary,
        # i.e., the right end of this segment in the original.
        # For the last original segment (first in result) use -inf.
        if i == n - 1:
            new_left = -math.inf
        else:
            new_left = -segs[i + 1][0]
        result.append((new_left, -slope, offset))
    return PWL(result)


def shift(pwl: PWL, s: float) -> PWL:
    """Return a PWL for g(x) = f(x + s).

    Breakpoints shift left by s; slopes are unchanged; offsets absorb the
    constant term: slope*(x+s) + offset = slope*x + (slope*s + offset).
    """
    result: list[Segment] = []
    for left, slope, offset in pwl.segments:
        new_left = -math.inf if left == -math.inf else left - s
        result.append((new_left, slope, slope * s + offset))
    return PWL(result)
