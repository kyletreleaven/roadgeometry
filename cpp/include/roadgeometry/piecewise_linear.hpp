#pragma once
#include <algorithm>
#include <cassert>
#include <vector>

namespace roadgeometry {

// ---------------------------------------------------------------------------
// PiecewiseLinear
//
// A convex piecewise-linear function f: R -> R represented as a sorted
// sequence of (breakpoint, slope, offset) triples.  On the interval
// [breakpoints[i], breakpoints[i+1]), f(x) = slopes[i] * x + offsets[i].
//
// The first breakpoint is typically -infinity (represented as the smallest
// value whose segment covers all x below the second breakpoint).
// Evaluation is O(log n) via binary search.
// ---------------------------------------------------------------------------
class PiecewiseLinear {
public:
    // Each segment covers [left, next_left) — left-closed, right-open.
    struct Segment {
        double left;   // inclusive left endpoint of this segment
        double slope;
        double offset;
    };

    // Construct from a sorted (ascending left) list of segments.
    explicit PiecewiseLinear(std::vector<Segment> segments)
        : segments_(std::move(segments))
    {
        assert(!segments_.empty());
    }

    double operator()(double x) const {
        // O(log n) binary search.
        // TODO: in the road-matching case all breakpoints are integers, so this
        // could be O(1) via a direct array index after an integer floor.
        auto it = std::upper_bound(
            segments_.begin(), segments_.end(), x,
            [](double val, const Segment& s) { return val < s.left; }
        );
        if (it != segments_.begin()) --it;
        return it->slope * x + it->offset;
    }

    const std::vector<Segment>& segments() const { return segments_; }

private:
    std::vector<Segment> segments_;
};

} // namespace roadgeometry
