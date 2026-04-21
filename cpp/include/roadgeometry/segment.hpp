#pragma once
#include <algorithm>
#include <deque>
#include <unordered_map>
#include <utility>
#include <vector>

namespace roadgeometry {

// ---------------------------------------------------------------------------
// sort_and_segment<Road>
//
// Given supply points P and demand points Q, each a list of (road, y) pairs,
// returns a map from road to a sorted list of YGroups.
//
// Road must be:
//   - strictly ordered (operator<) for sorting
//   - equality-comparable and hashable (std::hash<Road>) for the result map
//
// Within each road, groups are sorted by y.  Within a group at the same y,
// supply indices precede demand indices, both in ascending order.
// ---------------------------------------------------------------------------

struct YGroup {
    double y;
    std::deque<int> supply;
    std::deque<int> demand;
};

template <typename Road>
using RoadSegments = std::unordered_map<Road, std::vector<YGroup>>;

template <typename Road>
RoadSegments<Road> sort_and_segment(
    const std::vector<std::pair<Road, double>>& P,
    const std::vector<std::pair<Road, double>>& Q
) {
    struct Point {
        Road   road;
        double y;
        int    side;   // 0=supply, 1=demand — supply sorts before demand at equal y
        int    index;
    };

    std::vector<Point> points;
    points.reserve(P.size() + Q.size());

    for (int i = 0; i < (int)P.size(); ++i)
        points.push_back({P[i].first, P[i].second, 0, i});
    for (int j = 0; j < (int)Q.size(); ++j)
        points.push_back({Q[j].first, Q[j].second, 1, j});

    std::stable_sort(points.begin(), points.end(), [](const Point& a, const Point& b) {
        if (a.road != b.road) return a.road < b.road;
        if (a.y    != b.y)    return a.y    < b.y;
        if (a.side != b.side) return a.side < b.side;
        return a.index < b.index;
    });

    RoadSegments<Road> result;

    int n = (int)points.size();
    int i = 0;
    while (i < n) {
        Road   road = points[i].road;
        double y    = points[i].y;
        YGroup group;
        group.y = y;
        while (i < n && points[i].road == road && points[i].y == y) {
            if (points[i].side == 0)
                group.supply.push_back(points[i].index);
            else
                group.demand.push_back(points[i].index);
            ++i;
        }
        result[road].push_back(std::move(group));
    }

    return result;
}

} // namespace roadgeometry
