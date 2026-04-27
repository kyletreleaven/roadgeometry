#pragma once
#include <unordered_map>
#include <utility>
#include <vector>
#include "optimal_flow.hpp"
#include "segment.hpp"
#include "traverse.hpp"

namespace roadgeometry {

// ---------------------------------------------------------------------------
// prematch — single road, output arg
//
// For each group in a road segment, pair up supply and demand greedily,
// pushing matched (i, j) pairs into `out` via push_back.
// ---------------------------------------------------------------------------
template <typename Road, typename Out>
void prematch(Road& road, Out& out) {
    for (auto& g : road) {
        int n = std::min(g.supply.size(), g.demand.size());
        for (int k = 0; k < n; ++k) {
            int i = g.supply.front(); g.supply.pop_front();
            int j = g.demand.front(); g.demand.pop_front();
            out.push_back({i, j});
        }
    }
}

// ---------------------------------------------------------------------------
// prematch_all — iterable of roads, output arg
// ---------------------------------------------------------------------------
template <typename Roads, typename Out>
void prematch_all(Roads& roads, Out& out) {
    for (auto& [road_id, road] : roads)
        prematch(road, out);
}

// ---------------------------------------------------------------------------
// compute_matching<Road, Vertex>
//
// Full matching pipeline:
//   sort_and_segment → prematch_all → compute_optimal_flow
//   → create_topograph → traverse
//
// INPUTS
// ------
//   P, Q      : supply/demand points as (road, y) pairs; indices 0..n-1
//   endpoints : Road → (Vertex u, Vertex v)
//   lengths   : Road → double
//   is_oneway : Road → bool  (missing → false, i.e. bidirectional)
//// OUTPUT
//   {matching, cost} where matching is a vector of (supply_idx, demand_idx)
//   pairs (prematch pairs first, then traverse pairs) and cost is the total
//   weighted transport distance.
// ---------------------------------------------------------------------------
template <typename Road, typename Vertex>
std::pair<std::vector<std::pair<int,int>>, double>
compute_matching(
    const std::vector<std::pair<Road, double>>&                P,
    const std::vector<std::pair<Road, double>>&                Q,
    const std::unordered_map<Road, std::pair<Vertex,Vertex>>&  endpoints,
    const std::unordered_map<Road, double>&                    lengths,
    const std::unordered_map<Road, bool>&                      is_oneway
) {
    auto segments = sort_and_segment<Road>(P, Q);

    std::vector<std::pair<int,int>> matching;
    prematch_all(segments, matching);

    auto flow = compute_optimal_flow<Road, Vertex>(
        segments, endpoints, lengths, is_oneway);

    auto topo = create_topograph<Road, Vertex>(
        segments, flow, endpoints, lengths);

    double cost = traverse(topo, matching);

    return {std::move(matching), cost};
}

} // namespace roadgeometry
