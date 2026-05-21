#pragma once
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>
#include "double_ended_vector.hpp"
#include "input_graph.hpp"
#include "piecewise_linear.hpp"
#include "robust_mccf.hpp"
#include "segment.hpp"

namespace roadgeometry {

// ---------------------------------------------------------------------------
// VectorMap<V>
//
// Zero-cost adapter that gives a std::vector<V> the find()/begin()/end()
// interface of an int-keyed map, where key == index.  Used to pass a
// sequential edge-cost vector to robust_mccf without copying into an
// unordered_map.
// ---------------------------------------------------------------------------
template <typename V>
struct VectorMap {
    const std::vector<V>& data;

    struct iterator {
        const V* ptr;
        int      key;
        bool operator==(const iterator& o) const { return ptr == o.ptr; }
        struct ref { int first; const V& second; };
        ref        operator*()  const { return {key, *ptr}; }
        iterator&  operator++()       { ++ptr; ++key; return *this; }
    };

    iterator find(int key) const {
        if (key >= 0 && static_cast<std::size_t>(key) < data.size())
            return {&data[key], key};
        return end();
    }
    iterator begin() const { return {data.data(), 0}; }
    iterator end()   const { return {data.data() + data.size(), static_cast<int>(data.size())}; }
};

// ---------------------------------------------------------------------------
// measure
//
// Computes the interval-length histogram over integer flow levels for a single
// road segment.  Returns a DoubleEndedVector mapping flow level f → total
// length of road at that level.  The segment deques are read but not consumed.
// ---------------------------------------------------------------------------
template <typename Road>
DoubleEndedVector<double> measure(Road& road, double length) {
    DoubleEndedVector<double> result;
    int    f      = 0;
    double prev_y = 0.0;
    for (auto& g : road) {
        result[f] += g.y - prev_y;
        f      += (int)g.supply.size() - (int)g.demand.size();
        prev_y  = g.y;
    }
    result[f] += length - prev_y;
    return result;
}

// ---------------------------------------------------------------------------
// objective_from_measure
//
// Builds a convex PiecewiseLinear cost function from a flow-level histogram.
//
// The cost function f(z) is defined on the real line with n+1 linear pieces,
// where n = hi - lo + 1 (number of occupied flow levels).  Breakpoints are at
// integer values in [-hi-1, -lo]; the first piece extends to -∞.
//
// Slopes and offsets are computed by the sweep formula:
//
//   total_w  = sum_j w_j          (w_j = measure[lo+j])
//   total_kw = sum_j (lo+j) * w_j
//
//   alpha[j] = total_w  - 2 * prefix_w[j]     (slope of piece j from left)
//   kappa[j] = total_kw - 2 * prefix_kw[j]    (offset of piece j)
//
// Pieces are stored in ascending order of left boundary; piece k has
// left = -(hi+1)+k (or -∞ for k=0) and uses alpha[n-k], kappa[n-k].
// ---------------------------------------------------------------------------
inline PiecewiseLinear objective_from_measure(const DoubleEndedVector<double>& m) {
    int lo = m.min_index(), hi = m.max_index();
    int n  = hi - lo + 1;

    std::vector<double> alpha(n + 1), kappa(n + 1);
    double total_w = 0.0, total_kw = 0.0;
    for (int j = 0; j < n; ++j) {
        total_w  += m[lo + j];
        total_kw += static_cast<double>(lo + j) * m[lo + j];
    }
    double prefix_w = 0.0, prefix_kw = 0.0;
    for (int j = 0; j <= n; ++j) {
        alpha[j] = total_w  - 2.0 * prefix_w;
        kappa[j] = total_kw - 2.0 * prefix_kw;
        if (j < n) {
            double w  = m[lo + j];
            double fj = static_cast<double>(lo + j);
            prefix_w  += w;
            prefix_kw += fj * w;
        }
    }

    std::vector<PiecewiseLinear::Segment> segs;
    segs.reserve(n + 1);
    for (int k = 0; k <= n; ++k) {
        double left = (k == 0) ? -std::numeric_limits<double>::infinity()
                               : static_cast<double>(-(hi + 1) + k);
        segs.push_back({left, alpha[n - k], kappa[n - k]});
    }
    return PiecewiseLinear(std::move(segs));
}

// Shift f(z) → f(z - zmin): moves the domain right by zmin.
inline PiecewiseLinear pwl_shift(const PiecewiseLinear& obj, double zmin) {
    std::vector<PiecewiseLinear::Segment> segs;
    segs.reserve(obj.segments().size());
    for (auto& s : obj.segments())
        segs.push_back({s.left + zmin, s.slope, s.offset - s.slope * zmin});
    return PiecewiseLinear(std::move(segs));
}

// Negate f(z) → f(-z): reverses segment order, negates slopes, keeps offsets.
inline PiecewiseLinear pwl_negate(const PiecewiseLinear& obj) {
    const auto& orig = obj.segments();
    int ns = (int)orig.size();
    std::vector<PiecewiseLinear::Segment> segs;
    segs.reserve(ns);
    for (int k = ns - 1; k >= 0; --k) {
        double next_left = (k + 1 < ns) ? orig[k+1].left
                                         : std::numeric_limits<double>::infinity();
        double new_left  = (k == ns - 1) ? -std::numeric_limits<double>::infinity()
                                         : -next_left;
        segs.push_back({new_left, -orig[k].slope, orig[k].offset});
    }
    return PiecewiseLinear(std::move(segs));
}

// ---------------------------------------------------------------------------
// MatchingCostMap
//
// Unified cost container for the matching MCCF reduction.  Carries:
//   fns     — PWL cost function per edge_id, for non-empty roads only
//             (or all roads when EmptyRoadCost=true).  Absent = empty road.
//   lengths — road length per edge_id, for all edges.  Used by
//             fragile_mccf_sparse to compute empty-road lincost on demand.
//
// Implements the Cost concept expected by fragile_mccf_sparse:
//   find(key)  → iterator to PiecewiseLinear, or end() if absent (empty road)
//   begin/end  → iterates over non-empty road entries (for Stage 1 and make_cbound)
//   length(e)  → road length for lincost of empty arcs
//   empty_road_cbound(U) → sum of length×U for roads absent from fns (for cycle-
//                          edge prohibitive cost when EmptyRoadCost=false)
// ---------------------------------------------------------------------------
struct MatchingCostMap {
    std::unordered_map<int, PiecewiseLinear> fns;
    std::vector<double>                      lengths;

    using const_iterator = std::unordered_map<int, PiecewiseLinear>::const_iterator;
    const_iterator find(int key)  const { return fns.find(key); }
    const_iterator begin()        const { return fns.begin(); }
    const_iterator end()          const { return fns.end(); }

    double length(int e)          const { return lengths[static_cast<std::size_t>(e)]; }

    double empty_road_cbound(double U) const {
        double sum = 0.0;
        for (int e = 0; e < static_cast<int>(lengths.size()); ++e)
            if (fns.find(e) == fns.end()) sum += lengths[e];
        return sum * U;
    }
};

// ---------------------------------------------------------------------------
// FlowInstance<Vertex>
//
// A pure MCCF problem instance: network, supply, cost, U.
// Road-agnostic — suitable for passing directly to robust_mccf.
// ---------------------------------------------------------------------------
template <typename Vertex>
struct FlowInstance {
    HashMapGraph<Vertex, int>          network;
    std::unordered_map<Vertex, double> vertex_supply;
    MatchingCostMap                    cost;
    double                             U;
};

// ---------------------------------------------------------------------------
// FlowReduction<Road, Vertex>
//
// A FlowInstance together with the translation members needed to decode
// the edge-indexed flow back to Road-keyed flow.
// ---------------------------------------------------------------------------
template <typename Road, typename Vertex>
struct FlowReduction {
    FlowInstance<Vertex>             instance;
    std::vector<std::pair<Road,int>> edge_to_road; // indexed by edge_id; {road, sign}
    std::vector<double>              oneway_zmin;  // indexed by edge_id; 0.0 for bidirectional
};

// ---------------------------------------------------------------------------
// build_flow_reduction
//
// For each road in `lengths`:
//   - Computes measure from the (possibly pre-matched) segment.
//   - Derives surplus (net supply - demand) → added to vertex_supply[v].
//   - Builds a convex PWL objective from the measure.
//   - Oneway road: shifts objective so domain starts at 0; adds supply bias
//     to u and v to compensate.  One forward edge.
//   - Bidirectional road: forward edge u→v with obj, reverse edge v→u with
//     negated objective obj(-z).
//   - Accumulates U = 1 + sum over roads of (measure size - 1).
// ---------------------------------------------------------------------------
template <typename Road, typename Vertex>
FlowReduction<Road, Vertex> build_flow_reduction(
    RoadSegments<Road>&                                        segments,
    const std::unordered_map<Road, std::pair<Vertex,Vertex>>& endpoints,
    const std::unordered_map<Road, double>&                   lengths,
    const std::unordered_map<Road, bool>&                     is_oneway,
    bool empty_road_cost = true
) {
    FlowReduction<Road, Vertex> red;
    auto& inst = red.instance;
    inst.U = 1.0;  // base: +1 ensures U > max possible edge flow on empty network
    int edge_id = 0;

    for (auto& [road, length] : lengths) {
        auto ep_it = endpoints.find(road);
        if (ep_it == endpoints.end()) continue;

        auto [u, v] = ep_it->second;
        inst.network.add_node(u);
        inst.network.add_node(v);

        // Measure.
        static const std::vector<YGroup> empty_seg;
        auto seg_it = segments.find(road);
        const auto& seg = (seg_it != segments.end()) ? seg_it->second : empty_seg;
        DoubleEndedVector<double> m = measure(seg, length);
        inst.U += static_cast<double>(m.max_index() - m.min_index()); // size - 1

        // Surplus = net unmatched supply on this road → goes to endpoint v.
        int surplus = 0;
        for (auto& g : seg) surplus += (int)g.supply.size() - (int)g.demand.size();
        inst.vertex_supply[v] += static_cast<double>(surplus);

        // When empty_road_cost=false, skip cost fn for roads with no pins;
        // fragile_mccf_sparse detects absent key and computes lincost from edge_lengths.
        bool skip_cost = !empty_road_cost && seg.empty();
        std::optional<PiecewiseLinear> obj_opt;
        if (!skip_cost) obj_opt = objective_from_measure(m);

        auto ow_it = is_oneway.find(road);
        bool oneway = ow_it != is_oneway.end() && ow_it->second;

        if (oneway) {
            double zmin = static_cast<double>(-m.min_index());
            inst.vertex_supply[u] -= zmin;
            inst.vertex_supply[v] += zmin;
            inst.network.add_edge(edge_id, u, v);
            if (!skip_cost) inst.edge_cost.emplace(edge_id, pwl_shift(*obj_opt, zmin));
            inst.edge_lengths.push_back(length);
            red.oneway_zmin.push_back(zmin);
            red.edge_to_road.push_back({road, +1});
            ++edge_id;
        } else {
            inst.network.add_edge(edge_id, u, v);
            if (!skip_cost) inst.edge_cost.emplace(edge_id, *obj_opt);
            inst.edge_lengths.push_back(length);
            red.oneway_zmin.push_back(0.0);
            red.edge_to_road.push_back({road, +1});
            ++edge_id;

            inst.network.add_edge(edge_id, v, u);
            if (!skip_cost) inst.edge_cost.emplace(edge_id, pwl_negate(*obj_opt));
            inst.edge_lengths.push_back(length);
            red.oneway_zmin.push_back(0.0);
            red.edge_to_road.push_back({road, -1});
            ++edge_id;
        }
    }

    return red;
}

// ---------------------------------------------------------------------------
// compute_optimal_flow<Road, Vertex>
//
// Given pre-segmented (and optionally pre-matched) road segments, computes
// the integer optimal flow for the roadnet matching problem.
//
// Templated on Road and Vertex — requires equality and std::hash only.
// The Python binding instantiates with Road = Vertex = py::object.
//
// INPUTS
// ------
//   segments  : RoadSegments<Road> — pre-computed segments; keys define the
//               road set.  May be pre-matched (deques partially consumed).
//   endpoints : Road → (Vertex u, Vertex v)
//   lengths   : Road → double  (keys define the road set)
//   is_oneway : Road → bool    (missing → false, i.e. bidirectional)
//   epsilon   : optimality tolerance for robust_mccf
//
// OUTPUT
//   flow : Road → int  — integer optimal flow per road
// ---------------------------------------------------------------------------
// EmptyRoadCost=true  — include explicit PWL cost fn for every road (required for dense).
// EmptyRoadCost=false — omit cost fn for empty roads; sparse solver computes lincost from
//                       edge_lengths on demand.  Illegal with UseSparse=false.
template <typename Road, typename Vertex, bool UseSparse = true, bool EmptyRoadCost = !UseSparse>
std::unordered_map<Road, int> compute_optimal_flow(
    RoadSegments<Road>&                                        segments,
    const std::unordered_map<Road, std::pair<Vertex,Vertex>>& endpoints,
    const std::unordered_map<Road, double>&                   lengths,
    const std::unordered_map<Road, bool>&                     is_oneway,
    double epsilon = 1.0
) {
    static_assert(EmptyRoadCost || UseSparse,
        "EmptyRoadCost=false requires UseSparse=true (dense solver needs explicit cost fns)");
    auto red = build_flow_reduction<Road, Vertex>(segments, endpoints, lengths, is_oneway, EmptyRoadCost);
    auto& inst = red.instance;
    int num_edges = static_cast<int>(red.edge_to_road.size());

    // For empty roads absent from edge_cost, add length*U to cbound so cycle edges
    // remain prohibitively expensive relative to any feasible solution.
    double extra_cbound = 0.0;
    for (int e = 0; e < (int)inst.edge_lengths.size(); ++e)
        if (inst.edge_cost.find(e) == inst.edge_cost.end())
            extra_cbound += inst.edge_lengths[e] * inst.U;

    std::unordered_map<int, double> capacity;  // empty — no explicit capacity bounds
    auto int_flow = robust_mccf<UseSparse>(inst.network, capacity, inst.vertex_supply,
                                           inst.edge_cost, VectorMap{inst.edge_lengths},
                                           inst.U, extra_cbound, epsilon);

    // De-normalize: accumulate signed flow back to Road-keyed result.
    // For oneway roads: add back zmin bias.
    // For bidirectional roads: net flow = f_fwd - f_rev (handled by sign).
    std::unordered_map<Road, double> road_flow;
    for (int e = 0; e < num_edges; ++e) {
        double x = 0.0;
        auto it = int_flow.find(e);
        if (it != int_flow.end()) x = it->second;
        auto& [road, sign] = red.edge_to_road[e];
        road_flow[road] += sign * (x + red.oneway_zmin[e]);
    }

    std::unordered_map<Road, int> flow;
    for (auto& [road, x] : road_flow)
        flow[road] = static_cast<int>(x);

    return flow;
}

} // namespace roadgeometry
