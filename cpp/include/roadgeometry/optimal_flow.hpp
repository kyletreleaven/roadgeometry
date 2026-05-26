#pragma once
#include <cmath>
#include <limits>
#include <optional>
#include <unordered_map>
#include <utility>
#include <variant>
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
// LinearCost
//
// Zero-cost callable for empty roads: cost = length × |f|.
// Returned by value from MatchingCostMap::find() for edges with no pins.
// No heap allocation; fits in a register.
// ---------------------------------------------------------------------------
struct LinearCost {
    double length;
    double operator()(double x) const noexcept { return length * std::abs(x); }
};

// ---------------------------------------------------------------------------
// MatchingCostMap
//
// Unified cost container for the matching MCCF reduction.  Carries:
//   fns     — PWL cost function per edge_id, for non-empty roads only.
//   lengths — road length per edge_id, for all edges.
//
// Implements the MatchingCost concept expected by fragile_mccf_sparse:
//   find(key)         → always valid; empty roads return CostRef{LinearCost{len}}.
//   is_non_empty(key) → bool; true iff edge has a PWL cost fn (fns.count > 0).
//   non_empty_edges() → const ref to fns; for change_delta lincost invalidation
//                       and future negative_arcs tracked set.
//   length(e)         → road length; for empty-arc fast-path lincost computation.
// ---------------------------------------------------------------------------
struct MatchingCostMap {
    std::unordered_map<int, PiecewiseLinear> fns;
    std::vector<double>                      lengths;

    // Callable cost reference: non-owning pointer to PWL (non-empty road) or
    // inline LinearCost (empty road).  No heap allocation in either case.
    struct CostRef {
        std::variant<const PiecewiseLinear*, LinearCost> v;
        double operator()(double x) const {
            if (const auto* p = std::get_if<const PiecewiseLinear*>(&v)) return (**p)(x);
            return std::get<LinearCost>(v)(x);
        }
    };

    using value_type = std::pair<int, CostRef>;

    struct iterator {
        std::optional<value_type> entry_;
        const value_type* operator->() const { return &*entry_; }
        bool operator==(const iterator& o) const {
            if (!entry_ && !o.entry_) return true;
            if (!entry_ || !o.entry_) return false;
            return entry_->first == o.entry_->first;
        }
        bool operator!=(const iterator& o) const { return !(*this == o); }
    };

    iterator end() const { return {std::nullopt}; }

    iterator find(int key) const {
        auto it = fns.find(key);
        if (it != fns.end())
            return {value_type{key, CostRef{&it->second}}};
        return {value_type{key, CostRef{LinearCost{lengths[static_cast<std::size_t>(key)]}}}};
    }

    bool is_non_empty(int key) const { return fns.count(key) > 0; }

    const std::unordered_map<int, PiecewiseLinear>& non_empty_edges() const { return fns; }

    double length(int e) const { return lengths[static_cast<std::size_t>(e)]; }
};

// ---------------------------------------------------------------------------
// FlowInstance<Vertex, Cost>
//
// A pure MCCF problem instance: network, supply, cost, U.
// Road-agnostic — suitable for passing directly to robust_mccf.
//
// Cost type:
//   UseSparse=true  → MatchingCostMap  (default)
//   UseSparse=false → std::unordered_map<int, PiecewiseLinear>
// ---------------------------------------------------------------------------
template <typename Vertex, typename Cost = MatchingCostMap>
struct FlowInstance {
    HashMapGraph<Vertex, int>          network;
    std::unordered_map<Vertex, double> vertex_supply;
    Cost                               cost;
    double                             U;
};

// ---------------------------------------------------------------------------
// FlowReduction<Road, Vertex, Cost>
//
// A FlowInstance together with the translation members needed to decode
// the edge-indexed flow back to Road-keyed flow.
// ---------------------------------------------------------------------------
template <typename Road, typename Vertex, typename Cost = MatchingCostMap>
struct FlowReduction {
    FlowInstance<Vertex, Cost>       instance;
    std::vector<std::pair<Road,int>> edge_to_road; // indexed by edge_id; {road, sign}
    std::vector<double>              oneway_zmin;  // indexed by edge_id; 0.0 for bidirectional
};

// ---------------------------------------------------------------------------
// build_flow_reduction
//
// Reduces the roadnet matching problem to a min-cost convex flow instance.
//
// For each road in `lengths`:
//   - Computes measure m from the (possibly pre-matched) segment, where m[k]
//     is the total road length at flow level k.
//   - Derives surplus (net supply - demand) → added to vertex_supply[v].
//   - Builds obj = objective_from_measure(m): the convex PWL function whose
//     value obj(z) is the optimal matching cost on this road given net flow z.
//   - Oneway road: shifts objective so domain starts at 0; adds supply bias
//     to u and v to compensate.  One forward edge.
//   - Bidirectional road: two antiparallel edges —
//       e_fwd (u→v) with cost obj(z)
//       e_rev (v→u) with cost obj(-z)  [via pwl_negate]
//   - Accumulates U = 1 + sum over roads of (measure size - 1).
//
// Correctness of the bidirectional decomposition
// -----------------------------------------------
// The reduction is a related MCCF instance, not a direct encoding of the
// signed-flow matching problem.  The matching problem minimises Σ obj(z_e)
// over signed net flows z_e ∈ [−U, U]; the reduction instead minimises
// Σ [c_fwd(f_e) + c_rev(g_e)] over non-negative arc flows, where
// c_fwd(f) = obj(f) and c_rev(g) = obj(−g).
//
// The two problems share the same optimal net flows:
//
//   Bijection: every unidirectional reduction flow (f_e ≥ 0, g_e = 0, or
//   f_e = 0, g_e ≥ 0) corresponds bijectively to a signed flow z_e = f_e − g_e
//   on the original, with reduction cost obj(z_e) + obj(0).
//
//   Dominance: any anti-parallel reduction flow (f_e > 0 and g_e > 0) has
//   strictly higher cost than the cancellation (f_e − δ, g_e − δ) for any
//   δ ∈ (0, min(f_e, g_e)] — by convexity of obj.  So every optimal
//   reduction flow is unidirectional.
//
//   Conclusion: the reduction's optimal unidirectional flow maps to the
//   matching problem's optimal net flow.  Costs differ by a fixed constant
//   Σ obj(0) per bidirectional road, which does not affect which flow is
//   optimal.
//
//   Lower bounds: fragile_mccf assumes all arcs have lb = 0 (initial flow
//   x = 0 is feasible).  The reduction preserves this: bidirectional roads
//   use two non-negative arcs; oneway roads shift the domain so that zmin
//   maps to 0, absorbed into the supply map.
//   See TODO.md: native lb[e] ≤ 0 support in fragile_mccf would allow a
//   single arc per road and dramatically reduce the supply shift and re-translation.
//
// UseSparse=true  → Cost = MatchingCostMap: empty roads stored in lengths only,
//                   no PWL fn (fragile_mccf_sparse computes lincost on demand).
// UseSparse=false → Cost = unordered_map<int, PiecewiseLinear>: all roads get
//                   an explicit PWL fn (required by dense solver).
// ---------------------------------------------------------------------------
template <typename Road, typename Vertex, bool UseSparse = true>
auto build_flow_reduction(
    RoadSegments<Road>&                                        segments,
    const std::unordered_map<Road, std::pair<Vertex,Vertex>>& endpoints,
    const std::unordered_map<Road, double>&                   lengths,
    const std::unordered_map<Road, bool>&                     is_oneway
) {
    using Cost = std::conditional_t<UseSparse,
                     MatchingCostMap,
                     std::unordered_map<int, PiecewiseLinear>>;
    FlowReduction<Road, Vertex, Cost> red;
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

        // Sparse: skip PWL fn for empty roads (lincost computed from length on demand).
        // Dense: always build PWL fn (solver has no length-based fallback).
        bool is_empty = seg.empty();
        std::optional<PiecewiseLinear> obj_opt;
        if (!is_empty || !UseSparse) obj_opt = objective_from_measure(m);

        auto ow_it = is_oneway.find(road);
        bool oneway = ow_it != is_oneway.end() && ow_it->second;

        // Helper: store cost fn for one edge_id in the cost map.
        auto add_cost = [&](int eid, const PiecewiseLinear& fn) {
            if constexpr (UseSparse) {
                inst.cost.fns.emplace(eid, fn);
            } else {
                inst.cost.emplace(eid, fn);
            }
        };
        // Sparse only: record road length for every edge_id (empty or not).
        auto add_length = [&]() {
            if constexpr (UseSparse) inst.cost.lengths.push_back(length);
        };

        if (oneway) {
            double zmin = static_cast<double>(-m.min_index());
            inst.vertex_supply[u] -= zmin;
            inst.vertex_supply[v] += zmin;
            inst.network.add_edge(edge_id, u, v);
            if (obj_opt) add_cost(edge_id, pwl_shift(*obj_opt, zmin));
            add_length();
            red.oneway_zmin.push_back(zmin);
            red.edge_to_road.push_back({road, +1});
            ++edge_id;
        } else {
            inst.network.add_edge(edge_id, u, v);
            if (obj_opt) add_cost(edge_id, *obj_opt);
            add_length();
            red.oneway_zmin.push_back(0.0);
            red.edge_to_road.push_back({road, +1});
            ++edge_id;

            inst.network.add_edge(edge_id, v, u);
            if (obj_opt) add_cost(edge_id, pwl_negate(*obj_opt));
            add_length();
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
template <typename Road, typename Vertex, bool UseSparse = true>
std::unordered_map<Road, int> compute_optimal_flow(
    RoadSegments<Road>&                                        segments,
    const std::unordered_map<Road, std::pair<Vertex,Vertex>>& endpoints,
    const std::unordered_map<Road, double>&                   lengths,
    const std::unordered_map<Road, bool>&                     is_oneway,
    double epsilon = 1.0
) {
    auto red = build_flow_reduction<Road, Vertex, UseSparse>(segments, endpoints, lengths, is_oneway);
    auto& inst = red.instance;
    int num_edges = static_cast<int>(red.edge_to_road.size());

    std::unordered_map<int, double> capacity;  // empty — no explicit capacity bounds
    auto int_flow = robust_mccf<UseSparse>(inst.network, capacity, inst.vertex_supply,
                                           inst.cost, inst.U, epsilon);

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
