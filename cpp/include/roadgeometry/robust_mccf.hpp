#pragma once
#include <unordered_map>
#include <variant>
#include <vector>

#include <functional>
#include <limits>

#include "concepts.hpp"
#include "fragile_mccf.hpp"
#include "fragile_mccf_sparse.hpp"
#include "hash_utils.hpp"
#include "input_graph.hpp"
#include "piecewise_linear.hpp"

namespace roadgeometry {

// ---------------------------------------------------------------------------
// RobustEdge<Edge>
//
// Edge type for RobustInputGraph.  A RobustEdge is either:
//   RegularEdge{e}   — an original edge from the wrapped graph
//   CycleEdge{index} — the augmenting cycle edge from node[index]
//                      to node[(index+1) % n]
// ---------------------------------------------------------------------------
template <typename Edge>
struct RegularEdge {
    Edge e;
    bool operator==(const RegularEdge& o) const { return e == o.e; }
};

struct CycleEdge {
    int index;
    bool operator==(const CycleEdge& o) const { return index == o.index; }
};

template <typename Edge>
using RobustEdge = std::variant<RegularEdge<Edge>, CycleEdge>;

// ---------------------------------------------------------------------------
// RobustInputGraph<G>
//
// A view wrapping any InputGraph G with an added directed Hamiltonian cycle
// over all nodes.  The cycle guarantees strong connectivity of every
// Delta-residual graph, satisfying fragile_mccf's precondition.
//
// DESIGN
// ------
// Templated on G — wraps any InputGraph without copying its structure.
// Nodes are unchanged; edges are RobustEdge<G::edge_type>.
//
// The Hamiltonian cycle visits nodes in the order they were added to
// node_order_ at construction time.  cycle edge k goes from
// node_order_[k] to node_order_[(k+1) % n].
//
// CONSTRUCTION
// ------------
//   RobustInputGraph(g)
//     Builds node_order_ (vector<Node>) and node_index_ (Node→int map)
//     from g.nodes() in iteration order.  O(n).
//
// QUERY
// -----
//   nodes()       → same range as g.nodes()
//   edges()       → g.edges() mapped to RegularEdge + CycleEdge{0..n-1}
//   out_edges(u)  → g.out_edges(u) mapped to RegularEdge
//                   + {CycleEdge{node_index_[u]}}
//   in_edges(u)   → g.in_edges(u) mapped to RegularEdge
//                   + {CycleEdge{(node_index_[u] - 1 + n) % n}}
//   endpoints(e)  → RegularEdge{e}: g.endpoints(e)
//                   CycleEdge{k}:   {node_order_[k], node_order_[(k+1)%n]}
//
// COST / CAPACITY (external, caller's responsibility)
// ----------------------------------------------------
// Cycle edges have no finite capacity (omit from capacity map).
// Cycle edge cost = prohibit, a linear function with slope CBOUND where
//   CBOUND = sum(cost_fn(U) for each edge with a cost function)
// Since any feasible flow has cost <= CBOUND, prohibitive cost ensures
// no cycle edge carries flow in the optimal solution.
//
// TODO: replace CBOUND-based prohibitive cost with an InfiniteSlope cost type
// that returns ±∞ directly, once fragile_mccf's linearize_cost_edge is updated
// to short-circuit the difference computation for infinite-slope functions.
//
// TODO: implement.
// ---------------------------------------------------------------------------
template <InputGraph G>
class RobustInputGraph {
public:
    using node_type = typename G::node_type;
    using edge_type = RobustEdge<typename G::edge_type>;

    explicit RobustInputGraph(const G& g) : g_(g) {
        for (const node_type& u : g.nodes()) {
            node_index_.emplace(u, (int)node_order_.size());
            node_order_.push_back(u);
        }
    }

    using inner_iter = std::ranges::iterator_t<decltype(std::declval<const G&>().edges())>;
    struct RegularPhase {
        inner_iter it;
        bool operator==(const RegularPhase& o) const { return it == o.it; }
    };
    struct CyclePhase {
        int index;
        bool operator==(const CyclePhase& o) const { return index == o.index; }
    };

    auto nodes() const { return g_.nodes(); }

    // EdgeIterator: forward iterator over RegularEdge{...} then CycleEdge{k}.
    struct EdgeIterator {
        using iterator_category = std::forward_iterator_tag;
        using value_type        = edge_type;
        using difference_type   = std::ptrdiff_t;
        using pointer           = void;
        using reference         = edge_type;  // returned by value

        std::variant<RegularPhase, CyclePhase> state_;
        inner_iter                              inner_end_;
        int                                     n_;

        edge_type operator*() const {
            return std::visit([](const auto& s) -> edge_type {
                if constexpr (std::is_same_v<std::decay_t<decltype(s)>, CyclePhase>)
                    return CycleEdge{s.index};
                else
                    return RegularEdge<typename G::edge_type>{*s.it};
            }, state_);
        }

        EdgeIterator& operator++() {
            if (auto* r = std::get_if<RegularPhase>(&state_)) {
                ++r->it;
                if (r->it == inner_end_)
                    state_ = CyclePhase{0};
            } else {
                ++std::get<CyclePhase>(state_).index;
            }
            return *this;
        }

        EdgeIterator operator++(int) { auto tmp = *this; ++(*this); return tmp; }

        bool operator==(const EdgeIterator& o) const { return state_ == o.state_; }
        bool operator!=(const EdgeIterator& o) const { return !(*this == o); }
    };

    struct EdgeRange {
        const RobustInputGraph& rig;
        EdgeIterator begin() const {
            auto r = rig.g_.edges();
            auto b = std::ranges::begin(r);
            auto e = std::ranges::end(r);
            int  n = (int)rig.node_order_.size();
            if (b == e) return EdgeIterator{CyclePhase{0}, e, n};  // empty graph: go straight to cycle
            return EdgeIterator{RegularPhase{b}, e, n};
        }
        EdgeIterator end() const {
            auto e = std::ranges::end(rig.g_.edges());
            int  n = (int)rig.node_order_.size();
            return EdgeIterator{CyclePhase{n}, e, n};  // cycle index == n means past-the-end
        }
    };

    EdgeRange edges() const { return EdgeRange{*this}; }

    // TODO: make out_edges and in_edges iterator-based (lazy range) rather than
    // materializing a vector, using the same RegularPhase/CyclePhase pattern as EdgeIterator.
    std::vector<edge_type> out_edges(const node_type& u) const {
        std::vector<edge_type> out;
        for (const auto& e : g_.out_edges(u))
            out.push_back(RegularEdge<typename G::edge_type>{e});
        out.push_back(CycleEdge{node_index_.at(u)});
        return out;
    }

    std::vector<edge_type> in_edges(const node_type& u) const {
        std::vector<edge_type> out;
        for (const auto& e : g_.in_edges(u))
            out.push_back(RegularEdge<typename G::edge_type>{e});
        int n = (int)node_order_.size();
        out.push_back(CycleEdge{(node_index_.at(u) - 1 + n) % n});
        return out;
    }

    std::pair<node_type, node_type> endpoints(const edge_type& e) const {
        return std::visit([&](const auto& s) -> std::pair<node_type, node_type> {
            if constexpr (std::is_same_v<std::decay_t<decltype(s)>, CycleEdge>) {
                int n = (int)node_order_.size();
                return {node_order_[s.index], node_order_[(s.index + 1) % n]};
            } else {
                return g_.endpoints(s.e);
            }
        }, e);
    }

private:
    const G&                                    g_;
    std::vector<node_type>                      node_order_;
    std::unordered_map<node_type, int>          node_index_;
};

static_assert(InputGraph<RobustInputGraph<HashMapGraph<int,int>>>);

// ---------------------------------------------------------------------------
// RobustCapacity<Edge, Cap>
//
// A capacity map view for a RobustInputGraph.  Regular edges delegate to the
// original capacity map; cycle edges have no finite capacity (find returns
// end(), so fragile_mccf will use the default of U).
// ---------------------------------------------------------------------------
template <typename Edge, typename Cap>
struct RobustCapacity {
    using key_type = RobustEdge<Edge>;
    using iterator = typename Cap::const_iterator;

    const Cap& cap_;

    iterator end() const { return cap_.end(); }

    iterator find(const key_type& e) const {
        if (const auto* r = std::get_if<RegularEdge<Edge>>(&e))
            return cap_.find(r->e);
        return end();  // cycle edges: no finite capacity
    }
};

// ---------------------------------------------------------------------------
// RobustCost<Edge, Cost>
//
// A cost map view for a RobustInputGraph.  Regular edges delegate to the
// original cost map; cycle edges return a prohibitive PiecewiseLinear with
// slope CBOUND = sum(cost_fn(U) for all edges with a cost function).
// ---------------------------------------------------------------------------
template <typename Edge, typename Cost>
struct RobustCost {
    using key_type = RobustEdge<Edge>;
    using CostFn   = std::function<double(double)>;
    using value_type = std::pair<key_type, CostFn>;

    // Iterator holds an optional value_type (nullopt = end).
    // operator-> returns const value_type*, giving access to ->second (the CostFn).
    // operator== compares by key only to avoid comparing std::function values.
    struct iterator {
        std::optional<value_type> entry_;  // nullopt → end()

        const value_type* operator->() const { return &*entry_; }

        bool operator==(const iterator& o) const {
            if (!entry_ && !o.entry_) return true;   // both end
            if (!entry_ || !o.entry_) return false;
            return entry_->first == o.entry_->first; // compare by key
        }
        bool operator!=(const iterator& o) const { return !(*this == o); }
    };

    const Cost& cost_;
    double      slope_;    // prohibitive slope; TODO: replace with InfiniteSlope type
    double      offset_;   // prohibitive offset (currently 0)

    // Takes the original (pre-wrapping) network to compute the prohibitive slope.
    // cost_.find(e) is always valid for every edge in the network (MatchingCostMap
    // guarantees this), so make_cbound needs no end() check.
    template <InputGraph G2>
    RobustCost(const G2& network, const Cost& cost, double U)
        : cost_(cost)
        , slope_(make_cbound(network, cost, U))
        , offset_(0.0)
    {}

    iterator end() const { return {std::nullopt}; }

    // Regular edges: delegate to inner cost (find() always valid; no end() branch).
    // Cycle edges: prohibitive linear fn with slope = sum of all edge costs at U.
    iterator find(const key_type& e) const {
        if (const auto* r = std::get_if<RegularEdge<Edge>>(&e)) {
            auto it = cost_.find(r->e);
            return {value_type{e, it->second}};
        }
        return {value_type{e, [s = slope_, o = offset_](double x) { return s * x + o; }}};
    }

    // Regular edges: delegate to inner cost.
    // Cycle edges: always false (prohibitive cost, not a real road arc).
    bool is_non_empty(const key_type& e) const {
        if (const auto* r = std::get_if<RegularEdge<Edge>>(&e))
            return cost_.is_non_empty(r->e);
        return false;
    }

private:
    template <InputGraph G2>
    static double make_cbound(const G2& network, const Cost& cost, double U) {
        double cbound = 0.0;
        for (const auto& e : network.edges()) {
            auto ci = cost.find(e);
            cbound += ci->second(U);
        }
        return cbound;
    }
};

// ---------------------------------------------------------------------------
// robust_mccf
//
// Wraps fragile_mccf with a RobustInputGraph + RobustCapacity + RobustCost
// to guarantee strong connectivity of every Delta-residual graph.
//
// Returns: flow map Edge → double (only original edges, cycle edges filtered out).
// ---------------------------------------------------------------------------
template <bool UseSparse = true, InputGraph G, typename Cap, typename Cost>
std::unordered_map<typename G::edge_type, double>
robust_mccf(
    const G&                                              network,
    const Cap&                                            capacity,
    const std::unordered_map<typename G::node_type, double>& supply,
    const Cost&                                           cost,
    double U,
    double epsilon    = 1.0,
    double cycle_tol  = 1e-9
) {
    using Edge = typename G::edge_type;

    RobustInputGraph<G>       robust_network(network);
    RobustCapacity<Edge, Cap> robust_capacity{capacity};
    RobustCost<Edge, Cost>    robust_cost{network, cost, U};

    auto flow = [&]() {
        if constexpr (UseSparse)
            return fragile_mccf_sparse(robust_network, robust_capacity, supply, robust_cost, U, epsilon);
        else
            return fragile_mccf(robust_network, robust_capacity, supply, robust_cost, U, epsilon);
    }();

    std::unordered_map<Edge, double> result;
    for (auto& [e, x] : flow) {
        if (const auto* r = std::get_if<RegularEdge<Edge>>(&e))
            result[r->e] = x;
        else if (std::abs(x) > cycle_tol)
            throw std::runtime_error("robust_mccf: infeasible instance (nonzero flow on cycle edge)");
    }
    return result;
}

} // namespace roadgeometry

namespace std {

template <typename Edge>
struct hash<roadgeometry::RegularEdge<Edge>> {
    std::size_t operator()(const roadgeometry::RegularEdge<Edge>& r) const noexcept {
        return std::hash<Edge>{}(r.e);
    }
};

template <>
struct hash<roadgeometry::CycleEdge> {
    std::size_t operator()(const roadgeometry::CycleEdge& c) const noexcept {
        return std::hash<int>{}(c.index);
    }
};

template <typename Edge>
struct hash<roadgeometry::RobustEdge<Edge>> {
    std::size_t operator()(const roadgeometry::RobustEdge<Edge>& e) const noexcept {
        std::size_t h = std::hash<std::size_t>{}(e.index());
        std::visit([&](const auto& alt) {
            h = roadgeometry::hash_combine(h, std::hash<std::decay_t<decltype(alt)>>{}(alt));
        }, e);
        return h;
    }
};

} // namespace std
