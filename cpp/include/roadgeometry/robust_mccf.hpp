#pragma once
#include <limits>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "concepts.hpp"
#include "fragile_mccf.hpp"
#include "hash_utils.hpp"
#include "input_graph.hpp"
#include "mccf/concepts.hpp"   // mccf::Instance, node_t, edge_t
#include "mccf/traits.hpp"     // mccf::map_backed_instance

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

} // namespace roadgeometry

// std::hash for the RobustEdge alternatives — defined here, right after the types
// and BEFORE any InputGraph<RobustInputGraph<...>> check. Tightened InputGraph
// requires Hashable<edge_type>, and Hashable needs the specialization *complete* at
// the check (a forward declaration won't satisfy the concept), so these must
// precede RobustInputGraph's static_assert and RobustInstance below.
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

namespace roadgeometry {

// ---------------------------------------------------------------------------
// RobustInputGraph<G>
//
// A view wrapping any InputGraph G with an added directed Hamiltonian cycle
// over all nodes.  The cycle guarantees strong connectivity of every
// Delta-residual graph, satisfying fragile_mccf's precondition.
//
// Templated on G — wraps any InputGraph without copying its structure. Nodes are
// unchanged; edges are RobustEdge<G::edge_type>. The Hamiltonian cycle visits
// nodes in node_order_ (g.nodes() iteration order); cycle edge k goes from
// node_order_[k] to node_order_[(k+1) % n].
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
// RobustInstance<Base>
//
// An Instance adapter that wraps a base Instance and adds a Hamiltonian cycle,
// guaranteeing strong connectivity of every Delta-residual graph (fragile_mccf's
// precondition). It OWNS the base (move it in) so every accessor borrows from
// storage this object owns — no cost fn is ever copied.
//
// This rolls the old RobustCapacity / RobustCost views into accessors:
//   network()  → a RobustInputGraph over base.network()
//   cost(e)    → regular: base.cost(inner);  cycle: a prohibitive linear cost
//                (slope = CBOUND = sum base.cost(e)(U); any feasible flow costs
//                 <= CBOUND, so no cycle edge carries flow in the optimum)
//   ub(e)      → regular: base.ub(inner);    cycle: +inf (no finite capacity)
//   lb(e)      → regular: base.lb(inner);    cycle: 0
//   supply(n)  → base.supply(n)
// ---------------------------------------------------------------------------
template <mccf::Instance Base>
class RobustInstance {
public:
    using base_network = typename Base::network_type;
    using base_edge    = typename base_network::edge_type;

    using network_type = RobustInputGraph<base_network>;
    using node_type    = typename network_type::node_type;
    using edge_type    = typename network_type::edge_type;   // RobustEdge<base_edge>

    // Prohibitive linear cost for cycle edges: slope * x (slope = CBOUND).
    // TODO: make the cycle-edge cost a swappable ConnectivityCostPolicy — CBOUND
    // prohibitive (this) OR an ordinal / infinite-slope cost — rather than hard-coding
    // one. Cost representation is a separate policy from connectivity topology; see
    // plans/connectivity.md and plans/ordinal_costs.md. The ordinal/infinite option
    // additionally needs linearize_cost_edge to short-circuit the difference computation
    // for infinite-slope functions.
    struct Prohibitive {
        double slope;
        double operator()(double x) const { return slope * x; }
    };

    // The base cost's per-edge fn type (base.cost(e) returns it by const reference).
    using base_cost_fn = std::remove_cvref_t<
        decltype(std::declval<const Base&>().cost(std::declval<base_edge>()))>;

    // Borrowed, invocable cost handle: a pointer to the base cost fn (regular edges) or
    // to the shared prohibitive_ member (cycle edges). Both borrowed, both point into
    // stable storage (the base's owned map / the prohibitive_ member) — one visit, no
    // handle wrapping, no fn copy. (base.cost returns a const& into the base's map, so we
    // point straight at it rather than composing through another handle.)
    struct CostRef {
        std::variant<const base_cost_fn*, const Prohibitive*> v;
        double operator()(double x) const {
            return std::visit([x](const auto* c) -> double { return (*c)(x); }, v);
        }
    };

    RobustInstance(Base base, double U)
        : base_(std::move(base))
        , graph_(base_.network())                       // borrows base_.network() (base_ owned → stable)
        , prohibitive_{make_cbound(base_, U)}
    {}

    const network_type& network() const { return graph_; }

    CostRef cost(edge_type e) const {
        if (const auto* r = std::get_if<RegularEdge<base_edge>>(&e))
            return CostRef{ &base_.cost(r->e) };
        return CostRef{ &prohibitive_ };
    }

    double ub(edge_type e) const {
        if (const auto* r = std::get_if<RegularEdge<base_edge>>(&e))
            return base_.ub(r->e);
        return std::numeric_limits<double>::infinity();   // cycle: no finite capacity
    }
    double lb(edge_type e) const {
        if (const auto* r = std::get_if<RegularEdge<base_edge>>(&e))
            return base_.lb(r->e);
        return 0.0;   // cycle
    }
    double supply(node_type n) const { return base_.supply(n); }

private:
    Base         base_;         // owned — declared first so graph_/prohibitive_ can borrow it
    network_type graph_;
    Prohibitive  prohibitive_;

    static double make_cbound(const Base& base, double U) {
        double cbound = 0.0;
        for (const base_edge& e : base.network().edges())
            cbound += base.cost(e)(U);
        return cbound;
    }
};

template <mccf::Instance Base>
RobustInstance<Base> robust_instance(Base base, double U) {
    return RobustInstance<Base>(std::move(base), U);
}

// ---------------------------------------------------------------------------
// robust_mccf
//
// Robustify an Instance (wrap it in a RobustInstance to guarantee connectivity),
// solve with fragile_mccf, and filter the cycle-edge flow back out. Symmetric with
// fragile_mccf: an Instance core plus an unpacked (plain-maps) shim over it.
//
// Returns: flow map Edge → double (original edges only; cycle edges filtered out).
// ---------------------------------------------------------------------------
template <mccf::Instance Base>
std::unordered_map<mccf::edge_t<Base>, double>
robust_mccf(Base base, double U, double epsilon = 1.0, double cycle_tol = 1e-9)
{
    using Edge = mccf::edge_t<Base>;

    auto st = fragile_mccf_state(robust_instance(std::move(base), U), U, epsilon);

    std::unordered_map<Edge, double> result;
    for (auto& [e, x] : st.flow) {
        if (const auto* r = std::get_if<RegularEdge<Edge>>(&e))
            result[r->e] = x;
        else if (std::abs(x) > cycle_tol)
            throw std::runtime_error("robust_mccf: infeasible instance (nonzero flow on cycle edge)");
    }
    return result;
}

// Unpacked shim: build a map_backed_instance over the plain maps and delegate to
// the Instance core above.
//
// NOTE: the UseSparse template parameter is retained for binding compatibility but
// currently ignored — the sparse solver is temporarily unsupported (see TODO.md).
// The path is always dense.
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
    return robust_mccf(
        mccf::map_backed_instance(
            network, cost, capacity,
            std::unordered_map<typename G::edge_type, double>{},   // lb = {} (>= 0 ⇒ 0 default)
            supply),
        U, epsilon, cycle_tol);
}

} // namespace roadgeometry
