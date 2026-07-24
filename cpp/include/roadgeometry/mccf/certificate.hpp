#pragma once
#include <cmath>
#include <string>

#include "roadgeometry/mccf/concepts.hpp"
#include "roadgeometry/mccf/detail.hpp"   // map_get
#include "roadgeometry/mccf/traits.hpp"   // has_lower_bounds_v

namespace roadgeometry::mccf {

// ---------------------------------------------------------------------------
// OptimalityCertificate — a non-owning view of a {flow, potential} pair.
//
// Cheap to pass (two references); valid only while the underlying data is alive
// and unmutated. The dual/optimality certificate for min convex-cost flow:
// node potentials making every residual reduced cost >= 0.
// ---------------------------------------------------------------------------
template <class Flow, class Potential>
struct OptimalityCertificate {
    const Flow& flow;
    const Potential& potential;
};

template <class Flow, class Potential>
OptimalityCertificate<Flow, Potential> certificate_of(const Flow& f, const Potential& p) {
    return {f, p};
}

// ---------------------------------------------------------------------------
// certifies_optimality(instance, certificate) -> "" iff the certificate proves
// optimality: the flow is feasible (conservation + [lb, ub] bounds) AND every
// available unit-residual arc has reduced cost >= -tol under the potentials.
// Else a human-readable reason. Needs no reference solver.
//
// On an lb ≡ 0 instance (has_lower_bounds_v<I> == false) the lb path is compiled
// out: no lb() call, no subtraction against a nonzero lower bound.
// ---------------------------------------------------------------------------
// `epsilon` is the solver's finest scale (residual step); default 1.0 = integer
// optimality. Marginals are taken per-unit so they match the solver's potentials.
//
// TODO: the std::string return ("" = ok, else a reason) is stringly-typed — a
// bool + message that surfaces only the FIRST violation and can't be inspected
// programmatically. Replace with a structured, lazily-iterable set of violations
// (each: kind {conservation, bounds, reduced-cost}, the edge/node, direction,
// amount). Empty range = optimal; callers short-circuit or enumerate all.
template <Instance I,
          KeyMap<edge_t<I>> Flow,        // flow keyed by I's edges
          KeyMap<node_t<I>> Potential>   // potentials keyed by I's nodes
std::string certifies_optimality(const I& inst,
                                 const OptimalityCertificate<Flow, Potential>& cert,
                                 double epsilon = 1.0,
                                 double tol = 1e-6)
{
    using detail::map_get;
    const auto& g    = inst.network();
    const auto& flow = cert.flow;
    const auto& pot  = cert.potential;

    // Feasibility: flow conservation at every node.
    for (auto n : g.nodes()) {
        double net = inst.supply(n);
        for (auto e : g.in_edges(n))  net += map_get(flow, e, 0.0);
        for (auto e : g.out_edges(n)) net -= map_get(flow, e, 0.0);
        if (std::abs(net) > tol) return "conservation violated at a node";
    }

    // Feasibility (bounds) + optimality (reduced costs) per edge.
    for (auto e : g.edges()) {
        auto [i, j] = g.endpoints(e);
        double x  = map_get(flow, e, 0.0);
        double ub = inst.ub(e);
        double lb = 0.0;
        if constexpr (has_lower_bounds_v<I>) lb = inst.lb(e);

        if (x < lb - tol) return "flow below lb on an edge";
        if (x > ub + tol) return "flow above ub on an edge";

        double pi_i = map_get(pot, i, 0.0);
        double pi_j = map_get(pot, j, 0.0);
        const auto& c = inst.cost(e);   // retrieve once; evaluate below

        if (x + epsilon <= ub + tol) {   // forward residual arc i->j available
            double m = (c(x + epsilon) - c(x)) / epsilon;   // per-unit marginal
            if (m + pi_j - pi_i < -tol)
                return "negative reduced cost on a forward residual arc";
        }
        if (x - epsilon >= lb - tol) {   // backward residual arc j->i available
            double m = (c(x - epsilon) - c(x)) / epsilon;
            if (m + pi_i - pi_j < -tol)
                return "negative reduced cost on a backward residual arc";
        }
    }
    return "";
}

} // namespace roadgeometry::mccf
