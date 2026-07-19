#pragma once
// Framework-free test support: depends on NO test framework (see /testing.md,
// "Avoiding lock-in"). Returns plain values so the framework layer is a thin
// shell of CHECK(...) over these helpers.
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>

namespace rgtest {

template <typename Map, typename Key>
double map_get(const Map& m, const Key& k, double def) {
    auto it = m.find(k);
    return it != m.end() ? it->second : def;
}

// Returns "" if `flow` is feasible for the instance, else a human-readable
// reason. Checks flow conservation (supply + inflow - outflow == 0 at every
// node) and lb[e] <= flow[e] <= ub[e] (defaults: lb = 0, ub = +inf).
template <typename G, typename Supply, typename Flow,
          typename Bound = std::unordered_map<typename G::edge_type, double>>
std::string feasibility_violation(
    const G& network, const Supply& supply, const Flow& flow,
    const Bound& lb = {}, const Bound& ub = {}, double tol = 1e-6)
{
    for (const auto& i : network.nodes()) {
        double net = map_get(supply, i, 0.0);
        for (const auto& e : network.in_edges(i))  net += flow.at(e);
        for (const auto& e : network.out_edges(i)) net -= flow.at(e);
        if (std::abs(net) > tol)
            return "conservation violated (net=" + std::to_string(net) + ")";
    }
    for (const auto& e : network.edges()) {
        double x  = flow.at(e);
        double lo = map_get(lb, e, 0.0);
        double hi = map_get(ub, e, std::numeric_limits<double>::infinity());
        if (x < lo - tol) return "flow below lb on an edge";
        if (x > hi + tol) return "flow above ub on an edge";
    }
    return "";
}

template <typename G, typename Flow, typename Cost>
double total_cost(const G& network, const Flow& flow, const Cost& cost) {
    double c = 0.0;
    for (const auto& e : network.edges()) {
        auto it = cost.find(e);
        if (it != cost.end()) c += it->second(flow.at(e));
    }
    return c;
}

} // namespace rgtest
