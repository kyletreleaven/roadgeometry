#include <functional>
#include <vector>
#include <utility>
#include <unordered_map>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "roadgeometry/dijkstra.hpp"
#include "roadgeometry/input_graph.hpp"
#include "roadgeometry/fragile_mccf.hpp"

namespace py = pybind11;
using namespace roadgeometry;

PYBIND11_MODULE(_cpp, m) {
    m.doc() = "C++ backend for roadgeometry matching";

    // dijkstra(out_edges, endpoints, cost, source)
    //
    //   out_edges : list[list[int]]        — out_edges[node] = [edge_id, ...]
    //   endpoints : list[tuple[int, int]]  — endpoints[edge] = (tail, head)
    //   cost      : list[float]            — cost[edge]
    //   source    : int
    //
    // Returns (dist, upstream) as parallel lists indexed by node int:
    //   dist[i]     = shortest distance to node i (inf if unreachable)
    //   upstream[i] = edge_id on shortest path to i (-1 if source or unreachable)
    m.def("dijkstra", [](
        const std::vector<std::vector<int>>&    out_edges,
        const std::vector<std::pair<int, int>>& endpoints,
        const std::vector<double>&              cost,
        int                                      source
    ) {
        return dijkstra(out_edges, endpoints, cost, source);
    });

    // fragile_mccf(out_edges, endpoints, supply, cost, U, epsilon) -> list[float]
    //
    //   out_edges : list[list[int]]         — out_edges[node] = [edge_id, ...]
    //   endpoints : list[tuple[int, int]]   — endpoints[edge] = (tail, head)
    //   supply    : list[float]             — supply[node], indexed 0..n-1
    //   cost      : list[callable | None]   — cost[edge](x) -> float, or None for zero cost
    //   U         : float                   — capacity-scaling initial width
    //   epsilon   : float                   — final phase width (default 1.0)
    //
    // Returns flow as list[float] indexed by edge int.
    //
    // Preconditions (caller's responsibility):
    //   - supply is conservative (sums to zero)
    //   - every Delta-residual graph is strongly connected
    m.def("fragile_mccf", [](
        const std::vector<std::vector<int>>&    out_edges,
        const std::vector<std::pair<int, int>>& endpoints,
        const std::vector<double>&              supply_arr,
        const std::vector<py::object>&          cost_arr,
        double U,
        double epsilon
    ) -> std::vector<double> {
        int n = static_cast<int>(out_edges.size());
        int m = static_cast<int>(endpoints.size());

        // Build HashMapGraph<int, int>
        HashMapGraph<int, int> network;
        for (int i = 0; i < n; ++i) network.add_node(i);
        for (int e = 0; e < m; ++e) {
            auto [u, v] = endpoints[e];
            network.add_edge(e, u, v);
        }

        // Supply map (skip zero entries)
        std::unordered_map<int, double> supply;
        for (int i = 0; i < n; ++i)
            if (supply_arr[i] != 0.0) supply[i] = supply_arr[i];

        // Cost map: wrap Python callables as std::function<double(double)>
        using CostFn = std::function<double(double)>;
        std::unordered_map<int, CostFn> cost;
        for (int e = 0; e < m; ++e) {
            if (!cost_arr[e].is_none()) {
                py::object fn = cost_arr[e];
                cost[e] = [fn](double x) -> double {
                    return fn(x).cast<double>();
                };
            }
        }

        // Empty capacity map: fragile_mccf treats missing entries as infinity.
        std::unordered_map<int, double> capacity;

        auto flow_map = fragile_mccf(network, capacity, supply, cost, U, epsilon);

        std::vector<double> flow_arr(m, 0.0);
        for (auto& [e, x] : flow_map)
            flow_arr[e] = x;
        return flow_arr;
    },
    py::arg("out_edges"),
    py::arg("endpoints"),
    py::arg("supply"),
    py::arg("cost"),
    py::arg("U"),
    py::arg("epsilon") = 1.0
    );
}
