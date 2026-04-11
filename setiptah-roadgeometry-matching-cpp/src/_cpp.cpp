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
#include "roadgeometry/piecewise_linear.hpp"

namespace py = pybind11;
using namespace roadgeometry;

// Build a CostFn from a Python object: unwrap PiecewiseLinear directly,
// or wrap an arbitrary Python callable.
static std::function<double(double)> cost_fn_from_py(const py::object& obj)
{
    if (py::isinstance<PiecewiseLinear>(obj)) {
        // Copy the C++ object — no Python call overhead at evaluation time.
        PiecewiseLinear pwl = obj.cast<PiecewiseLinear>();
        return [pwl = std::move(pwl)](double x) { return pwl(x); };
    }
    return [obj](double x) -> double { return obj(x).cast<double>(); };
}

PYBIND11_MODULE(_cpp, m) {
    m.doc() = "C++ backend for roadgeometry matching";

    // ------------------------------------------------------------------
    // PiecewiseLinear
    //
    // Construct from a list of (left, slope, offset) tuples (sorted by left).
    // Callable as f(x) -> float from both Python and C++.
    // ------------------------------------------------------------------
    py::class_<PiecewiseLinear>(m, "PiecewiseLinear")
        .def(py::init([](const std::vector<std::tuple<double,double,double>>& segs) {
            std::vector<PiecewiseLinear::Segment> segments;
            segments.reserve(segs.size());
            for (auto& [left, slope, offset] : segs)
                segments.push_back({left, slope, offset});
            return PiecewiseLinear(std::move(segments));
        }), py::arg("segments"),
            "Construct from a list of (left, slope, offset) tuples sorted by left.")
        .def("__call__", &PiecewiseLinear::operator(), py::arg("x"));

    // ------------------------------------------------------------------
    // dijkstra(out_edges, endpoints, cost, source)
    // ------------------------------------------------------------------
    m.def("dijkstra", [](
        const std::vector<std::vector<int>>&    out_edges,
        const std::vector<std::pair<int, int>>& endpoints,
        const std::vector<double>&              cost,
        int                                      source
    ) {
        return dijkstra(out_edges, endpoints, cost, source);
    });

    // ------------------------------------------------------------------
    // fragile_mccf(out_edges, endpoints, supply, cost, U, epsilon)
    //
    //   cost : list[PiecewiseLinear | callable | None]
    //          PiecewiseLinear entries are unwrapped directly (no Python
    //          callback at evaluation time).  Other callables are wrapped
    //          in std::function.  None means zero cost.
    // ------------------------------------------------------------------
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

        HashMapGraph<int, int> network;
        for (int i = 0; i < n; ++i) network.add_node(i);
        for (int e = 0; e < m; ++e) {
            auto [u, v] = endpoints[e];
            network.add_edge(e, u, v);
        }

        std::unordered_map<int, double> supply;
        for (int i = 0; i < n; ++i)
            if (supply_arr[i] != 0.0) supply[i] = supply_arr[i];

        using CostFn = std::function<double(double)>;
        std::unordered_map<int, CostFn> cost;
        for (int e = 0; e < m; ++e) {
            if (!cost_arr[e].is_none())
                cost[e] = cost_fn_from_py(cost_arr[e]);
        }

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
