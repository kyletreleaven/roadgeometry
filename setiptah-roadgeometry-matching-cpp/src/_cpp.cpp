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
#include "roadgeometry/segment.hpp"

namespace py = pybind11;
using namespace pybind11::literals;
using namespace roadgeometry;

// std::hash specialization for py::object — delegates to Python's hash().
namespace std {
template <>
struct hash<py::object> {
    size_t operator()(const py::object& obj) const {
        return py::hash(obj);
    }
};
} // namespace std

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
        const std::vector<double>&              capacity_arr,
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
        for (int e = 0; e < m; ++e)
            if (std::isfinite(capacity_arr[e]))
                capacity[e] = capacity_arr[e];

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
    py::arg("capacity"),
    py::arg("U"),
    py::arg("epsilon") = 1.0
    );

    // ------------------------------------------------------------------
    // sort_and_segment(P, Q, road_ids)
    //
    // P, Q     : list of (road_id, float y); road_id is any comparable+hashable
    // road_ids : all road ids in the network (pre-populates result keys)
    //
    // Returns dict[road_id, list[tuple[float, SimpleNamespace(supply, demand)]]]
    // where supply and demand are collections.deque[int] — drop-in for the
    // Python BiPartite[deque] returned by compute_segments2.
    // ------------------------------------------------------------------
    m.def("sort_and_segment", [](
        const py::sequence& P,
        const py::sequence& Q,
        const py::sequence& road_ids
    ) {
        std::vector<std::pair<py::object, double>> P_cpp, Q_cpp;
        P_cpp.reserve(py::len(P));
        Q_cpp.reserve(py::len(Q));
        for (auto p : P) {
            auto t = p.cast<py::tuple>();
            P_cpp.push_back({t[0].cast<py::object>(), t[1].cast<double>()});
        }
        for (auto q : Q) {
            auto t = q.cast<py::tuple>();
            Q_cpp.push_back({t[0].cast<py::object>(), t[1].cast<double>()});
        }

        auto segs = sort_and_segment<py::object>(P_cpp, Q_cpp);

        auto deque_cls = py::module_::import("collections").attr("deque");
        auto ns_cls    = py::module_::import("types").attr("SimpleNamespace");

        py::dict out;
        for (auto r : road_ids)
            out[r] = py::list();

        for (auto& [road, groups] : segs) {
            py::list seg;
            for (auto& g : groups) {
                py::object qs = ns_cls(
                    "supply"_a = deque_cls(py::cast(g.supply)),
                    "demand"_a = deque_cls(py::cast(g.demand))
                );
                seg.append(py::make_tuple(g.y, qs));
            }
            out[road] = seg;
        }
        return out;
    },
    py::arg("P"),
    py::arg("Q"),
    py::arg("road_ids")
    );
}
