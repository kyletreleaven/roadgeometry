// TODO: downgrade C++ standard from C++20 to C++11/17 for broader compatibility.
// This affects: concepts, ranges, structured bindings, std::optional, if-constexpr, etc.

#include <functional>
#include <vector>
#include <utility>
#include <unordered_map>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "roadgeometry/dijkstra.hpp"
#include "roadgeometry/robust_mccf.hpp"
#include "roadgeometry/input_graph.hpp"
#include "roadgeometry/fragile_mccf.hpp"
#include "roadgeometry/fragile_mccf_sparse.hpp"
#include "roadgeometry/piecewise_linear.hpp"
#include "roadgeometry/segment.hpp"
#include "roadgeometry/optimal_flow.hpp"
#include "roadgeometry/matching.hpp"

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
template <>
struct equal_to<py::object> {
    bool operator()(const py::object& a, const py::object& b) const {
        return a.equal(b);
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
    // TODO: both fragile_mccf and robust_mccf bindings build a HashMapGraph<int,int>
    // from compact int arrays, which is wasteful — nodes are already 0..n-1.
    // Replace with a flat-array graph representation for better cache performance.

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

        auto flow_map = fragile_mccf_sparse(network, capacity, supply, cost, U, epsilon);

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
    // robust_mccf(out_edges, endpoints, supply, cost, capacity, U, epsilon)
    //
    // Same interface as fragile_mccf but wraps the network in a Hamiltonian
    // cycle to guarantee strong connectivity.  Returns only original edge flows
    // (cycle edges filtered out).  Raises RuntimeError if infeasible.
    // ------------------------------------------------------------------
    m.def("robust_mccf", [](
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

        auto flow_map = robust_mccf(network, capacity, supply, cost, U, epsilon);

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

    // ------------------------------------------------------------------
    // build_flow_reduction(segments, endpoints, lengths, is_oneway)
    //
    // Returns a dict with keys: supply (dict), U (float), num_edges (int),
    // edge_to_road (list of [road, sign]), oneway_zmin (list of float).
    // Useful for debugging: compare supply/U against the Python-built instance.
    // ------------------------------------------------------------------
    m.def("build_flow_reduction", [](
        const py::dict& segments_py,
        const py::dict& endpoints_py,
        const py::dict& lengths_py,
        const py::dict& is_oneway_py
    ) -> py::dict {
        using Road = py::object;
        using Vertex = py::object;

        RoadSegments<Road> segments;
        for (auto [road, seg_list] : segments_py) {
            std::vector<YGroup> groups;
            for (auto item : seg_list.cast<py::list>()) {
                auto t = item.cast<py::tuple>();
                YGroup g;
                g.y = t[0].cast<double>();
                auto ns = t[1];
                for (auto x : ns.attr("supply")) g.supply.push_back(x.cast<int>());
                for (auto x : ns.attr("demand")) g.demand.push_back(x.cast<int>());
                groups.push_back(std::move(g));
            }
            segments[road.cast<Road>()] = std::move(groups);
        }

        std::unordered_map<Road, std::pair<Vertex, Vertex>> endpoints;
        for (auto [road, uv] : endpoints_py) {
            auto t = uv.cast<py::tuple>();
            endpoints[road.cast<Road>()] = {t[0].cast<Vertex>(), t[1].cast<Vertex>()};
        }

        std::unordered_map<Road, double> lengths;
        for (auto [road, length] : lengths_py)
            lengths[road.cast<Road>()] = length.cast<double>();

        std::unordered_map<Road, bool> is_oneway;
        for (auto [road, ow] : is_oneway_py)
            is_oneway[road.cast<Road>()] = ow.cast<bool>();

        auto red = build_flow_reduction<Road, Vertex>(segments, endpoints, lengths, is_oneway);

        py::dict supply_py;
        for (auto& [v, s] : red.instance.vertex_supply)
            supply_py[v] = s;

        py::list edge_to_road_py;
        for (auto& [road, sign] : red.edge_to_road)
            edge_to_road_py.append(py::make_tuple(road, sign));

        py::list oneway_zmin_py;
        for (auto z : red.oneway_zmin)
            oneway_zmin_py.append(z);

        py::list cost_py;
        for (auto& pwl : red.instance.edge_cost)
            cost_py.append(pwl);

        py::list edge_endpoints_py;
        for (auto& [road, sign] : red.edge_to_road) {
            (void)sign;
            // endpoints indexed by edge_id — same order as edge_to_road
        }
        // expose network edge endpoints in edge_id order
        for (int e = 0; e < (int)red.edge_to_road.size(); ++e) {
            auto [u, v] = red.instance.network.endpoints(e);
            edge_endpoints_py.append(py::make_tuple(u, v));
        }

        py::dict result;
        result["supply"]          = supply_py;
        result["U"]               = red.instance.U;
        result["edge_to_road"]    = edge_to_road_py;
        result["oneway_zmin"]     = oneway_zmin_py;
        result["cost"]            = cost_py;
        result["edge_endpoints"]  = edge_endpoints_py;
        return result;
    },
    py::arg("segments"),
    py::arg("endpoints"),
    py::arg("lengths"),
    py::arg("is_oneway")
    );

    // ------------------------------------------------------------------
    // compute_optimal_flow(segments, endpoints, lengths, is_oneway, epsilon)
    //
    // segments  : dict[road, list[tuple[float, SimpleNamespace(supply, demand)]]]
    //             as returned by sort_and_segment (post-prematch)
    // endpoints : dict[road, (u, v)]
    // lengths   : dict[road, float]
    // is_oneway : dict[road, bool]
    //
    // Returns dict[road, float] — integer-valued optimal flow per road.
    // ------------------------------------------------------------------
    m.def("compute_optimal_flow", [](
        const py::dict& segments_py,
        const py::dict& endpoints_py,
        const py::dict& lengths_py,
        const py::dict& is_oneway_py,
        double epsilon,
        const std::string& solver
    ) -> py::dict {
        using Road = py::object;
        using Vertex = py::object;

        RoadSegments<Road> segments;
        for (auto [road, seg_list] : segments_py) {
            std::vector<YGroup> groups;
            for (auto item : seg_list.cast<py::list>()) {
                auto t = item.cast<py::tuple>();
                YGroup g;
                g.y = t[0].cast<double>();
                auto ns = t[1];
                for (auto x : ns.attr("supply")) g.supply.push_back(x.cast<int>());
                for (auto x : ns.attr("demand")) g.demand.push_back(x.cast<int>());
                groups.push_back(std::move(g));
            }
            segments[road.cast<Road>()] = std::move(groups);
        }

        std::unordered_map<Road, std::pair<Vertex, Vertex>> endpoints;
        for (auto [road, uv] : endpoints_py) {
            auto t = uv.cast<py::tuple>();
            endpoints[road.cast<Road>()] = {t[0].cast<Vertex>(), t[1].cast<Vertex>()};
        }

        std::unordered_map<Road, double> lengths;
        for (auto [road, length] : lengths_py)
            lengths[road.cast<Road>()] = length.cast<double>();

        std::unordered_map<Road, bool> is_oneway;
        for (auto [road, ow] : is_oneway_py)
            is_oneway[road.cast<Road>()] = ow.cast<bool>();

        py::dict result;
        auto emit = [&](auto flow) {
            for (auto& [road, x] : flow) result[road] = x;
        };
        if (solver == "sparse")
            emit(compute_optimal_flow<Road, Vertex, true>(segments, endpoints, lengths, is_oneway, epsilon));
        else
            emit(compute_optimal_flow<Road, Vertex, false>(segments, endpoints, lengths, is_oneway, epsilon));
        return result;
    },
    py::arg("segments"),
    py::arg("endpoints"),
    py::arg("lengths"),
    py::arg("is_oneway"),
    py::arg("epsilon") = 1.0,
    py::arg("solver") = "sparse"
    );

    m.def("compute_matching", [](
        const py::list& P_py,
        const py::list& Q_py,
        const py::dict& endpoints_py,
        const py::dict& lengths_py,
        const py::dict& is_oneway_py,
        const std::string& solver
    ) -> py::tuple {
        using Road   = py::object;
        using Vertex = py::object;

        auto to_points = [](const py::list& pts) {
            std::vector<std::pair<Road, double>> out;
            out.reserve(pts.size());
            for (auto item : pts) {
                auto t = item.cast<py::tuple>();
                out.push_back({t[0].cast<Road>(), t[1].cast<double>()});
            }
            return out;
        };

        auto P = to_points(P_py);
        auto Q = to_points(Q_py);

        std::unordered_map<Road, std::pair<Vertex, Vertex>> endpoints;
        for (auto [road, uv] : endpoints_py) {
            auto t = uv.cast<py::tuple>();
            endpoints[road.cast<Road>()] = {t[0].cast<Vertex>(), t[1].cast<Vertex>()};
        }

        std::unordered_map<Road, double> lengths;
        for (auto [road, length] : lengths_py)
            lengths[road.cast<Road>()] = length.cast<double>();

        std::unordered_map<Road, bool> is_oneway;
        for (auto [road, ow] : is_oneway_py)
            is_oneway[road.cast<Road>()] = ow.cast<bool>();

        auto run = [&](auto result) -> py::tuple {
            auto& [matching, cost] = result;
            py::list matching_py;
            for (auto& [i, j] : matching)
                matching_py.append(py::make_tuple(i, j));
            return py::make_tuple(matching_py, cost);
        };

        if (solver == "sparse")
            return run(compute_matching<Road, Vertex, true>(P, Q, endpoints, lengths, is_oneway));
        else
            return run(compute_matching<Road, Vertex, false>(P, Q, endpoints, lengths, is_oneway));
    },
    py::arg("P"),
    py::arg("Q"),
    py::arg("endpoints"),
    py::arg("lengths"),
    py::arg("is_oneway"),
    py::arg("solver") = "sparse"
    );
}
