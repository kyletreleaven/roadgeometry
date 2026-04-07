#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "roadgeometry/dijkstra.hpp"

namespace py = pybind11;
using namespace roadgeometry;

PYBIND11_MODULE(_cpp, m) {
    m.doc() = "C++ backend for roadgeometry matching";

    // dijkstra(out_edges, endpoints, cost, source)
    //   out_edges  : list[list[int]]         — out_edges[node] = [edge_id, ...]
    //   endpoints  : list[tuple[int, int]]   — endpoints[edge] = (tail, head)
    //   cost       : list[float]             — cost[edge]
    //   source     : int
    // Returns (distances: dict[int, float], upstream: dict[int, int])
    m.def("dijkstra", [](
        const std::vector<std::vector<int>>&      out_edges,
        const std::vector<std::pair<int, int>>&   endpoints,
        const std::vector<double>&                cost,
        int                                        source
    ) {
        IntGraph g;
        g.out_edges_ = out_edges;
        g.endpoints_ = endpoints;

        auto [dist, upstream] = dijkstra(g, cost, source);
        return py::make_tuple(dist, upstream);
    });
}
