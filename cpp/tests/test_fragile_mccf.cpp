#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "third_party/doctest/doctest.h"

#include <functional>
#include <unordered_map>

#include "roadgeometry/input_graph.hpp"
#include "roadgeometry/fragile_mccf.hpp"
#include "roadgeometry/robust_mccf.hpp"
#include "support.hpp"

using Graph   = roadgeometry::HashMapGraph<int, int>;
using CostMap = std::unordered_map<int, std::function<double(double)>>;

// Harness-proving test: a real solve through robust_mccf on a tiny instance,
// checked with the framework-free feasibility oracle + a known optimum.
TEST_CASE("robust_mccf: unit flow across a 2-cycle is feasible and optimal") {
    Graph g;
    g.add_edge(0, 0, 1);   // road 0: 0 -> 1  (cheap)
    g.add_edge(1, 1, 0);   // road 1: 1 -> 0  (expensive)

    std::unordered_map<int, double> capacity{{0, 5.0}, {1, 5.0}};
    std::unordered_map<int, double> supply {{0, +1.0}, {1, -1.0}};
    CostMap cost{
        {0, [](double x) { return 1.0 * x; }},
        {1, [](double x) { return 5.0 * x; }},
    };

    auto flow = roadgeometry::robust_mccf</*UseSparse=*/false>(
        g, capacity, supply, cost, /*U=*/1.0);

    CHECK(rgtest::feasibility_violation(g, supply, flow) == "");
    CHECK(flow.at(0) == doctest::Approx(1.0));   // 1 unit along the cheap arc
    CHECK(flow.at(1) == doctest::Approx(0.0));
    CHECK(rgtest::total_cost(g, flow, cost) == doctest::Approx(1.0));
}
