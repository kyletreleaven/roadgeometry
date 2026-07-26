#include "third_party/doctest/doctest.h"

#include <functional>
#include <unordered_map>
#include <utility>

#include "roadgeometry/input_graph.hpp"
#include "roadgeometry/mccf/concepts.hpp"
#include "roadgeometry/mccf/traits.hpp"

namespace mccf = roadgeometry::mccf;

using Graph   = roadgeometry::HashMapGraph<int, int>;
using CostMap = std::unordered_map<int, std::function<double(double)>>;

TEST_CASE("mccf::map_backed_instance models Instance and exposes the problem data") {
    Graph g;
    g.add_edge(0, 0, 1);

    auto inst = mccf::map_backed_instance(
        g,
        CostMap{ {0, [](double x) { return 2.0 * x; }} },
        std::unordered_map<int, double>{{0, 5.0}},         // ub
        std::unordered_map<int, double>{{0, -1.0}},        // lb
        std::unordered_map<int, double>{{0, +1.0}, {1, -1.0}});  // supply

    // The default model satisfies the concept (compile-time contract check).
    static_assert(mccf::Instance<decltype(inst)>);

    CHECK(inst.cost(0)(3.0) == doctest::Approx(6.0));   // retrieve the cost callable, evaluate at x=3
    CHECK(inst.ub(0)     == doctest::Approx(5.0));
    CHECK(inst.lb(0)     == doctest::Approx(-1.0));
    CHECK(inst.supply(0) == doctest::Approx(1.0));
    CHECK(inst.supply(1) == doctest::Approx(-1.0));
    CHECK(inst.network().endpoints(0) == std::pair<int, int>{0, 1});

    // Defaults for missing keys: ub -> +inf, lb -> 0, supply -> 0. (cost has NO
    // fallback — the cost map must be total — so it is only ever queried for a
    // present edge, not exercised here.)
    Graph g2;
    g2.add_edge(7, 0, 1);
    auto bare = mccf::map_backed_instance(
        g2, CostMap{}, std::unordered_map<int, double>{},
        std::unordered_map<int, double>{}, std::unordered_map<int, double>{});
    CHECK(bare.lb(7) == doctest::Approx(0.0));
    CHECK(bare.supply(0) == doctest::Approx(0.0));
}
