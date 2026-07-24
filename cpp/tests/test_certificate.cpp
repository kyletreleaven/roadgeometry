#include "third_party/doctest/doctest.h"

#include <functional>
#include <unordered_map>

#include "roadgeometry/input_graph.hpp"
#include "roadgeometry/mccf/concepts.hpp"
#include "roadgeometry/mccf/traits.hpp"
#include "roadgeometry/mccf/certificate.hpp"

namespace mccf = roadgeometry::mccf;

using Graph   = roadgeometry::HashMapGraph<int, int>;
using CostMap = std::unordered_map<int, std::function<double(double)>>;
using DMap    = std::unordered_map<int, double>;

// Single road 0->1, convex cost c(x)=x, bounds [0,5], move one unit 0->1.
// Optimal flow x0=1; potentials pi=(1,0) make both residual arcs tight.
TEST_CASE("certificate: optimal {flow, pi} accepted; wrong pi rejected") {
    Graph g;
    g.add_edge(0, 0, 1);
    auto inst = mccf::map_backed_instance(
        g, CostMap{ {0, [](double x) { return x; }} },
        DMap{{0, 5.0}}, DMap{{0, 0.0}}, DMap{{0, +1.0}, {1, -1.0}});

    DMap flow{{0, 1.0}};
    DMap good{{0, 1.0}, {1, 0.0}};
    DMap bad {{0, 0.0}, {1, 0.0}};   // not certifying: backward residual arc goes negative

    CHECK(mccf::certifies_optimality(inst, mccf::certificate_of(flow, good)) == "");
    CHECK(mccf::certifies_optimality(inst, mccf::certificate_of(flow, bad)) != "");
}

// A hand-written zero-lb instance model: writes only the kernel
// (network/cost/ub/supply) and inherits lb()=0 + has_lower_bounds=false from the
// mixin. Exercises the has_lower_bounds_v == false `if constexpr` fast path.
struct ZeroLbInstance : mccf::ZeroLowerBounds<int> {
    using network_type = Graph;
    using node_type    = int;
    using edge_type    = int;
    Graph g;
    const Graph& network() const { return g; }
    auto cost(int) const { return [](double x) { return x; }; }   // returns an invocable
    double ub(int) const { return 5.0; }
    double supply(int n) const { return n == 0 ? +1.0 : (n == 1 ? -1.0 : 0.0); }
};

TEST_CASE("certificate: zero-lb instance (mixin) certifies via the fast path") {
    static_assert(mccf::Instance<ZeroLbInstance>);
    static_assert(!mccf::has_lower_bounds_v<ZeroLbInstance>);   // tag from the mixin

    ZeroLbInstance inst;
    inst.g.add_edge(0, 0, 1);

    DMap flow{{0, 1.0}};
    DMap pot {{0, 1.0}, {1, 0.0}};
    CHECK(mccf::certifies_optimality(inst, mccf::certificate_of(flow, pot)) == "");
}

// epsilon is really used: a strictly-convex cost makes the per-unit marginal
// depend on the step, so the same {flow, pi} certifies at one epsilon and not
// another. Guards against epsilon silently becoming a dead parameter again.
TEST_CASE("certificate: epsilon changes the verdict (guards a dead parameter)") {
    Graph g;
    g.add_edge(0, 0, 1);
    auto inst = mccf::map_backed_instance(
        g, CostMap{ {0, [](double x) { return x * x; }} },   // strictly convex
        DMap{{0, 10.0}}, DMap{{0, 0.0}}, DMap{{0, +2.0}, {1, -2.0}});

    DMap flow{{0, 2.0}};
    DMap pot {{0, 5.0}, {1, 0.0}};
    auto cert = mccf::certificate_of(flow, pot);

    // forward per-unit marginal at x=2 is 4 + eps, so forward reduced cost = eps - 1.
    CHECK(mccf::certifies_optimality(inst, cert, /*epsilon=*/1.0) == "");   // eps-1 = 0
    CHECK(mccf::certifies_optimality(inst, cert, /*epsilon=*/0.5) != "");   // eps-1 = -0.5 < 0
}
