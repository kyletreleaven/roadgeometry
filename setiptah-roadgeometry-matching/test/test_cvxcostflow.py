import logging

import networkx as nx
import numpy as np
import pytest

from setiptah.roadgeometry.matching.util.mygraph import mygraph
from setiptah.roadgeometry.matching.nxopt.cvxcostflow import (
    MinConvexCostFlow,
    CppMinConvexCostFlow,
)

LOG = logging.getLogger(__name__)

FLOW_SOLVERS = {
    "py":  MinConvexCostFlow,
    "cpp": CppMinConvexCostFlow,
}


@pytest.mark.parametrize("solver_name", list(FLOW_SOLVERS))
def test_capacity_respected(solver_name):
    """Solver must not exceed finite edge capacities.

    Two parallel edges 0→1:
      e_cheap : cost x,    capacity 1
      e_dear  : cost 10x,  capacity ∞

    Supply 2 at node 0, demand 2 at node 1.
    Optimal: 1 unit on e_cheap (at capacity), 1 unit on e_dear.
    Any solver that ignores capacity will push 2 units on e_cheap.
    """
    solver = FLOW_SOLVERS[solver_name]

    g = mygraph()
    g.add_edge("e_cheap", 0, 1)
    g.add_edge("e_dear",  0, 1)

    supply   = {0: 2.0, 1: -2.0}
    capacity = {"e_cheap": 1.0}
    cost     = {
        "e_cheap": lambda x: x,
        "e_dear":  lambda x: 10.0 * x,
    }
    U = 4.0

    flow = solver(g, capacity, supply, cost, U)

    assert flow.get("e_cheap", 0.0) <= 1.0 + 1e-9, "capacity violated on e_cheap"
    assert flow.get("e_dear",  0.0) >= 1.0 - 1e-9, "1 unit must use e_dear"
    total = flow.get("e_cheap", 0.0) + flow.get("e_dear", 0.0)
    assert abs(total - 2.0) < 1e-9, "flow must satisfy supply"


@pytest.mark.xfail(reason="not implemented")
def test_min_convex_cost_flow():
    """
    convert linear instances on non-multi graphs to networkx format
    for comparison against nx.min_cost_flow() algorithm
    """

    def mincostflow_nx(network, capacity, supply, weight):
        digraph = nx.DiGraph()
        for i in network.nodes():
            digraph.add_node(i, demand=-supply.get(i, 0.))

        for e in network.edges():
            i, j = network.endpoints(e)
            digraph.add_edge(i, j, capacity=capacity.get(e, np.inf), weight=weight.get(e, 1.))
        return digraph

    g = mygraph()

    if False:
        g.add_edge('a', 0, 1)
        g.add_edge('b', 1, 2)
        g.add_edge('c', 2, 3)
        g.add_edge('d', 3, 0)

        u = {e: 10. for e in g.edges()}
        supply = {0: 1., 1: 2., 2: -3., 3: 0.}
        c = {'a': 10., 'b': 5., 'c': 1., 'd': .5}
    else:
        u = {}
        c = {}
        s = {}

        s[0] = 10.

        g.add_edge('a', 0, 1)
        c['a'] = 1.
        # u['a'] = 1.35

        # g.add_edge( 'aprime', 0, 1 )
        # c['aprime'] = 1000.

        g.add_edge('b', 0, 2)
        c['b'] = 10.

        g.add_edge('c', 1, 3)
        g.add_edge('d', 2, 3)

        s[3] = -10.
        supply = s

    cf = {}
    # for e in c : cf[e] = line( c[e] )
    cf['a'] = lambda x: 5.5 * x + 100.
    cf['b'] = lambda x: np.power(x, 2.0)
    # cf['c'] = lambda x : 2. * np.exp( .5 * ( x - 1. ) )
    cf['c'] = lambda x: 2. * np.exp(.5 * (1. - x))

    def FLOWCOST(flow, cost):
        res = [cc(flow.get(e, 0.)) for e, cc in cost.items()]  # big difference!
        # res = [ cost.get( e, line(0.) )( flow[e] ) for e in flow ]
        return sum(res)

    def FEAS(flow, capacity, network):
        for e in network.edges():
            if flow.get(e, 0.) > capacity.get(e, np.inf): return False
        return True

    flow = MinConvexCostFlow(g, u, supply, cf, 20, epsilon=.001)
    flow_feasible = FEAS(flow, u, g)
    assert flow_feasible
    flow_cost = FLOWCOST(flow, cf)
    LOG.debug(flow, flow_cost, flow_feasible)

    flowstar = {'b': 10., 'd': 10.}
    LOG.debug(flowstar, FLOWCOST(flowstar, cf), FEAS(flowstar, u, g))

    digraph = mincostflow_nx(g, u, supply, c)
    nxflow = nx.min_cost_flow(digraph)

    assert False  # need to compare nxflow to flowstar
