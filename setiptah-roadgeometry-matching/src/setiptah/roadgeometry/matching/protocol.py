"""Protocol interfaces for pluggable algorithm implementations."""

from collections.abc import Callable, Mapping
from typing import Any, Protocol, TypeVar, runtime_checkable

from setiptah.roadgeometry.protocol import Topology

TEdge = TypeVar("TEdge")
TNode = TypeVar("TNode")

# Piecewise-linear cost structure: bintrees.RBTree[float, LineData].
# Typed as Any for now; will be refined to a proper protocol when LineData
# is moved to its own module during C++ implementation.
PiecewiseLinearCost = Any


@runtime_checkable
class FlowSolver(Protocol[TEdge, TNode]):
    """Oracle-style convex cost flow solver.

    Treats cost functions as black-box callables. Any solver whose signature
    matches MinConvexCostFlow satisfies this protocol.

    The type variables TEdge and TNode tie network, capacity, supply, cost,
    and the return value together — they all reference the same graph's
    edge and node types.

    Args:
        network:  directed graph satisfying Topology[TEdge, TNode]
        capacity: edge -> capacity bound (missing key implies infinite capacity)
        supply:   node -> supply (positive) or demand (negative)
        cost:     edge -> callable convex cost function  f: float -> float
        U:        bound on optimal flow on any edge; width of first scaling phase

    Returns:
        Mapping of edge -> optimal flow value
    """
    def __call__(
        self,
        network: Topology[TEdge, TNode],
        capacity: Mapping[TEdge, float],
        supply: Mapping[TNode, float],
        cost: Mapping[TEdge, Callable[[float], float]],
        U: float,
    ) -> Mapping[TEdge, float]: ...


@runtime_checkable
class ConvexFlowSolver(Protocol[TEdge, TNode]):
    """Piecewise-linear-aware convex cost flow solver.

    Like FlowSolver, but receives the raw piecewise-linear cost structure
    rather than wrapped callables. A solver that knows about this structure
    can exploit it directly — e.g. the C++ ConvexCostFlowSolver template.

    Args:
        network:    directed graph satisfying Topology[TEdge, TNode]
        capacity:   edge -> capacity bound
        supply:     node -> supply/demand
        cost_lines: edge -> PiecewiseLinearCost (RBTree of LineData)
        U:          bound on optimal flow on any edge

    Returns:
        Mapping of edge -> optimal flow value
    """
    def __call__(
        self,
        network: Topology[TEdge, TNode],
        capacity: Mapping[TEdge, float],
        supply: Mapping[TNode, float],
        cost_lines: Mapping[TEdge, PiecewiseLinearCost],
        U: float,
    ) -> Mapping[TEdge, float]: ...
