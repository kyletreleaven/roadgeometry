"""

"""
from typing import TypeVar, Generic, Protocol
from collections.abc import Collection


TRoad = TypeVar("TRoad")
TVert = TypeVar("TVert")


class Topology(Protocol[TRoad, TVert]):

    def edges(self) -> Collection[TRoad]:
        """Get the edges (roads) in the graph."""

    def out_edges(self, u: TVert) -> Collection[TRoad]:
        """Get edges out of a vertex."""

    def in_edges(self, u: TVert) -> Collection[TRoad]:
        """Get edges into a vertex."""

    def nodes(self) -> Collection[TVert]:
        """Get the nodes (interchanges) in the graph."""

    def endpoints(self, road: TRoad) -> tuple[TVert, TVert]:
        """Get the endpoints of the road."""


class Roadnet(
    Topology[TRoad, TVert],  # It extends the Topology protocol.
    Protocol[TRoad, TVert],  # (It is itself a protocol.)
):
    """The basic protocol of a metric graph, potentially with one-way roads.

    """

    def length(self, road: TRoad) -> float:
        """Get the length of a road in the network."""

    def is_oneway(self, road: TRoad) -> bool:
        """Get whether the road is one-way."""

    class Point(Protocol):

        def __iter__(self) -> tuple[TRoad, float]:
            """Get the tuple of road and coordinate."""
