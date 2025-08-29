"""

TODO: _Not_ the permanent home of this protocol.

"""
from typing import TypeVar, Generic, Protocol
from collections.abc import Collection


TRoad = TypeVar("TRoad")
TVert = TypeVar("TVert")


class Roadnet(Protocol[TRoad, TVert]):
    """The basic protocol of a metric graph, potentially with one-way roads.

    TODO: This is very low-level. Move this into geometry package?

    """

    def edges(self) -> Collection[TRoad]:
        """Get the edges (roads) in the graph."""

    def out_edges(self, u: TVert) -> Collection[TRoad]:
        """Get edges out of a vertex."""

    def in_edges(self, u: TVert) -> Collection[TRoad]:
        """Get edges into a vertex."""

    def nodes(self) -> Collection[TVert]:
        """Get the nodes (interchanges) in the graph."""

    def length(self, road: TRoad) -> float:
        """Get the length of a road in the network."""

    def endpoints(self, road: TRoad) -> tuple[TVert, TVert]:
        """Get the endpoints of the road."""

    def is_oneway(self, road: TRoad) -> bool:
        """Get whether the road is one-way."""
