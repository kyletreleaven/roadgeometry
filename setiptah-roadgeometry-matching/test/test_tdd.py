"""

TODOs:
- [x] build a network
- [ ] sample a matching problem instance
- [ ] compute optimal matching
- [ ] compute corr. shortest paths
- [ ] compute total cost

"""
from setiptah.basic_graph.graphs import RoadNetwork, SetReadOnlyView

import pytest


@pytest.fixture
def roadnet():
    rn = RoadNetwork()
    rn.add_edge("edge1", 0, 1, 1.0)
    rn.add_edge("edge2", 1, 0, 2.0, oneway=True)
    return rn


class TestSetReadOnlyView:

    def test_contains(self):
        view = SetReadOnlyView({2, 3, 4})
        assert 2 in view
        assert 5 not in view

    def test_iter(self):
        L = list(SetReadOnlyView({2, 3, 4}))
        L.sort()
        assert L == [2, 3, 4]

    def test_len(self):
        view = SetReadOnlyView({2, 3, 4})
        assert len(view) == 3


class TestRoadNetwork:

    def test_nodes(self, roadnet):
        assert roadnet.nodes() == {0, 1}

    def test_length(self, roadnet):
        assert roadnet.length("edge2") == 2.

    def test_endpoints(self, roadnet):
        assert roadnet.endpoints("edge1") == (0, 1)

    def test_oneway(self, roadnet):
        assert not roadnet.is_oneway("edge1")
        assert roadnet.is_oneway("edge2")

    def test_has_node(self, roadnet):
        assert roadnet.has_node(0)
        assert not roadnet.has_node(4)

    def test_hasedge(self, roadnet):
        assert roadnet.has_edge("edge1")
        assert not roadnet.has_edge("some_other_edge")

    def test_remove_node(self, roadnet):
        roadnet.remove_node(0)
        assert roadnet.nodes() == {1}
        assert roadnet.edges() == set()


def test_create_roadnet(roadnet):
    rn = roadnet

    assert rn.nodes() == {0, 1}
    assert rn.edges() == {"edge1", "edge2"}

    assert rn.out_edges(0) == {"edge1"}
    assert rn.in_edges(0) == {"edge2"}
