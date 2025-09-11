import json

from setiptah.roadgeometry.formats import GraphSchema
from setiptah.roadgeometry.graphs import RoadNetwork, SetReadOnlyView
import networkx as nx

import pytest


@pytest.fixture
def roadnet():
    rn = RoadNetwork()
    rn.add_edge("edge1", 0, 1, 1.0)
    rn.add_edge("edge2", 1, 0, 2.0, oneway=True)
    return rn


from networkx.readwrite import json_graph


def write_node_link_data(g, f):
    data = json_graph.node_link_data(g, edges="edges")
    json.dump(data, f)


def read_node_link_data(f):
    data = json.load(f)
    return json_graph.node_link_graph(data, edges="edges")


@pytest.mark.parametrize("write_fn, read_fn", [
    # (nx.nx_pydot.write_dot, nx.nx_pydot.read_dot),
    # (nx.write_graphml, nx.read_graphml),
    # (nx.write_gml, nx.read_gml)
    (write_node_link_data, read_node_link_data)
])
def test_json_graph(write_fn, read_fn, roadnet, tmp_path):

    schema = GraphSchema(edge_attr="name")
    g = schema.to_networkx(roadnet)

    if False:
        import io
        buf = io.StringIO()
        # buf = io.BytesIO()

        # nx.nx_pydot.write_dot(g, buf)
        write_fn(g, buf)

        assert False, buf.getvalue()

    path = tmp_path / "g.dot"
    with path.open("w") as f:
        write_fn(g, f)
    with path.open("r") as f:
        g_ = read_fn(f)

    # g_ = nx.relabel_nodes(g_, int)
    roadnet_ = schema.from_networkx(g_)

    assert roadnet_.nodes() == roadnet.nodes()
    assert roadnet_._edges == roadnet._edges


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
