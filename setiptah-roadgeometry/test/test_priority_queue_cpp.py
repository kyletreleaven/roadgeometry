"""Tests for the C++ PriorityQueue binding.

Validates _cpp.PriorityQueue against the pure-Python priorityDictionary,
running the same operations on both and asserting identical results.
"""

import pytest

_cpp = pytest.importorskip("setiptah.roadgeometry._cpp")

from setiptah.roadgeometry.util.priodict import priorityDictionary

PriorityQueue = _cpp.PriorityQueue


# ---------------------------------------------------------------------------
# Unit tests for the C++ PriorityQueue in isolation
# ---------------------------------------------------------------------------

class TestPriorityQueueBasic:

    def test_empty(self):
        pq = PriorityQueue()
        assert len(pq) == 0
        assert 0 not in pq

    def test_push_and_contains(self):
        pq = PriorityQueue()
        pq.push(1, 5.0)
        assert 1 in pq
        assert len(pq) == 1

    def test_peek_min(self):
        pq = PriorityQueue()
        pq.push(3, 3.0)
        pq.push(1, 1.0)
        pq.push(2, 2.0)
        assert pq.peek_min() == 1

    def test_pop_min_order(self):
        pq = PriorityQueue()
        pq.push(10, 10.0)
        pq.push(5, 5.0)
        pq.push(7, 7.0)
        assert pq.pop_min() == 5
        assert pq.pop_min() == 7
        assert pq.pop_min() == 10
        assert len(pq) == 0

    def test_decrease_key(self):
        pq = PriorityQueue()
        pq.push(1, 10.0)
        pq.push(2, 5.0)
        pq.push(1, 2.0)   # decrease key 1 from 10 → 2
        assert pq.peek_min() == 1
        assert pq.get(1) == 2.0

    def test_push_no_increase(self):
        """push with higher priority must be a no-op."""
        pq = PriorityQueue()
        pq.push(1, 3.0)
        pq.push(1, 99.0)   # should not overwrite
        assert pq.get(1) == 3.0

    def test_get_default(self):
        pq = PriorityQueue()
        import math
        assert math.isinf(pq.get(99))          # default → inf
        assert pq.get(99, -1.0) == -1.0

    def test_pop_min_removes(self):
        pq = PriorityQueue()
        pq.push(42, 1.0)
        pq.pop_min()
        assert 42 not in pq
        assert len(pq) == 0


# ---------------------------------------------------------------------------
# Parity tests: C++ PriorityQueue vs pure-Python priorityDictionary
# ---------------------------------------------------------------------------

def _pd_get(pd, key, default=float("inf")):
    return pd.get(key, default)


def _run_sequence(ops):
    """
    Execute a list of (op, *args) tuples against both implementations
    and assert identical outcomes at every step.

    ops:
        ("push",  key, priority)
        ("pop_min",)
        ("peek_min",)
        ("contains", key)
        ("get",  key)
        ("len",)
    """
    pq = PriorityQueue()
    pd = priorityDictionary()

    for op, *args in ops:
        if op == "push":
            key, pri = args
            pq.push(key, pri)
            if key not in pd or pd[key] > pri:
                pd[key] = pri

        elif op == "pop_min":
            cpp_key = pq.pop_min()
            py_key = pd.smallest()
            del pd[py_key]
            assert cpp_key == py_key, f"pop_min: C++={cpp_key}, py={py_key}"

        elif op == "peek_min":
            cpp_key = pq.peek_min()
            py_key = pd.smallest()
            assert cpp_key == py_key, f"peek_min: C++={cpp_key}, py={py_key}"

        elif op == "contains":
            key, = args
            assert (key in pq) == (key in pd)

        elif op == "get":
            key, = args
            assert pq.get(key) == _pd_get(pd, key)

        elif op == "len":
            assert len(pq) == len(pd)

        else:
            raise ValueError(f"Unknown op: {op}")


def test_parity_basic():
    _run_sequence([
        ("push", 3, 3.0),
        ("push", 1, 1.0),
        ("push", 2, 2.0),
        ("len",),
        ("peek_min",),
        ("pop_min",),
        ("pop_min",),
        ("pop_min",),
        ("len",),
    ])


def test_parity_decrease_key():
    _run_sequence([
        ("push", 10, 10.0),
        ("push", 20, 20.0),
        ("push", 10, 1.0),   # decrease
        ("peek_min",),
        ("pop_min",),
        ("pop_min",),
    ])


def test_parity_mixed():
    _run_sequence([
        ("push", 5, 5.0),
        ("push", 3, 3.0),
        ("push", 7, 7.0),
        ("push", 1, 1.0),
        ("push", 3, 0.5),   # decrease key 3
        ("len",),
        ("peek_min",),
        ("pop_min",),
        ("contains", 3),
        ("get", 3),
        ("pop_min",),
        ("pop_min",),
        ("pop_min",),
        ("len",),
    ])


# ---------------------------------------------------------------------------
# Dijkstra parity: run Python Dijkstra and verify same distances when
# PriorityQueue replaces priorityDictionary as the frontier.
# ---------------------------------------------------------------------------

def _dijkstra_cpp(graph, source):
    """
    Dijkstra using _cpp.PriorityQueue as the frontier.
    `graph` is a dict: node -> list of (neighbor, weight).
    """
    dist = {}
    pq = PriorityQueue()
    pq.push(source, 0.0)

    while len(pq) > 0:
        u = pq.pop_min()
        if u in dist:
            continue
        dist[u] = pq.get(u, 0.0) if False else _extract_dist(pq, u, dist)
        for v, w in graph.get(u, []):
            new_d = dist[u] + w
            if v not in dist:
                pq.push(v, new_d)

    return dist


def _extract_dist(pq, u, dist):
    # After pop_min we lose the priority; track separately.
    raise AssertionError("unreachable")


def _dijkstra_cpp_v2(graph, source):
    """Dijkstra with _cpp.PriorityQueue, tracking distances separately."""
    dist = {}
    pq = PriorityQueue()
    pq.push(source, 0.0)
    best = {source: 0.0}

    while len(pq) > 0:
        u = pq.pop_min()
        if u in dist:
            continue
        dist[u] = best[u]
        for v, w in graph.get(u, []):
            new_d = dist[u] + w
            if v not in dist and (v not in best or new_d < best[v]):
                best[v] = new_d
                pq.push(v, new_d)

    return dist


def _dijkstra_py(graph, source):
    """Reference Dijkstra using priorityDictionary."""
    dist = {}
    pd = priorityDictionary()
    pd[source] = 0.0

    for u in pd:
        dist[u] = pd[u]
        for v, w in graph.get(u, []):
            new_d = dist[u] + w
            if v not in dist:
                pd.setdefault(v, float("inf"))
                if new_d < pd[v]:
                    pd[v] = new_d

    return dist


@pytest.fixture
def small_graph():
    # Simple directed weighted graph
    return {
        0: [(1, 1.0), (2, 4.0)],
        1: [(2, 2.0), (3, 5.0)],
        2: [(3, 1.0)],
        3: [],
    }


def test_dijkstra_parity(small_graph):
    for source in small_graph:
        cpp_dist = _dijkstra_cpp_v2(small_graph, source)
        py_dist = _dijkstra_py(small_graph, source)
        assert cpp_dist == py_dist, (
            f"source={source}: C++={cpp_dist}, py={py_dist}"
        )


def test_dijkstra_parity_disconnected():
    graph = {
        0: [(1, 1.0)],
        1: [],
        2: [(3, 1.0)],   # disconnected component
        3: [],
    }
    cpp_dist = _dijkstra_cpp_v2(graph, 0)
    py_dist = _dijkstra_py(graph, 0)
    assert cpp_dist == py_dist
    assert 2 not in cpp_dist
    assert 3 not in cpp_dist
