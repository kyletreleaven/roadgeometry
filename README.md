# roadgeometry

`setiptah.roadgeometry` is a currently pure-Python library for geometric queries on
[**metric graphs**](https://en.wikipedia.org/wiki/Quantum_graph#Metric_graphs), a broad class of
one-dimensional continuous "roadmap" metric spaces.

The library
provides api for defining and querying road networks as metric graphs,
including support for parallel roads, mixtures of one-way and bidirectional roads,
shortest paths, and nearest-neighbor queries.

The main contribution of the library is an **efficient $O(n \log n)$ bipartite matching algorithm**,
achieving the fastest possible asymptotic runtime in terms of the number of query points.
The algorithm reduces bipartite matching to minimum-cost flow on graphs with convex edge costs,
all within the highly restrictive runtime budget
(see [arXiv:1311.4609](https://arxiv.org/abs/1311.4609)).
In contrast,
classical algorithms like the [**Hungarian method**](https://en.wikipedia.org/wiki/Hungarian_algorithm) run in $O(n^3)$,
which can be too slow at scale for many applications.

---

## Highlights

- 🚀 **Fastest-possible bipartite matching**
  - Exact $O(n \log n)$ algorithm for bipartite matching weighted by roadmap distance.
  - Based on convex-cost minimum-cost flow on graphs.
  - Scales efficiently to large problem instances.

- 🛣 **Road-aware metric graphs**
  - Networks defined in terms of *roads*, not just connected vertices:
    - Allows multiple distinct roads between the same endpoints.
    - Consistent support for mixtures of one-way and bidirectional roads.
  - Planar embeddings supported.
  - Visualization available with [NetworkX](https://networkx.org/).

- 📏 **Continuous shortest paths**
  - Compute distances and shortest paths between **arbitrary points along roads**, not just vertices.
  - Dijkstra-like algorithm adapted to parallel roads and mixed directionality.

- 🔍 **Nearest-neighbor search**
  - Efficient Dijkstra-like search for the closest network location to a query point.

---

## Trying it out

Please feel free to check out the unit tests for usage examples.

This project uses [nox](https://nox.thea.codes/) for automation.
With `nox` installed, you can run the unit tests in either directory by:

```bash
# Run the unit test suite in either package directory.
nox -r -s test
```
