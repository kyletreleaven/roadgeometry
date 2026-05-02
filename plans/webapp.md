# Road Matching Web App — Design Plan

## Concept

Interactive map where users place supply/demand pins and see the optimal matching
displayed as "carbon trail" polylines — the shortest road paths between each matched pair.

## Scope

- Geographic area: Cambridge, MA (single osmnx tile, ~2km radius)
- Road network: OSM drive network, simplified graph, projected to UTM
- Algorithm: existing Python/C++ matching code (`RoadnetMatchingProblem` / `_cpp.compute_matching`)

---

## Architecture

### Stack

- **Frontend**: Leaflet.js + OpenStreetMap tiles, React + TypeScript + Vite
- **Backend**: FastAPI, single Python process
- **Road data**: osmnx, downloaded eagerly at server startup in background thread

### File Structure

```
webapp/
  backend/
    app.py          # FastAPI app, endpoints, startup hook
    state.py        # MatchingSession dataclass
    network.py      # Graph load, pin snapping, Dijkstra, relevant subgraph
    matching.py     # Bridge to RoadnetMatchingProblem, trail geometry
    models.py       # Pydantic request/response shapes
  frontend/
    index.html
    app.js
    style.css
  requirements.txt  # fastapi, uvicorn, osmnx, shapely, pyproj
```

---

## Road Network

Loaded once at startup:

```python
import osmnx as ox

G = ox.graph_from_point((42.373, -71.109), dist=2500,
                         network_type="drive", simplify=True)
G = ox.project_graph(G)   # UTM — lengths in meters
```

Wrapped as `GeoFramesNetwork` (from `setiptah-roadgeometry`) to satisfy the `Roadnet`
protocol required by the matching algorithm.

Edge keys are `(u, v, k)` OSM tuples — used as `TRoad` throughout.

If the graph is not yet ready when the first pin arrives, return HTTP 503 with a
`"loading"` status so the frontend can show a spinner.

---

## Pin Placement

- Points are `(road, y)` pairs: `road = (u, v, k)`, `y` = arc-length offset in meters
- Snapping: `osmnx.nearest_edges` → project click point onto edge geometry via
  `shapely LineString.project`
- Supply/demand alternation: `kind = "supply" if supply_count <= demand_count else "demand"`
  (imbalance ≤ 1, ties broken by supply)

---

## Relevant Subgraph

The matching runs not on the full Cambridge graph but on a small subgraph — the union
of shortest paths between each supply/demand pair. This stays small regardless of graph size.

### Maintenance

**Adding a pin:**
1. Run early-termination Dijkstra from the new pin's position.
2. Stop as soon as the wavefront settles any node already in the relevant subgraph.
3. Add all nodes/edges along the path from new pin to that hit node.
4. Re-run matching on the updated subgraph.

**Cold start (first matched pair):**
The subgraph is empty so early-termination never fires. Dijkstra runs until it reaches
the other pin's nearest node. The resulting path initializes the subgraph.

**Deleting a pin:**
Rebuild the relevant subgraph from scratch by re-running shortest paths between all
remaining matched pairs. Acceptable for small pin counts (≤ ~50).

### Dijkstra

Uses the existing continuous-space implementation in
`setiptah-roadgeometry/src/setiptah/roadgeometry/dijkstra.py` (`RoadnetQuery` class).
Early-termination added as a stop condition: halt when a settled node is in
`relevant_subgraph.nodes`.

Future: replace with A* (haversine heuristic) for larger areas.

---

## Matching

```python
def run_matching(session, roadnet):
    supply = [p for p in session.pins if p.kind == "supply"]
    demand = [p for p in session.pins if p.kind == "demand"]
    n = min(len(supply), len(demand))
    if n == 0:
        return [], []
    P = [p.road_point for p in supply[:n]]
    Q = [p.road_point for p in demand[:n]]
    sub = subgraph_roadnet(session.relevant_subgraph, roadnet)
    prob = RoadnetMatchingProblem(P, Q, sub)
    matching = prob.compute_optimal(MatchingResult.MATCHING)
    paths = [roadnet_metric.shortest_path(P[i], Q[j]) for i, j in matching]
    return matching, paths
```

Trail geometries: walk each `RoadSegment` in the path, crop edge geometry via
`crop_line_string(edges_gdf.geometry.loc[road], seg.start, seg.end)`, serialize
as `[[lat, lon]]` for Leaflet.

---

## API

| Method   | Path             | Body          | Response                          |
|----------|------------------|---------------|-----------------------------------|
| `GET`    | `/`              | —             | `index.html`                      |
| `GET`    | `/status`        | —             | `{ready: bool}`                   |
| `POST`   | `/pins`          | `{lat, lon}`  | `{pin, matching}`                 |
| `DELETE` | `/pins/{pin_id}` | —             | `{matching}`                      |
| `POST`   | `/reset`         | —             | `{}`                              |
| `GET`    | `/state`         | —             | `{pins, matching}`                |

`matching` response shape:
```json
{
  "pairs": [{"supply_id": "...", "demand_id": "..."}],
  "trails": [{"coordinates": [[lat, lon], ...]}]
}
```

---

## Frontend

- Leaflet map centered on Cambridge, OSM tiles
- Click → `POST /pins` → place circle marker (red=supply, blue=demand) at snapped location
- Clear old trail polylines, draw new ones (dark gray, weight=3)
- "Reset" button → `POST /reset`
- Poll `GET /status` on load; show spinner until `ready: true`

---

## Implementation Order

1. ~~Frontend scaffold~~ ✓ — Vite + React + TypeScript, Leaflet map centered on Cambridge
2. `network.py` — load graph, wrap as `GeoFramesNetwork`, pin snapping
3. `state.py` — `MatchingSession`, `RelevantSubgraph`, pin balance logic
4. `network.py` — early-termination Dijkstra, subgraph maintenance
5. `matching.py` — run matching, compute trail geometries
6. `app.py` — wire endpoints, startup background load
7. Frontend — click handler, pin markers, trail rendering, spinner

---

## Open Questions

- Does `GeoFramesNetwork` work out of the box with `RoadnetMatchingProblem`, or does
  it need adaptation? Check `geopandas.py` before starting step 1.
- UTM projection zone: osmnx picks automatically via `project_graph`; verify lengths
  are in meters after projection.
- Does `_cpp.compute_matching` accept the subgraph `RoadNetwork` directly, or does it
  need the raw `(road, y)` interface? Check binding in `_cpp.cpp`.

---

## Future

- A* with haversine heuristic (needed for larger geographic areas)
- Dynamic tile loading + LRU cache (needed for unbounded panning)
- Multi-user sessions (session token → `MatchingSession` dict)
- Dijkstra in C++ (already in `cpp/include/roadgeometry/dijkstra.hpp`)
