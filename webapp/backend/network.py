import os
import time
import threading

import osmnx as ox
import pyproj
from shapely.geometry import Point

CACHE_PATH = os.path.join(os.path.dirname(__file__), 'cambridge.graphml')
TTL_SECONDS = 30 * 24 * 3600  # 30 days

_POINT = (42.373, -71.109)
_DIST = 2500

_G = None
_edges = None       # edges GeoDataFrame, indexed by (u, v, k)
_to_utm = None      # pyproj Transformer: WGS84 → graph UTM CRS
_from_utm = None    # pyproj Transformer: graph UTM CRS → WGS84
_ready = False


def _cache_fresh() -> bool:
    if not os.path.exists(CACHE_PATH):
        return False
    return time.time() - os.path.getmtime(CACHE_PATH) < TTL_SECONDS


def _load():
    global _G, _edges, _to_utm, _from_utm, _ready

    if _cache_fresh():
        G = ox.load_graphml(CACHE_PATH)
    else:
        G = ox.graph_from_point(_POINT, dist=_DIST, network_type='drive', simplify=True)
        G = ox.project_graph(G)
        ox.save_graphml(G, CACHE_PATH)

    crs = G.graph['crs']
    to_utm = pyproj.Transformer.from_crs('EPSG:4326', crs, always_xy=True)
    from_utm = pyproj.Transformer.from_crs(crs, 'EPSG:4326', always_xy=True)
    _, edges = ox.graph_to_gdfs(G)

    _ = edges.sindex  # build spatial index once at load time

    _G = G
    _edges = edges
    _to_utm = to_utm
    _from_utm = from_utm
    _ready = True


def start():
    threading.Thread(target=_load, daemon=True).start()


def is_ready() -> bool:
    return _ready


def snap_pin(lat: float, lon: float) -> dict:
    utm_x, utm_y = _to_utm.transform(lon, lat)
    u, v, k = ox.nearest_edges(_G, utm_x, utm_y)
    geom = _edges.loc[(u, v, k), 'geometry']
    offset = geom.project(Point(utm_x, utm_y))
    snapped = geom.interpolate(offset)
    snap_lon, snap_lat = _from_utm.transform(snapped.x, snapped.y)
    return {
        'road': (int(u), int(v), int(k)),
        'y': offset,
        'lat': snap_lat,
        'lon': snap_lon,
    }
