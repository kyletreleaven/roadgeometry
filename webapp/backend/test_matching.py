import pytest
import network
import matching as matching_module
from models import Matching
from state import Session


@pytest.fixture(scope='module')
def loaded_network():
    network._load()
    assert network._ready


# Two pairs of points in Cambridge, MA
POINTS = [
    (42.3750, -71.1050),  # near Harvard Square
    (42.3700, -71.1150),  # near Central Square
    (42.3780, -71.1100),  # near Inman Square
    (42.3660, -71.1020),  # near MIT
]


def test_snap_pin(loaded_network):
    result = network.snap_pin(*POINTS[0])
    assert 'road' in result and 'y' in result
    assert 'lat' in result and 'lon' in result
    assert isinstance(result['road'], tuple) and len(result['road']) == 3
    assert result['y'] >= 0


def test_run_matching_empty(loaded_network):
    session = Session()
    result = matching_module.run_matching(session)
    assert result['pairs'] == [] and result['trails'] == []


def test_run_matching_one_pair(loaded_network):
    session = Session()
    for lat, lon in POINTS[:2]:
        snapped = network.snap_pin(lat, lon)
        pin = session.add_pin(snapped['lat'], snapped['lon'])
        pin.road = snapped['road']
        pin.y = snapped['y']

    result = matching_module.run_matching(session)
    m = Matching(**result)
    assert len(m.pairs) == 1
    assert len(m.trails) == 1
    trail = m.trails[0]
    assert len(trail.coordinates) > 0
    lat, lon = trail.coordinates[0]
    assert 42.0 < lat < 43.0
    assert -72.0 < lon < -71.0


def test_run_matching_two_pairs(loaded_network):
    session = Session()
    for lat, lon in POINTS:
        snapped = network.snap_pin(lat, lon)
        pin = session.add_pin(snapped['lat'], snapped['lon'])
        pin.road = snapped['road']
        pin.y = snapped['y']

    result = matching_module.run_matching(session)
    m = Matching(**result)
    assert len(m.pairs) == 2
    assert len(m.trails) == 2
    for trail in m.trails:
        assert len(trail.coordinates) > 0
