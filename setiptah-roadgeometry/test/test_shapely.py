from setiptah.roadgeometry.util.shapely import *
from shapely import LineString


def test_crop_line_string():

    line_string = LineString([
        (0, 0),
        (1, 0),
        (1, 1),
        (0, 1),
    ])

    assert crop_line_string(line_string, .5, 2.5) == LineString([
        (.5, 0),
        (1, 0),
        (1, 1),
        (.5, 1),
    ])


def test_crop_line_string_consume_all():

    line_string = LineString([
        (0, 0),
        (1, 0),
        (1, 1),
        (0, 1),
    ])

    left, right = split_line_string(line_string, 100)
    assert left == line_string
    assert right.length == 0
