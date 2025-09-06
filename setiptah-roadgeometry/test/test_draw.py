from setiptah.roadgeometry.draw.demo.viz import *

import pytest


def from_polar(r, theta):
    return r * np.array([np.cos(theta), np.sin(theta)])


def test_main():
    main()


@pytest.mark.xfail(reason="don't know")
def test_fitarc():
    p1 = np.array([0, 0])
    p2 = np.array([1, 0])
    center, R, thetai, thetaj = fitArc2Segment(p1, p2, 2)

    arclen = R * (thetaj - thetai)
    assert arclen == 2

    p1_ = center + from_polar(R, thetai)
    p2_ = center + from_polar(R, thetaj)
