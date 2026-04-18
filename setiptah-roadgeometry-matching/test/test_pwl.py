import pytest
from setiptah.roadgeometry.matching.nxopt.pwl import IntPWL


def test_intpwl_basic():
    """Knots at (1, 0) and (2, -1); slopes -2 left of 1, 1 right of 2.

    offset = leftmost_knot - 1 = 0, since the first breakpoint is at offset + 1.


    Continuity forces middle slope = (-1 - 0) / (2 - 1) = -1.

    Segments:
      x < 1 :      slope=-2, intercept=2   →  f(0)   = 2.0
      1 <= x < 2 : slope=-1, intercept=1   →  f(1.5) = -0.5
      x >= 2 :     slope=1,  intercept=-3  →  f(3)   = 0.0
    """
    f = IntPWL(offset=0, slopes=[-2.0, -1.0, 1.0], intercepts=[2.0, 1.0, -3.0])

    assert f(0)   == pytest.approx(2.0)
    assert f(1)   == pytest.approx(0.0)   # knot
    assert f(1.5) == pytest.approx(-0.5)
    assert f(2)   == pytest.approx(-1.0)  # knot
    assert f(3)   == pytest.approx(0.0)
