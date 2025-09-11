from setiptah.roadgeometry.matching.range_queues import *


class TestRoadPointSeq:

    def test_cat(self):

        a = RoadPointSeq("A", 2, 10)
        a, b = a.split(4)
        b, c = b.split(2)

        assert a == RoadPointSeq("A", 2, 6)
        assert b == RoadPointSeq("A", 6, 8)
        assert c == RoadPointSeq("A", 8, 10)

        assert a.can_cat(b)
        assert not a.can_cat(c)
        assert b.can_cat(c)

        assert a.cat(b) == RoadPointSeq("A", 2, 8)
        assert b.cat(c) == RoadPointSeq("A", 6, 10)

        d = RoadPointSeq("B", 0, 7, reverse=True)
        d, e = d.split(3)
        e, f = e.split(2)

        assert d == RoadPointSeq("B", 4, 7, reverse=True)
        assert e == RoadPointSeq("B", 2, 4, reverse=True)
        assert f == RoadPointSeq("B", 0, 2, reverse=True)

        assert d.can_cat(e)
        assert d.cat(e) == RoadPointSeq("B", 2, 7, reverse=True)
        assert not d.can_cat(f)

        assert not RoadPointSeq("A", 0, 4).can_cat(RoadPointSeq("B", 4, 6))

    def test_deque_ops(self):

        q = deque()
        append_range(q, RoadPointSeq("A", 0, 4))
        extend_points(q, [
            RoadPointSeq("A", 4, 7),
            RoadPointSeq("A", 8, 10)  # skip one
        ])

        extend_points(q, [
            RoadPointSeq("B", 5, 8, reverse=True),
            RoadPointSeq("B", 3, 5, reverse=True),
            RoadPointSeq("B", 0, 2, reverse=True)
        ])

        assert list(q) == [
            RoadPointSeq("A", 0, 7),
            RoadPointSeq("A", 8, 10),
            RoadPointSeq("B", 3, 8, reverse=True),
            RoadPointSeq("B", 0, 2, reverse=True),
        ]

        q1 = take_points(q, 12)

        assert list(q1) == [
            RoadPointSeq("A", 0, 7),
            RoadPointSeq("A", 8, 10),
            RoadPointSeq("B", 5, 8, reverse=True),
        ]

        assert list(q) == [
            RoadPointSeq("B", 3, 5, reverse=True),
            RoadPointSeq("B", 0, 2, reverse=True),
        ]

        p = pop_point(q)
        assert p == ("B", 4)
