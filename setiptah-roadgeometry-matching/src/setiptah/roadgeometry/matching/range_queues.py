import dataclasses
from collections import deque
from dataclasses import dataclass
from typing import Generic, TypeVar

TRoad = TypeVar("TRoad")


@dataclass(frozen=True)
class RoadPointSeq(Generic[TRoad]):
    road: TRoad
    start: int
    end: int
    reverse: bool = False

    def __post_init__(self):
        assert self.start <= self.end

    def __len__(self):
        return self.end - self.start

    def as_point(self):
        assert len(self) == 1
        return self.road, self.start

    def split(self, n: int) -> tuple["RoadPointSeq", "RoadPointSeq"]:
        assert n <= len(self)

        if self.reverse:
            rem = dataclasses.replace(self, end=self.end - n)
            taken = dataclasses.replace(self, start=self.end - n)
        else:
            taken = dataclasses.replace(self, end=self.start + n)
            rem = dataclasses.replace(self, start=self.start + n)

        return taken, rem

    def can_cat(self, other: "RoadPointSeq[TRoad]") -> bool:
        if other.road != self.road:
            return False

        if self.reverse:
            return other.reverse and other.end == self.start
        else:
            return not other.reverse and other.start == self.end

    def cat(self, other):
        assert self.can_cat(other)
        if self.reverse:
            return dataclasses.replace(self, start=other.start)
        else:
            return dataclasses.replace(self, end=other.end)


PointSeqQ = deque[RoadPointSeq[TRoad]]


def num_points(rq: PointSeqQ):
    return sum(len(r) for r in rq)


def extend_points(ps: PointSeqQ[TRoad], qs: PointSeqQ[TRoad]):
    for r in qs:
        append_range(ps, r)


def append_range(point_seq: PointSeqQ[TRoad], points: RoadPointSeq[TRoad]):
    if len(point_seq) > 0 and point_seq[-1].can_cat(points):
        point_seq[-1] = point_seq[-1].cat(points)
    else:
        point_seq.append(points)


def take_points(point_seq: PointSeqQ[TRoad], n: int) -> PointSeqQ[TRoad]:
    assert n <= sum(len(r) for r in point_seq)

    out = deque()
    while n > 0:
        front = point_seq.popleft()
        n_ = min(n, len(front))
        taken, rem = front.split(n_)
        out.append(taken)
        if len(rem) > 0:
            point_seq.appendleft(rem)  # TODO: Optimize?
        n -= n_

    return out


def pop_point(point_seq: PointSeqQ[TRoad]) -> tuple[TRoad, int]:
    pseq, = take_points(point_seq, 1)
    return pseq.as_point()
