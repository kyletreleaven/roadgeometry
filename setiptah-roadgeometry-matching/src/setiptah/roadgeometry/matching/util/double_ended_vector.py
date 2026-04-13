"""Double-ended growable vector with integer indexing and default value zero.

Supports O(1) amortized indexed read/write over an arbitrary integer range,
growing in both directions as needed.  Unwritten positions read as zero.

Internally two lists:
  _right[k]  holds the value at index  offset + k   (k >= 0)
  _left[k]   holds the value at index  offset - 1 - k (k >= 0)

Both lists grow by Python's usual list-append doubling.  The first write
establishes the offset; subsequent writes extend whichever side is needed.

Designed as a reference for a C++ equivalent (std::vector pair + offset).

This pattern appears in cellular automata (infinite grids that grow as cells are born),
Turing machine tape simulations, and arbitrary-precision arithmetic (digit arrays growing
left or right).  Python's `collections.deque` and C++'s `std::deque` are spiritually
similar but only expose sequential push/pop, not arbitrary integer-indexed access over
an unbounded range.  No canonical name or stdlib implementation exists in most languages.
"""

from typing import Iterator


class DoubleEndedVector:
    """Growable integer-indexed array with implicit zero default.

    Supports arbitrary integer indices; grows in both directions on demand.
    Iteration yields (index, value) pairs in ascending index order over all
    allocated positions, including zeros.
    """

    __slots__ = ("_right", "_left", "_offset")

    def __init__(self) -> None:
        self._right: list[float] = []   # value at offset + k
        self._left:  list[float] = []   # value at offset - 1 - k
        self._offset: int | None = None

    def __getitem__(self, i: int) -> float:
        if self._offset is None:
            return 0.0
        k = i - self._offset
        if k >= 0:
            return self._right[k] if k < len(self._right) else 0.0
        else:
            k = -k - 1
            return self._left[k] if k < len(self._left) else 0.0

    def __setitem__(self, i: int, v: float) -> None:
        if self._offset is None:
            self._offset = i
        k = i - self._offset
        if k >= 0:
            while len(self._right) <= k:
                self._right.append(0.0)
            self._right[k] = v
        else:
            k = -k - 1
            while len(self._left) <= k:
                self._left.append(0.0)
            self._left[k] = v

    @property
    def min_index(self) -> int:
        assert self._offset is not None
        return self._offset - len(self._left)

    @property
    def max_index(self) -> int:
        assert self._offset is not None
        return self._offset + len(self._right) - 1

    def __len__(self) -> int:
        return len(self._left) + len(self._right)

    def __iter__(self) -> Iterator[int]:
        """Yield indices in ascending order over all allocated positions."""
        for i, _ in self.items():
            yield i

    def items(self) -> Iterator[tuple[int, float]]:
        """Yield (index, value) in ascending order over all allocated positions."""
        if self._offset is None:
            return
        for k in range(len(self._left) - 1, -1, -1):
            yield self._offset - 1 - k, self._left[k]
        for k in range(len(self._right)):
            yield self._offset + k, self._right[k]
