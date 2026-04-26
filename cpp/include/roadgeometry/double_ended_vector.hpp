#pragma once
#include <cassert>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace roadgeometry {

// ---------------------------------------------------------------------------
// DoubleEndedVector<T>
//
// Growable integer-indexed array with implicit zero/default-constructed value.
// Supports arbitrary integer indices; grows in both directions on demand.
// The first write establishes the offset; subsequent writes extend whichever
// side is needed.
//
// Internally two vectors:
//   _right[k]  holds the value at index  offset + k   (k >= 0)
//   _left[k]   holds the value at index  offset - 1 - k  (k >= 0)
//
// C++ equivalent of the Python DoubleEndedVector (double_ended_vector.py).
// ---------------------------------------------------------------------------
template <typename T = double>
class DoubleEndedVector {
public:
    DoubleEndedVector() = default;

    // -----------------------------------------------------------------------
    // operator[] non-const — allocates slot on demand (zero-initialized).
    // Enables result[f] += delta naturally.
    // -----------------------------------------------------------------------
    T& operator[](int i) {
        if (!_offset.has_value())
            _offset = i;
        int k = i - *_offset;
        if (k >= 0) {
            if (k >= (int)_right.size())
                _right.resize(k + 1, T{});
            return _right[k];
        } else {
            k = -k - 1;
            if (k >= (int)_left.size())
                _left.resize(k + 1, T{});
            return _left[k];
        }
    }

    // operator[] const — returns T{} for unwritten positions.
    T operator[](int i) const {
        if (!_offset.has_value()) return T{};
        int k = i - *_offset;
        if (k >= 0) {
            return (k < (int)_right.size()) ? _right[k] : T{};
        } else {
            k = -k - 1;
            return (k < (int)_left.size()) ? _left[k] : T{};
        }
    }

    // -----------------------------------------------------------------------
    // Index range — only valid after at least one write.
    // -----------------------------------------------------------------------
    int min_index() const {
        assert(_offset.has_value());
        return *_offset - (int)_left.size();
    }
    int max_index() const {
        assert(_offset.has_value());
        return *_offset + (int)_right.size() - 1;
    }

    std::size_t size() const { return _left.size() + _right.size(); }
    bool empty() const { return !_offset.has_value(); }

    // -----------------------------------------------------------------------
    // items() — iterate (index, value) in ascending index order.
    // -----------------------------------------------------------------------
    template <typename Fn>
    void items(Fn&& fn) const {
        if (!_offset.has_value()) return;
        for (int k = (int)_left.size() - 1; k >= 0; --k)
            fn(*_offset - 1 - k, _left[k]);
        for (int k = 0; k < (int)_right.size(); ++k)
            fn(*_offset + k, _right[k]);
    }

private:
    std::vector<T>   _right;   // value at offset + k
    std::vector<T>   _left;    // value at offset - 1 - k
    std::optional<int> _offset;
};

} // namespace roadgeometry
