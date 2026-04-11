#pragma once
#include <functional>
#include <utility>

namespace roadgeometry {

// ---------------------------------------------------------------------------
// PairHash
//
// Combines the hashes of both elements of a std::pair using a
// boost-style hash_combine.  Usable as the Hash template argument
// of std::unordered_map / std::unordered_set.
// ---------------------------------------------------------------------------
struct PairHash {
    template <typename A, typename B>
    std::size_t operator()(const std::pair<A, B>& p) const noexcept {
        std::size_t h = std::hash<A>{}(p.first);
        h ^= std::hash<B>{}(p.second) + 0x9e3779b9u + (h << 6) + (h >> 2);
        return h;
    }
};

} // namespace roadgeometry
