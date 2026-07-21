#pragma once

// Private implementation helpers shared across the mccf module. Not public API.
namespace roadgeometry::mccf::detail {

// Map lookup with a default for missing keys.
template <class M, class K>
double map_get(const M& m, const K& k, double def) {
    auto it = m.find(k);
    return it != m.end() ? it->second : def;
}

} // namespace roadgeometry::mccf::detail
