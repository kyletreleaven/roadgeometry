#pragma once
#include <concepts>
#include <cstddef>

namespace roadgeometry {

// ---------------------------------------------------------------------------
// PopQueue concept — minimal interface for a pop-only queue of int indices.
// ---------------------------------------------------------------------------
template <typename Q>
concept PopQueue = requires(Q q) {
    { q.size()      } -> std::convertible_to<std::size_t>;
    { q.front()     } -> std::convertible_to<int>;
    q.pop_front();
};

// ---------------------------------------------------------------------------
// BiPartiteQueues concept — anything exposing supply() and demand() returning
// a PopQueue.  Method access is strictly more permissive than member access.
// ---------------------------------------------------------------------------
template <typename B>
concept BiPartiteQueues =
    PopQueue<decltype(std::declval<B>().supply())> &&
    PopQueue<decltype(std::declval<B>().demand())>;

// ---------------------------------------------------------------------------
// BiPartite — canonical struct implementation of BiPartiteQueues.
// ---------------------------------------------------------------------------
template <PopQueue Queue>
struct BiPartite {
    Queue supply_;
    Queue demand_;

    Queue& supply() { return supply_; }
    Queue& demand() { return demand_; }
};

} // namespace roadgeometry
