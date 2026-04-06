#pragma once

#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>
#include <queue>

namespace roadgeometry {

/**
 * Priority queue with decrease-key support via lazy deletion.
 *
 * Maps keys of type Key to priorities of type Priority. Supports O(log n)
 * push/decrease-key and O(log n) amortized pop_min. Internally maintains a
 * min-heap of (priority, key) pairs alongside an unordered_map for O(1)
 * membership and priority lookup. Stale heap entries (superseded by a
 * decrease-key) are discarded lazily during pop_min.
 *
 * @tparam Key       Node/key type. Must be hashable (std::hash<Key> must exist).
 * @tparam Priority  Priority type. Must be totally ordered. Defaults to double.
 * @tparam Hash      Hash function for Key. Defaults to std::hash<Key>.
 */
template<
    typename Key,
    typename Priority = double,
    typename Hash = std::hash<Key>
>
class PriorityQueue {
public:
    using key_type      = Key;
    using priority_type = Priority;

    PriorityQueue() = default;

    /** Number of active entries (keys currently in the queue). */
    std::size_t size() const { return map_.size(); }

    bool empty() const { return map_.empty(); }

    /** Returns true if key is currently in the queue. */
    bool contains(const Key& key) const {
        return map_.count(key) > 0;
    }

    /**
     * Returns the current priority of key, or default_val if not present.
     */
    Priority get(const Key& key, Priority default_val = std::numeric_limits<Priority>::infinity()) const {
        auto it = map_.find(key);
        return it != map_.end() ? it->second : default_val;
    }

    /**
     * Insert key with given priority, or decrease its priority if already present.
     * If the new priority is >= the existing priority, this is a no-op.
     */
    void push(const Key& key, Priority priority) {
        auto it = map_.find(key);
        if (it != map_.end() && it->second <= priority)
            return;  // existing priority is at least as good

        map_[key] = priority;
        heap_.push({priority, key});
    }

    /**
     * Returns the key with the smallest priority without removing it.
     * Skips stale heap entries (lazily).
     */
    const Key& peek_min() {
        prune();
        return heap_.top().second;
    }

    /**
     * Removes and returns the key with the smallest priority.
     */
    Key pop_min() {
        prune();
        if (heap_.empty())
            throw std::out_of_range("pop_min on empty PriorityQueue");

        Key key = heap_.top().second;
        heap_.pop();
        map_.erase(key);
        return key;
    }

private:
    using Pair = std::pair<Priority, Key>;

    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> heap_;
    std::unordered_map<Key, Priority, Hash> map_;

    /** Discard stale heap entries until the top is a live entry. */
    void prune() {
        while (!heap_.empty()) {
            const auto& [priority, key] = heap_.top();
            auto it = map_.find(key);
            if (it != map_.end() && it->second == priority)
                break;  // live entry
            heap_.pop();
        }
    }
};

}  // namespace roadgeometry
