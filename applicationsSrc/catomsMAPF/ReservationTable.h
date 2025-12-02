#ifndef RESERVATION_TABLE_H
#define RESERVATION_TABLE_H

#include <unordered_set>
#include <map>
#include "robots/catoms3D/catoms3DWorld.h"

using namespace Catoms3D;

// Global space-time reservation table for MAPF
class ReservationTable {
public:
    // Each entry = (cell,t)
    struct Key {
        Cell3DPosition pos;
        int t;
        bool operator==(const Key &o) const {
            return pos == o.pos && t == o.t;
        }
    };

    struct KeyHash {
        std::size_t operator()(const Key &k) const noexcept {
            // Combine 3D cell and time into a single hash
            return ((k.pos[0] & 0x3FF) << 20)
                 ^ ((k.pos[1] & 0x3FF) << 10)
                 ^ (k.pos[2] & 0x3FF)
                 ^ (k.t << 2);
        }
    };

    std::unordered_set<Key, KeyHash> occ; // vertex reservations
    std::unordered_set<uint64_t> edge;    // edge reservations

    static uint64_t encodeEdge(const Cell3DPosition &a,
                               const Cell3DPosition &b, int t) {
        return ((uint64_t)(a[0] & 0xFFF) << 48)
             | ((uint64_t)(a[1] & 0xFFF) << 36)
             | ((uint64_t)(a[2] & 0xFFF) << 24)
             | ((uint64_t)(b[0] & 0xFFF) << 12)
             | ((uint64_t)t & 0xFFF);
    }

    bool isOccupied(const Cell3DPosition &p, int t) const {
        return occ.count({p, t});
    }

    bool isEdgeReserved(const Cell3DPosition &a,
                        const Cell3DPosition &b, int t) const {
        return edge.count(encodeEdge(a, b, t))
            || edge.count(encodeEdge(b, a, t)); // prevent swaps
    }

    void reserve(const Cell3DPosition &p, int t) { occ.insert({p, t}); }

    void reserveEdge(const Cell3DPosition &a,
                     const Cell3DPosition &b, int t) {
        edge.insert(encodeEdge(a, b, t));
    }

    void clear() {
        occ.clear();
        edge.clear();
    }
};

#endif
