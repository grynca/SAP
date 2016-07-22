#ifndef PAIRMANAGER_H
#define PAIRMANAGER_H

#include "types/containers/fast_vector.h"
#include <stdint.h>

namespace grynca {

    class PairManager {
    public:
        struct Pair {
            Pair();

            uint32_t id1;
            uint32_t id2;
        };

    public:
        PairManager(uint32_t initial_size);

        void clear();
        const Pair* addPair(uint32_t id1, uint32_t id2);
        const Pair* findPair(uint32_t id1, uint32_t id2);
        bool removePair(uint32_t id1, uint32_t id2);        // return true if pair was found and removed

        uint32_t getPairIndex(const Pair* p) const;

        uint32_t getPairsCount() const { return pairs_count_; }
        const Pair* getPairs() const { return &pairs_[0]; }
    private:
        void sortIds_(uint32_t& id1, uint32_t& id2);
        uint32_t hash_(uint32_t id1, uint32_t id2);
        bool isPowerOfTwo_(uint32_t x);
        bool cmpPairs_(const Pair& p, uint32_t id1, uint32_t id2) const;
        Pair* findPairInternal_(uint32_t id1, uint32_t id2, uint32_t hash);
        void removePairLink_(uint32_t pair_id, uint32_t hash);

        fast_vector<uint32_t> hash_table_;
        fast_vector<Pair> pairs_;
        fast_vector<uint32_t> next_;
        uint32_t pairs_count_;
        uint32_t modulo_mask_;
        uint32_t initial_size_;
    };

}

#include "PairManager.inl"
#endif //PAIRMANAGER_H
