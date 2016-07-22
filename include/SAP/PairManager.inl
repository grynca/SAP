#include "PairManager.h"
#include "base.h"

namespace grynca {
    
    inline PairManager::Pair::Pair()
#ifdef DEBUG_BUILD
     : id1(InvalidId())
#endif
    {}

    inline PairManager::PairManager(uint32_t initial_size)
     : pairs_count_(0), modulo_mask_(initial_size -1), initial_size_(initial_size)
    {
        ASSERT(isPowerOfTwo_(initial_size));
        hash_table_.resize(initial_size, InvalidId());
        pairs_.resize(initial_size);
#ifdef DEBUG_BUILD
        next_.resize(initial_size, InvalidId());
#else
        next_.resize(initial_size);
#endif
    }
    
    inline void PairManager::clear() {
        hash_table_.resize(initial_size_);
        pairs_.resize(initial_size_);
        next_.resize(initial_size_);
        std::fill(hash_table_.begin(), hash_table_.end(), InvalidId());
        std::fill(next_.begin(), next_.end(), InvalidId());
        pairs_count_ = 0;
        modulo_mask_ = initial_size_ -1;
    }
    
    inline const PairManager::Pair* PairManager::addPair(uint32_t id1, uint32_t id2) {
        sortIds_(id1, id2);
        uint32_t hash = hash_(id1, id2);
        Pair* p = findPairInternal_(id1, id2, hash);
        if (p)
            // already in manager
            return p;

        if (pairs_count_ >= hash_table_.size()) {
            uint32_t new_size = calcNextPowerOfTwo(pairs_count_+1);
            modulo_mask_ = new_size - 1;
            hash_table_.resize(new_size);
            std::fill(hash_table_.begin(), hash_table_.end(), InvalidId());
            pairs_.resize(new_size);
#ifdef DEBUG_BUILD
            next_.resize(new_size, InvalidId());
#else
            next_.resize(new_size);
#endif
            // recalc hashes for pairs
            for (uint32_t i=0; i<pairs_count_; ++i) {
                uint32_t new_hash = hash_(pairs_[i].id1, pairs_[i].id2);
                next_[i] = hash_table_[new_hash];
                hash_table_[new_hash] = i;
            }

            hash = hash_(id1, id2); // recompute hash for added pair with new modulo
        }

        p = &pairs_[pairs_count_];
        p->id1 = id1;
        p->id2 = id2;

        next_[pairs_count_] = hash_table_[hash];
        hash_table_[hash] = pairs_count_;
        ++pairs_count_;
        return p;
    }

    inline const PairManager::Pair* PairManager::findPair(uint32_t id1, uint32_t id2) {
        sortIds_(id1, id2);
        uint32_t hash = hash_(id1, id2);
        return findPairInternal_(id1, id2, hash);
    }

    inline bool PairManager::removePair(uint32_t id1, uint32_t id2) {
        sortIds_(id1, id2);
        uint32_t hash = hash_(id1, id2);
        Pair* p = findPairInternal_(id1, id2, hash);
        if (!p)
            return false;

        ASSERT(p->id1==id1);
        ASSERT(p->id2==id2);

        uint32_t pair_id = getPairIndex(p);

        removePairLink_(pair_id, hash);

        uint32_t last_pair_id = pairs_count_-1;
        if (last_pair_id != pair_id) {
            Pair* last_pair = &pairs_[last_pair_id];
            // remove last pair from its list
            uint32_t last_hash = hash_(last_pair->id1, last_pair->id2);
            removePairLink_(last_pair_id, last_hash);

            // move to freed slot and add its link to list
            pairs_[pair_id] = pairs_[last_pair_id];
            next_[pair_id] = hash_table_[last_hash];
            hash_table_[last_hash] = pair_id;
        }

        --pairs_count_;
        return true;
    }

    inline uint32_t PairManager::getPairIndex(const Pair* p) const {
        return (size_t(p) - size_t(&pairs_[0]))/sizeof(Pair);
    }

    inline void PairManager::sortIds_(uint32_t& id1, uint32_t& id2) {
        if (id1>id2)
            std::swap(id1, id2);
    }

    inline uint32_t PairManager::hash_(uint32_t id1, uint32_t id2) {
        return calcHash32((id1&0xffff)|(id1<<16)) & modulo_mask_;
    }

    inline bool PairManager::isPowerOfTwo_(uint32_t x) {
        while (((x % 2) == 0) && x > 1)
            x /= 2;
        return (x == 1);
    }

    
    inline bool PairManager::cmpPairs_(const Pair& p, uint32_t id1, uint32_t id2) const {
        return (id1 == p.id1) && (id2 == p.id2);
    }
    
    inline PairManager::Pair* PairManager::findPairInternal_(uint32_t id1, uint32_t id2, uint32_t hash) {
        uint32_t offset = hash_table_[hash];
        while (offset != InvalidId()) {

            if (cmpPairs_(pairs_[offset], id1, id2)) {
                return &pairs_[offset];
            }
            offset = next_[offset];
        }
        return NULL;
    }
    
    inline void PairManager::removePairLink_(uint32_t pair_id, uint32_t hash) {
        // remove from linked list and stitch
        uint32_t prev_offset = InvalidId();
        uint32_t offset = hash_table_[hash];

        while (offset != pair_id) {
            prev_offset = offset;
            offset = next_[offset];
        }

        if (prev_offset == InvalidId()) {
            hash_table_[hash] = next_[pair_id];
        }
        else {
            ASSERT(next_[prev_offset] == pair_id);
            next_[prev_offset] = next_[pair_id];
        }

#ifdef DEBUG_BUILD
        next_[pair_id]=InvalidId();
#endif
    }

}