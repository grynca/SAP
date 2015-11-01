#ifndef SAP_MANAGER_H
#define SAP_MANAGER_H

#include "types/containers/UnsortedVector.h"
#include "SAP_internal.h"
#include <vector>

namespace grynca {

    static const uint32_t MAX_COLLISION_GROUPS = 512;

    struct Extent {
        float min;
        float max;
    };

    typedef std::bitset<MAX_COLLISION_GROUPS> GroupFlags;

    // provides asymetrical collision setup (e.g.: G1->G2 collide, but G2->G1 dont collide)
    // update fast (exploits time coherence from previous frames)
    // insertion, deletion, changing collision group flags slow (up-to 10k boxes ok)

    template <typename ClientData, uint32_t AXES_COUNT=3>
    class SAPManager {
        typedef SAP_internal::Box_<ClientData, AXES_COUNT> Box;

    public:
        SAPManager();

        GroupFlags & getCollisionFlags(uint32_t group_id);  // default: all true
        void setCollisionFlags(uint32_t group_id, bool value, const fast_vector<uint32_t>& group_ids);

        uint32_t addBox(Extent* extents, uint32_t coll_group_id, const ClientData& client_data);       // returns box id
        void updateBox(uint32_t box_id, Extent* extents);
        void removeBox(uint32_t box_id);
        void getBox(uint32_t box_id, Extent* extents);
        ClientData& getClientData(uint32_t box_id);
        const fast_vector<uint32_t>& getOverlaps(uint32_t box_id);
        uint32_t getGroupId(uint32_t box_id);
        uint32_t getBoxesCount();
        void updateCollisionFlags();

        // for debug
        void validate();
        uint32_t getOverlapsCount();
    private:
        void updateMinMaxPoints_(uint32_t box_id, float min_value, float max_value, uint32_t axis);
        uint32_t movePointRight_(int32_t point_id, float new_value, uint32_t axis);
        uint32_t movePointLeft_(int32_t point_id, float new_value, uint32_t axis);
        void insertSingleAxis(uint32_t new_box_id, float min_val, float max_val, uint32_t axis);
        void findOverlapsOnAxis(uint32_t box_id, uint32_t axis);
        uint32_t bisectInsertFind(SAP_internal::Points& points, float val, uint32_t from, uint32_t to);
        SAP_internal::LongestSide findLongestSide(uint32_t axis);
        bool boxesOverlap_(Box& b1, Box& b2);
        bool intervalsOverlap_(uint32_t min1, uint32_t max1, uint32_t min2, uint32_t max2);
        uint32_t getScanStartId_(uint32_t min_id, uint32_t axis);   // if from where we must scan for overlaps
        void clearInterestingFlags();

        UnsortedVector<Box> boxes_;
        // sorted from low to high
        SAP_internal::Points points_[AXES_COUNT];
        SAP_internal::LongestSide longest_sides_[AXES_COUNT];

        std::vector<bool> interesting_ids_flags_;
        fast_vector<uint32_t> interesting_ids_;

        GroupFlags collision_groups_matrix_[MAX_COLLISION_GROUPS];
        GroupFlags collision_flags_changed_;
    };

}

#include "SAP.inl"
#endif //SAP_MANAGER_H
