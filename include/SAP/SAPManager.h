#ifndef SAP_MANAGER_H
#define SAP_MANAGER_H

#include "types/containers/Array.h"
#include "types/Path.h"
#include <vector>
#include "SAPSegment.h"
#ifdef VISUAL_DEBUG
#   include "assets/Image.h"
#endif

namespace grynca {

    // provides asymetrical collision setup (e.g.: G1->G2 collide, but G2->G1 dont collide)
    // update fast (exploits time coherence from previous frames)
    // insertion, deletion, changing collision group flags slow (up-to 10k boxes ok)

    template <typename ClientData, uint32_t AXES_COUNT>
    class SAPManager {
        typedef SAP::Box_<ClientData, AXES_COUNT> Box;
        typedef SAPSegment<ClientData, AXES_COUNT> Segment;
    public:
        SAPManager();
        ~SAPManager();

        SAP::GroupFlags & getCollisionFlags(uint32_t group_id);  // default: all true
        void setCollisionFlags(uint32_t group_id, bool value, const fast_vector<uint32_t>& group_ids);
        void updateCollisionFlags();    // updates current collisions after changing collision flags -> loops through all boxes, can be slow

        uint32_t addBox(SAP::Extent* extents, uint32_t coll_group_id, const ClientData& client_data);       // returns box id
        void updateBox(uint32_t box_id, SAP::Extent* extents);
        void moveBox(uint32_t box_id, float* move_vec);
        void removeBox(uint32_t box_id);
        void getBox(uint32_t box_id, SAP::Extent* extents);
        bool tryGetBox(uint32_t box_id, SAP::Extent* extents);
        ClientData& getClientData(uint32_t box_id);
        uint32_t getGroupId(uint32_t box_id);
        uint32_t getBoxesCount();
        uint32_t getBoxesPoolSize();        // with holes included


        Segment* getRootSegment();
        // for debug
        void validate();
        uint32_t getOverlapsCount();
        std::string debugPrint();
#ifdef VISUAL_DEBUG
        Image::Ref debugImage();
#endif
    private:
        template <typename, uint32_t> friend class SAPSegment;

        struct DeferredAfterUpdate {
            fast_vector<Segment*> crossings_;
            fast_vector<Segment*> merges_;
        };

        Box* getBox_(uint32_t box_id);
        bool boxesOverlap_(Box& b1, Box& b2);
        void debugPrintSegmentRec_(const std::string& name, std::vector<bool>& path, Segment* s, std::ostream& os, uint32_t& segs_cnt, uint32_t* splits_count);
        float debugFindLowPointRec_(Segment* seg, uint32_t a);
        float debugFindHighPointRec_(Segment* seg, uint32_t a);
        void afterUpdate_(Box* box, uint32_t box_id, SAP::Extent* extents);


        Segment* root_;
        Pool boxes_;

        SAP::Overlaps overlaps_;
        DeferredAfterUpdate deferred_after_update_;

        SAP::GroupFlags collision_groups_matrix_[SAP::MAX_COLLISION_GROUPS];
        SAP::GroupFlags collision_flags_changed_;
    };

    template <typename ClientData>
    class SAPManager2D : public SAPManager<ClientData, 2> {};

    template <typename ClientData>
    class SAPManager3D : public SAPManager<ClientData, 3> {};
}

#include "SAPManager.inl"
#endif //SAP_MANAGER_H
