#ifndef SAP_MANAGER_H
#define SAP_MANAGER_H

#include "types/containers/Array.h"
#include "types/Path.h"
#include <vector>
#include "SAPSegment.h"
#include "SAPRaycaster.h"
#ifdef VISUAL_DEBUG
#   include "assets/Image.h"
#endif

namespace grynca {

    // update fast (exploits time coherence from previous frames)
    // insertion, deletion, changing collision group flags slow (up-to 10k boxes ok)

    template <typename ClientData, uint32_t AXES_COUNT>
    class SAPManager {
        typedef SAP::Box_<ClientData, AXES_COUNT> Box;
        typedef SAPSegment<ClientData, AXES_COUNT> Segment;
        typedef SAPRaycaster<ClientData, AXES_COUNT> Raycaster;
    public:
        SAPManager();
        ~SAPManager();

        // bounds is float coords array [LTx, LTy, ... , RBx, RBy, ...] (LeftTop, RightBot)
        uint32_t addBox(float* bounds, const ClientData& client_data);       // returns box id
        void updateBox(uint32_t box_id, float* bounds);
        void moveBox(uint32_t box_id, float* move_vec);
        void removeBox(uint32_t box_id);
        void getBox(uint32_t box_id, float* bounds);
        bool tryGetBox(uint32_t box_id, float* bounds);
        ClientData& getClientData(uint32_t box_id);
        uint32_t getGroupId(uint32_t box_id);
        uint32_t getBoxesCount();
        uint32_t getBoxesPoolSize();        // with holes included
        Segment* getRootSegment();

        uint32_t getOverlapsCount();
        void getOverlap(uint32_t overlap_id, uint32_t& b1_id_out, uint32_t& b2_id_out);
        Raycaster getRayCaster();

        void calcBounds(float* bounds);
        // for debug
        void validate();
        std::string debugPrint();
#ifdef VISUAL_DEBUG
        Image::Ref debugImage();
        void debugRender(SDL_Window* w, SDL_Renderer* r);
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
        void afterUpdate_(Box* box, uint32_t box_id, float* bounds);


        Segment* root_;
        Pool boxes_;

        SAP::Overlaps overlaps_;
        DeferredAfterUpdate deferred_after_update_;
    };

    template <typename ClientData>
    class SAPManager2D : public SAPManager<ClientData, 2> {};

    template <typename ClientData>
    class SAPManager3D : public SAPManager<ClientData, 3> {};
}

#include "SAPManager.inl"
#endif //SAP_MANAGER_H
