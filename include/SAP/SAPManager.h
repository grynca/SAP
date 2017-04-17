#ifndef SAP_MANAGER_H
#define SAP_MANAGER_H

#include "types/containers/Array.h"
#include "types/Path.h"
#include <vector>
#include "SAPSegment.h"
#include "SAPRaycaster.h"
#ifdef USE_SDL2
#   include "assets/Image.h"
#endif

namespace grynca {

    // update fast (exploits time coherence from previous frames)
    // insertion, deletion, changing collision group flags slow (up-to 10k boxes ok)

    template <typename ClientData, u32 AXES_COUNT>
    class SAPManager {
        typedef SAP::Box_<ClientData, AXES_COUNT> Box;
        typedef SAPSegment<ClientData, AXES_COUNT> Segment;
        typedef SAPRaycaster<ClientData, AXES_COUNT> Raycaster;
    public:
        SAPManager();
        ~SAPManager();

        // bounds is f32 coords array [LTx, LTy, ... , RBx, RBy, ...] (LeftTop, RightBot)
        u32 addBox(f32* bounds, const ClientData& client_data);       // returns box id
        void updateBox(u32 box_id, f32* bounds);
        void moveBox(u32 box_id, f32* move_vec);
        void removeBox(u32 box_id);
        void getBox(u32 box_id, f32* bounds);
        bool tryGetBox(u32 box_id, f32* bounds);
        ClientData& getClientData(u32 box_id);
        u32 getGroupId(u32 box_id);
        u32 getBoxesCount();
        u32 getBoxesPoolSize();        // with holes included
        Segment* getRootSegment();

        u32 getOverlapsCount();
        void getOverlap(u32 overlap_id, u32& b1_id_out, u32& b2_id_out);
        Raycaster getRayCaster();

        void calcBounds(f32* bounds);
        // for debug
        void validate();
        std::string debugPrint();
#ifdef USE_SDL2
        Image::Ref debugImage();
        void debugRender(SDL_Window* w, SDL_Renderer* r);
#endif
    private:
        template <typename, u32> friend class SAPSegment;

        struct DeferredAfterUpdate {
            fast_vector<Segment*> crossings_;
            fast_vector<Segment*> merges_;
        };

        Box* getBox_(u32 box_id);
        bool boxesOverlap_(Box& b1, Box& b2);
        void debugPrintSegmentRec_(const std::string& name, std::vector<bool>& path, Segment* s, std::ostream& os, u32& segs_cnt, u32* splits_count);
        void afterUpdate_(Box* box, u32 box_id, f32* bounds);


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
