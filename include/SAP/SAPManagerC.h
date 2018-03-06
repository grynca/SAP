#ifndef SAP_MANAGER_H
#define SAP_MANAGER_H

#include "SAP_internal.h"
#include "types/containers/Array.h"
#include "types/Path.h"
#include <vector>
#ifdef USE_SDL2
#   include "assets/Image.h"
#endif

namespace grynca {

    template <typename SAPDomain>
    class SAPManagerBase {
    public:
        SAP_DOMAIN_TYPES(SAPDomain);

        SAPManagerBase();
        ~SAPManagerBase();

        bool containsBox(Index box_id)const;
        Box& accBox(Index box_id);
        const Box& getBox(Index box_id)const;
        u32 getGroupId(u32 box_id)const;
        u32 getBoxesCount()const;
        Segment* getRootSegment()const;

        u32 getOverlapsCount()const;
        void getOverlap(u32 overlap_id, Index& b1_id_out, Index& b2_id_out)const;
        void getOverlapWithData(u32 overlap_id, Index& b1_id_out, Index& b2_id_out, OverlapDataT*& coll_data_ptr_out);
        OverlapDataT* findOverlap(Index b1_id, Index b2_id);
        OverlapDataT* findOverlap(Box& b1, Box& b2);
        Raycaster getRayCaster()const;

        void calcBounds(f32* bounds);
        // for debug
        void validate();
        std::string debugPrint();
#ifdef USE_SDL2
        Image::Ref debugImage();        // only for 2D
        // when viewport is not specified, it is automatically computed to show all boxes
        void debugRender(SDL_Window* w, SDL_Renderer* r, const f32* bounds = NULL, const Color& box_color = Color::Red(), const Color& segment_color = Color::White());   // only for 2D
#endif
        void clear();
    protected:
        friend Segment;
        friend Raycaster;

        template <typename Derived>
        Box& addBoxInner_(Index& box_id_out, f32* bounds, const BoxDataT& box_data);
        template <typename Derived>
        void updateBoxInner_(Index box_id, f32* bounds);
        template <typename Derived>
        void moveBoxInner_(Index box_id, f32* move_vec);
        template <typename Derived>
        void removeBoxInner_(Index box_id);

        template <typename Derived>
        void afterUpdate_(Box& box, Index box_id, const f32* bounds);
        template <typename Derived>
        void addOverlaps_(Box& box, Index box_id);
        template <typename Derived>
        void removeOverlaps_(Box& box, Index box_id);

        template <typename Derived>
        Derived& getAs_() { return (*(Derived*)this); }

        struct DeferredAfterUpdate {
            bool crossing_;
            fast_vector<Segment*> merges_;
        };

        bool boxesOverlap_(Box& b1, Box& b2);
        void debugPrintSegmentRec_(const std::string& name, std::vector<bool>& path, Segment* s, std::ostream& os, u32& segs_cnt, u32* splits_count);

        Segment* root_;
        TightArray<Box> boxes_;

        SAP::Overlaps<OverlapDataT> overlaps_;
        DeferredAfterUpdate deferred_after_update_;
    };

    // update fast (exploits time coherence from previous frames)
    // insertion, deletion, changing collision group flags slower (up-to 10k boxes ok)
    template <typename Derived /*CRTP*/, typename SAPDomain>
    class SAPManagerC : public SAPManagerBase<SAPDomain> {
    public:
        SAP_DOMAIN_TYPES(SAPDomain);

        // bounds is f32 coords array [LTx, LTy, ... , RBx, RBy, ...] (LeftTop, RightBot)
        Box& addBox(Index& box_id_out, f32* bounds, const BoxDataT& box_data);
        void updateBox(Index box_id, f32* bounds);
        void moveBox(Index box_id, f32* move_vec);
        void removeBox(Index box_id);
    protected:
        friend Manager;

        // available for override
        bool beforeBoxesOverlap_(Box&, Box&) { return true; }
        void afterBoxesOverlap_(Box&, Box&) {}
    };

    template <typename SAPDomain>
    class SAPManagerSimple2D : public SAPManagerC<SAPManagerSimple2D<SAPDomain>, SAPDomain> {};

    template <typename SAPDomain>
    class SAPManagerSimple3D : public SAPManagerC<SAPManagerSimple3D<SAPDomain>, SAPDomain> {};
}

#include "SAPManagerC.inl"
#endif //SAP_MANAGER_H
