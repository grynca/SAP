#ifndef SAPSEGMENT_H
#define SAPSEGMENT_H

#include "SAP_internal.h"
#include "../SAP.h"

namespace grynca {

    // fw
    template <typename, u32> class SAPManager;
    template <typename, u32> class SAPRaycaster;

    template <typename ClientData, u32 AXES_COUNT>
    class SAPSegment {
        typedef SAPManager<ClientData, AXES_COUNT> Manager;
        typedef SAP::Box_<ClientData, AXES_COUNT> Box;
        typedef SAPSegment<ClientData, AXES_COUNT> Segment;
    public:
        SAPSegment(Manager& mgr, Segment* parent);
        ~SAPSegment();

        void addBox(Box* box, u32 box_id, f32* bounds);

        // returns true if moved out of segment
        bool moveBox(Box* box, u32 box_id, SAP::MinMax* old_min_max_ids, f32* bounds, f32* move_vec, typename Manager::DeferredAfterUpdate& dau);
        bool updateBox(Box* box, u32 box_id, SAP::MinMax* old_min_max_ids, f32* bounds, typename Manager::DeferredAfterUpdate& dau);
        void removeBox(Box* box, u32 box_id, SAP::MinMax* min_max_ids);


        Segment* getParent() { return parent_; }
        Segment* getChild(u32 id);
        Segment* getPrevNeighbor(u32 axis);        // returns largest neighboring segment (may not be leaf)
        Segment* getNextNeighbor(u32 axis);
        Segment* getSplitNeighbor();
        Segment* getSplitNeighbor(u32& child_id_out);       // returns also neighbors child id (0 or 1)

        // segment borders
        bool getLowBorder(u32 axis, f32& lb_out);
        bool getHighBorder(u32 axis, f32& hb_out);

        bool isSplit();
        u32 getBoxesCount();
        f32 getSplitValue() { return split_value_; }
        u8 getSplitAxis() { return split_axis_; }

        void getCrossedBoxes(fast_vector<u32>& crossed_out);
        std::string getDebugName();
    private:
        template <typename, u32> friend class SAPManager;
        template <typename, u32> friend class SAP::Box_;
        template <typename, u32> friend class SAPRaycaster;

        u32 bisectInsertFind_(SAP::Points& points, f32 val, u32 from, u32 to);
        void findOverlapsOnAxis_(Box* box, u32 axis);
        u32 getScanStartId_(u32 min_id, u32 axis);   // if from where we must scan for overlaps
        void insertSingleAxis_(Box* new_box, u32 new_box_id, f32 min_val, f32 max_val, u32 axis);
        void addBoxInner_(Box* box, u32 box_id, f32* bounds);
        void removeBoxInner_(Box* box, u32 box_id, SAP::MinMax* min_max_ids);
        SAP::LongestSide findLongestSide_(u32 axis);
        void moveMinMaxPoints_(Box* box, u32 box_id, SAP::MinMax* old_min_max_ids, f32* bounds, f32* move_vec, u32 axis, f32 low, f32 high, typename Manager::DeferredAfterUpdate& dau);
        void updateMinMaxPoints_(Box* box, u32 box_id, SAP::MinMax* old_min_max_ids, f32* bounds, u32 axis, f32 low, f32 high, typename Manager::DeferredAfterUpdate& dau);
        u32 moveMinRight_(u32 point_id, f32 new_value, u32 axis);
        u32 moveMaxRight_(u32 point_id, f32 new_value, u32 axis);
        u32 moveMinLeft_(i32 point_id, f32 new_value, u32 axis);
        u32 moveMaxLeft_(i32 point_id, f32 new_value, u32 axis);
        void split_();
        void merge_(Segment* removed_child);
        void pointToChild_(Segment* child, u32 a, u32 point_id);
        void setDebugName_();
        void getPrevOverlappingLeafsRec_(u32 axis, f32* bounds, Segment* seg, fast_vector<Segment*>& leafs_out);
        void getNextOverlappingLeafsRec_(u32 axis, f32* bounds, Segment* seg, fast_vector<Segment*>& leafs_out);
        void startCrossingLow_(f32* bounds, u32 axis, typename Manager::DeferredAfterUpdate& dau);
        void startCrossingHigh_(f32* bounds, u32 axis, typename Manager::DeferredAfterUpdate& dau);
        void calcBorders_();
        bool calcLowBorder_(u32 axis, f32& val);
        bool calcHighBorder_(u32 axis, f32& val);
        f32 findLowestPointRec_(u32 a);
        f32 findHighestPointRec_(u32 a);

        Manager* manager_;
        Segment* parent_;

#ifdef DEBUG_BUILD
        std::string debug_name_;
#endif

        u8 split_axis_;
        f32 split_value_;

        SAP::Points points_[AXES_COUNT];       // sorted from low to high
        SAP::LongestSide longest_sides_[AXES_COUNT];

        SAPSegment<ClientData, AXES_COUNT>* children_[2];

        struct {
            f32 low;
            f32 high;
            bool has_low;
            bool has_high;
        } borders_[AXES_COUNT];
    };
}

#include "SAPSegment.inl"
#endif //SAPSEGMENT_H
