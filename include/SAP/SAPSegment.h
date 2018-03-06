#ifndef SAPSEGMENT_H
#define SAPSEGMENT_H

#include "SAP_internal.h"
#include "../SAP.h"

namespace grynca {

    template <typename SAPDomain>
    class SAPSegment {
    public:
        SAP_DOMAIN_TYPES(SAPDomain);

        SAPSegment(Manager& mgr, Segment* parent);
        ~SAPSegment();

        void addBox(Box& box, Index box_id);

        // returns true if moved out of segment
        bool moveBox(Box& box, Index box_id, SAP::MinMax* old_min_max_ids, f32* move_vec, typename Manager::DeferredAfterUpdate& dau);
        bool updateBox(Box& box, Index box_id, SAP::MinMax* old_min_max_ids, typename Manager::DeferredAfterUpdate& dau);
        void removeBox(Box& box, Index box_id, SAP::MinMax* min_max_ids);


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
        friend Box;
        friend Raycaster;
        friend Manager;

        template <typename AddedCb>
        void addBoxTree_(const f32* bounds, const AddedCb& cb);
        u32 bisectInsertFind_(SAP::Points& points, f32 val, u32 from, u32 to);
        void findOverlapsOnAxis_(Box& box, u32 axis);
        u32 getScanStartId_(u32 min_id, u32 axis);   // if from where we must scan for overlaps
        void insertSingleAxis_(Box& new_box, u32 new_box_inner_id, u32 axis);
        void addBoxInner_(Box& box, u32 box_inner_id);
        void removeBoxInner_(Box& box, Index box_id, SAP::MinMax* min_max_ids);
        SAP::LongestSide findLongestSide_(u32 axis);
        void moveMinMaxPoints_(Box& box, Index box_id, SAP::MinMax* old_min_max_ids, f32* move_vec, u32 axis, f32 low, f32 high, bool& crossing_out, bool& oos_out);
        void updateMinMaxPoints_(Box& box, Index box_id, SAP::MinMax* old_min_max_ids, u32 axis, f32 low, f32 high, bool& crossing_out);
        u32 moveMinRight_(u32 point_id, f32 new_value, u32 axis);
        u32 moveMaxRight_(u32 point_id, f32 new_value, u32 axis);
        u32 moveMinLeft_(i32 point_id, f32 new_value, u32 axis);
        u32 moveMaxLeft_(i32 point_id, f32 new_value, u32 axis);
        void split_();
        void merge_(Segment* removed_child);
        void pointToChild_(Segment* child, u32 a, u32 point_id);
        void setDebugName_();
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

        SAPSegment<SAPDomain>* children_[2];

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
