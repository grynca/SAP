#ifndef SAPSEGMENT_H
#define SAPSEGMENT_H

#include "SAP_internal.h"
#include "../SAP.h"

namespace grynca {

    // fw
    template <typename, uint32_t> class SAPManager;
    template <typename, uint32_t> class SAPRaycaster;

    template <typename ClientData, uint32_t AXES_COUNT>
    class SAPSegment {
        typedef SAPManager<ClientData, AXES_COUNT> Manager;
        typedef SAP::Box_<ClientData, AXES_COUNT> Box;
        typedef SAPSegment<ClientData, AXES_COUNT> Segment;
    public:
        SAPSegment(Manager& mgr, Segment* parent);
        ~SAPSegment();

        void addBox(Box* box, uint32_t box_id, float* bounds);

        // returns true if moved out of segment
        bool moveBox(Box* box, uint32_t box_id, SAP::MinMax* old_min_max_ids, float* bounds, float* move_vec, typename Manager::DeferredAfterUpdate& dau);
        bool updateBox(Box* box, uint32_t box_id, SAP::MinMax* old_min_max_ids, float* bounds, typename Manager::DeferredAfterUpdate& dau);
        void removeBox(Box* box, uint32_t box_id, SAP::MinMax* min_max_ids);


        Segment* getParent() { return parent_; }
        Segment* getChild(uint32_t id);
        Segment* getPrevNeighbor(uint32_t axis);        // returns largest neighboring segment (may not be leaf)
        Segment* getNextNeighbor(uint32_t axis);
        Segment* getSplitNeighbor();
        Segment* getSplitNeighbor(uint32_t& child_id_out);       // returns also neighbors child id (0 or 1)

        // segment borders
        bool getLowBorder(uint32_t axis, float& lb_out);
        bool getHighBorder(uint32_t axis, float& hb_out);

        bool isSplit();
        uint32_t getBoxesCount();
        float getSplitValue() { return split_value_; }
        int8_t getSplitAxis() { return split_axis_; }

        void getCrossedBoxes(fast_vector<uint32_t>& crossed_out);
        std::string getDebugName();
    private:
        template <typename, uint32_t> friend class SAPManager;
        template <typename, uint32_t> friend class SAP::Box_;
        template <typename, uint32_t> friend class SAPRaycaster;

        uint32_t bisectInsertFind_(SAP::Points& points, float val, uint32_t from, uint32_t to);
        void findOverlapsOnAxis_(Box* box, uint32_t axis);
        uint32_t getScanStartId_(uint32_t min_id, uint32_t axis);   // if from where we must scan for overlaps
        void insertSingleAxis_(Box* new_box, uint32_t new_box_id, float min_val, float max_val, uint32_t axis);
        void addBoxInner_(Box* box, uint32_t box_id, float* bounds);
        void removeBoxInner_(Box* box, uint32_t box_id, SAP::MinMax* min_max_ids);
        SAP::LongestSide findLongestSide_(uint32_t axis);
        void moveMinMaxPoints_(Box* box, uint32_t box_id, SAP::MinMax* old_min_max_ids, float* bounds, float* move_vec, uint32_t axis, float low, float high, typename Manager::DeferredAfterUpdate& dau);
        void updateMinMaxPoints_(Box* box, uint32_t box_id, SAP::MinMax* old_min_max_ids, float* bounds, uint32_t axis, float low, float high, typename Manager::DeferredAfterUpdate& dau);
        uint32_t moveMinRight_(uint32_t point_id, float new_value, uint32_t axis);
        uint32_t moveMaxRight_(uint32_t point_id, float new_value, uint32_t axis);
        uint32_t moveMinLeft_(int32_t point_id, float new_value, uint32_t axis);
        uint32_t moveMaxLeft_(int32_t point_id, float new_value, uint32_t axis);
        void split_();
        void merge_(Segment* removed_child);
        void pointToChild_(Segment* child, uint32_t a, uint32_t point_id);
        void setDebugName_();
        void getPrevOverlappingLeafsRec_(uint32_t axis, float* bounds, Segment* seg, fast_vector<Segment*>& leafs_out);
        void getNextOverlappingLeafsRec_(uint32_t axis, float* bounds, Segment* seg, fast_vector<Segment*>& leafs_out);
        void startCrossingLow_(float* bounds, uint32_t axis, typename Manager::DeferredAfterUpdate& dau);
        void startCrossingHigh_(float* bounds, uint32_t axis, typename Manager::DeferredAfterUpdate& dau);
        void calcBorders_();
        bool calcLowBorder_(uint32_t axis, float& val);
        bool calcHighBorder_(uint32_t axis, float& val);
        float findLowestPointRec_(uint32_t a);
        float findHighestPointRec_(uint32_t a);

        Manager* manager_;
        Segment* parent_;

#ifdef DEBUG_BUILD
        std::string debug_name_;
#endif

        int8_t split_axis_;
        float split_value_;

        SAP::Points points_[AXES_COUNT];       // sorted from low to high
        SAP::LongestSide longest_sides_[AXES_COUNT];

        SAPSegment<ClientData, AXES_COUNT>* children_[2];

        struct {
            float low;
            float high;
            bool has_low;
            bool has_high;
        } borders_[AXES_COUNT];
    };
}

#include "SAPSegment.inl"
#endif //SAPSEGMENT_H
