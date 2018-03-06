#include <SAP.h>
#include "SAPSegment.h"
#include "SAPManagerC.h"
#include "types/containers/fast_vector.h"
#include "types/Index.h"

#define GET_MIN(BOUNDS, AXIS) BOUNDS[AXIS]
#define GET_MAX(BOUNDS, AXIS) BOUNDS[AXES_COUNT+AXIS]
#define SEG_TPL template<typename SAPDomain>
#define SEG_TYPE SAPSegment<SAPDomain>

namespace grynca {

    SEG_TPL
    inline SEG_TYPE::SAPSegment(Manager& mgr, Segment* parent)
     : manager_(&mgr),
       parent_(parent),
       split_axis_(SAP::InvalidAxis),
       split_value_(0.0)
    {
        children_[0] = children_[1] = NULL;
    }

    SEG_TPL
    inline SEG_TYPE::~SAPSegment() {
        if (children_[0])
            delete children_[0];
        if (children_[1])
            delete children_[1];
    }

    SEG_TPL
    inline void SEG_TYPE::addBox(Box& box, Index box_id) {
        for (u32 a=0; a<AXES_COUNT; ++a) {
            insertSingleAxis_(box, box_id.getIndex(), a);
            findOverlapsOnAxis_(box, a);
            // update longest side if neccessary
            f32 side_len = box.getMaxValue(a) - box.getMinValue(a);
            if (side_len > longest_sides_[a].length) {
                longest_sides_[a].length = side_len;
                longest_sides_[a].box_id = box_id.getIndex();
            }
        }
        if (getBoxesCount() > SAP::MAX_BOXES_IN_SEGMENT) {
            split_();
        }
    }

    SEG_TPL
    inline bool SEG_TYPE::moveBox(Box& box, Index box_id, SAP::MinMax* old_min_max_ids, f32* move_vec, typename Manager::DeferredAfterUpdate& dau) {
        PROFILE_BLOCK("SAPSegment::moveBox");

        bool oos = false;
        for (u32 a=0; a<AXES_COUNT; ++a) {
            moveMinMaxPoints_(box, box_id, old_min_max_ids, move_vec, a, borders_[a].low, borders_[a].high, dau.crossing_, oos);
        }

        if (oos) {
            removeBoxInner_(box, box_id, old_min_max_ids);
            if (getBoxesCount() < SAP::MIN_BOXES_IN_SEGMENT && parent_) {
                dau.merges_.push_back(this);
            }
            return true;
        }

        return false;
    }

    SEG_TPL
    inline bool SEG_TYPE::updateBox(Box& box, Index box_id, SAP::MinMax* old_min_max_ids, typename Manager::DeferredAfterUpdate& dau) {
        bool oos = false;
        for (u32 a=0; a<AXES_COUNT; ++a) {
            updateMinMaxPoints_(box, box_id, old_min_max_ids, a, borders_[a].low, borders_[a].high, dau.crossing_);

            f32 min_val = box.getMinValue(a);
            f32 max_val = box.getMaxValue(a);
            if (min_val > borders_[a].high || max_val < borders_[a].low) {
                oos = true;
            }
            else {
                f32 side_len = max_val - min_val;
                if (side_len > longest_sides_[a].length) {
                    longest_sides_[a].length = side_len;
                    longest_sides_[a].box_id = box_id.getIndex();
                }
                else if (box_id.getIndex() == longest_sides_[a].box_id) {
                    longest_sides_[a] = findLongestSide_(u32(a));
                }
            }
        }

        if (oos) {
            removeBoxInner_(box, box_id, old_min_max_ids);
            if (getBoxesCount() < SAP::MIN_BOXES_IN_SEGMENT && parent_) {
                dau.merges_.push_back(this);
            }
        }
        return oos;
    }

    SEG_TPL
    inline void SEG_TYPE::removeBox(Box& box, Index box_id, SAP::MinMax* min_max_ids) {

        removeBoxInner_(box, box_id, min_max_ids);

        if (getBoxesCount() < SAP::MIN_BOXES_IN_SEGMENT && parent_) {
            parent_->merge_(this);
        }
    }

    SEG_TPL
    inline typename SEG_TYPE::Segment* SEG_TYPE::getChild(u32 id) {
        ASSERT(id < 2);
        return children_[id];
    }

    SEG_TPL
    inline typename SEG_TYPE::Segment* SEG_TYPE::getPrevNeighbor(u32 axis) {
        if (!parent_)
            return NULL;
        if (parent_->getSplitAxis() == axis && parent_->getChild(1) == this)
            return parent_->getChild(0);

        return parent_->getPrevNeighbor(axis);
    }

    SEG_TPL
    inline typename SEG_TYPE::Segment* SEG_TYPE::getNextNeighbor(u32 axis) {
        if (!parent_)
            return NULL;
        if (parent_->getSplitAxis() == axis && parent_->getChild(0) == this)
            return parent_->getChild(1);

        return parent_->getNextNeighbor(axis);
    }

    SEG_TPL
    inline typename SEG_TYPE::Segment* SEG_TYPE::getSplitNeighbor() {
        if (!parent_)
            return NULL;
        if (parent_->getChild(0) == this)
            return parent_->getChild(1);
        return parent_->getChild(0);
    }

    SEG_TPL
    inline typename SEG_TYPE::Segment* SEG_TYPE::getSplitNeighbor(u32& child_id_out) {
        if (!parent_)
            return NULL;
        if (parent_->getChild(0) == this) {
            child_id_out = 1;
            return parent_->getChild(1);
        }
        child_id_out = 0;
        return parent_->getChild(0);
    }

    SEG_TPL
    inline bool SEG_TYPE::getLowBorder(u32 axis, f32& lb_out) {
        lb_out = borders_[axis].low;
        return borders_[axis].has_low;
    }

    SEG_TPL
    inline bool SEG_TYPE::getHighBorder(u32 axis, f32& hb_out) {
        hb_out = borders_[axis].high;
        return borders_[axis].has_high;
    }

    SEG_TPL
    inline bool SEG_TYPE::isSplit() {
        return split_axis_ != SAP::InvalidAxis;
    }

    SEG_TPL
    inline u32 SEG_TYPE::getBoxesCount() {
        return points_[0].size()/2;
    }

    SEG_TPL
    inline void SEG_TYPE::getCrossedBoxes(fast_vector<u32>& crossed_out) {
        ASSERT(isSplit());
        if (children_[1]->points_[split_axis_].empty())
            return;

        u32 i = 0;
        while (children_[1]->points_[split_axis_][i].getValue() < split_value_) {
            crossed_out.push_back(children_[1]->points_[split_axis_][i].getBoxId());
            ++i;
        }
    }

    SEG_TPL
    inline std::string SEG_TYPE::getDebugName() {
#ifdef DEBUG_BUILD
        return debug_name_;
#endif
        return std::string();
    }

    SEG_TPL
    template <typename AddedCb>
    inline void SEG_TYPE::addBoxTree_(const f32* bounds, const AddedCb& cb) {
        if (!isSplit()) {
            cb(this);
        }
        else if (GET_MAX(bounds, getSplitAxis()) < getSplitValue()) {
            // put to first
            getChild(0)->addBoxTree_(bounds, cb);
        }
        else if (GET_MIN(bounds, getSplitAxis()) > getSplitValue()) {
            // put to second
            getChild(1)->addBoxTree_(bounds, cb);
        }
        else {
            // put to both
            getChild(0)->addBoxTree_(bounds, cb);
            getChild(1)->addBoxTree_(bounds, cb);
        }
    }


    SEG_TPL
    inline u32 SEG_TYPE::bisectInsertFind_(SAP::Points& points, f32 val, u32 from, u32 to) {
        u32 half = from + (to-from)/2;
        if (val < points[half].getValue()) {
            if (half == from)
                return from;
            else
                return bisectInsertFind_(points, val, from, half-1);
        }
        else {
            if (half == to)
                return to+1;
            else
                return bisectInsertFind_(points, val, half+1, to);
        }
    }

    SEG_TPL
    inline void SEG_TYPE::findOverlapsOnAxis_(Box& box, u32 axis) {
        SAP::Points& ps = points_[axis];
        u32 min_id = box.getMinId(this, axis);
        u32 max_id = box.getMaxId(this, axis);

        u32 from = getScanStartId_(min_id, axis);

        for (u32 i=from; i< min_id; ++i) {
            u32 bid = ps[i].getBoxId();
            if (!ps[i].getIsMax()) {
                ASSERT(ps[min_id].getBoxId() != bid);
                manager_->overlaps_.possibly_added_.push_back(bid);
            }
        }

        for (u32 i= min_id +1; i<max_id; ++i) {
            u32 bid = ps[i].getBoxId();
            if (!ps[i].getIsMax()) {
                ASSERT(ps[min_id].getBoxId() != bid);
                manager_->overlaps_.possibly_added_.push_back(bid);
            }
        }
    }

    SEG_TPL
    inline u32 SEG_TYPE::getScanStartId_(u32 min_id, u32 axis) {
        SAP::Points& ps = points_[axis];
        f32 min_val = ps[min_id].getValue();
        // find from where we must scan for overlaps
        f32 longest_len = longest_sides_[axis].length;
        if (longest_len != 0.0f) {
            for (i32 from= min_id -1; from>0; --from) {
                f32 len = min_val - ps[from].getValue();
                if (len > longest_len)
                    return u32(from);
            }
        }
        return 0;
    }

    SEG_TPL
    inline void SEG_TYPE::insertSingleAxis_(Box& new_box, u32 new_box_inner_id, u32 axis) {
        f32 min_val = new_box.getMinValue(axis);
        f32 max_val = new_box.getMaxValue(axis);

        SAP::Points& ps = points_[axis];

        u32 new_min_id, new_max_id;
        u32 points_count = u32(ps.size());
        if (!ps.empty()) {
            new_min_id = bisectInsertFind_(ps, min_val, 0, points_count-1);
            if (new_min_id == points_count)
                new_max_id = new_min_id+1;
            else
                new_max_id = bisectInsertFind_(ps, max_val, new_min_id, points_count-1) +1;
        }
        else {
            new_min_id = 0;
            new_max_id = 1;
        }

        ps.emplace(ps.begin()+new_min_id, new_box_inner_id, false, min_val);
        ps.emplace(ps.begin()+new_max_id, new_box_inner_id, true, max_val);

        new_box.setMinMaxId(this, axis, new_min_id, new_max_id);

        // fix ids for other boxes
        for (u32 i=new_min_id+1; i<new_max_id; ++i) {
            Box& b = manager_->boxes_.accItemWithInnerIndex(ps[i].getBoxId());
            b.setEndPointId(this, axis, i, ps[i].getIsMax());
        }
        for (u32 i=new_max_id+1; i<ps.size(); ++i) {
            Box& b = manager_->boxes_.accItemWithInnerIndex(ps[i].getBoxId());
            b.setEndPointId(this, axis, i, ps[i].getIsMax());
        }
    }

    SEG_TPL
    inline SAP::LongestSide SEG_TYPE::findLongestSide_(u32 axis) {
        f32 longest_side = 0.0f;
        u32 longest_id = SAP::InvalidAxis;
        SAP::Points& ps = points_[axis];
        for (u32 i=0; i<ps.size(); ++i) {
            if (ps[i].getIsMax()) {
                u32 box_inner_id = ps[i].getBoxId();
                Box& b = manager_->boxes_.accItemWithInnerIndex(box_inner_id);
                f32 side = ps[i].getValue() - b.getMinValue(axis);
                if (side > longest_side) {
                    longest_side = side;
                    longest_id = box_inner_id;
                }
            }
        }
        return SAP::LongestSide{longest_id, longest_side};
    }

    SEG_TPL
    inline void SEG_TYPE::addBoxInner_(Box& box, u32 box_inner_id) {
        for (u32 a=0; a<AXES_COUNT; ++a) {
            insertSingleAxis_(box, box_inner_id, a);
            // update longest side if neccessary
            f32 side_len = box.getMaxValue(a) - box.getMinValue(a);
            if (side_len > longest_sides_[a].length) {
                longest_sides_[a].length = side_len;
                longest_sides_[a].box_id = box_inner_id;
            }
        }
        // TODO: dont split during merge - was buggy
//        if (getBoxesCount() > SAP::MAX_BOXES_IN_SEGMENT) {
//            split_();
//        }
    }

    SEG_TPL
    inline void SEG_TYPE::removeBoxInner_(Box& box, Index box_id, SAP::MinMax* min_max_ids) {
        for (u32 a=0; a<AXES_COUNT; ++a) {
            u32 min_id = min_max_ids[a].v[0];
            u32 max_id = min_max_ids[a].v[1];
            SAP::Points& ps = points_[a];

            // fix ids for other boxes
            for (u32 i=min_id+1; i<max_id; ++i) {
                u32 b2_inner_id = ps[i].getBoxId();
                Box& b2 = manager_->boxes_.accItemWithInnerIndex(b2_inner_id);
                b2.setEndPointId(this, a, i-1, ps[i].getIsMax());
            }
            for (u32 i=max_id+1; i<ps.size(); ++i) {
                u32 b2_inner_id = ps[i].getBoxId();
                Box& b2 = manager_->boxes_.accItemWithInnerIndex(b2_inner_id);
                b2.setEndPointId(this, a, i-2, ps[i].getIsMax());
            }

            ps.erase(ps.begin()+max_id);
            ps.erase(ps.begin()+min_id);

            if (longest_sides_[a].box_id == box_id.getIndex()) {
                longest_sides_[a] = findLongestSide_(a);
            }
        }

        box.removeOccurence(this);
    }

    SEG_TPL
    inline u32 SEG_TYPE::moveMinRight_(u32 point_id, f32 new_value, u32 axis) {
        SAP::Points& points = points_[axis];

        u32 from_id = point_id;
        for (++point_id; point_id <points.size(); ++point_id) {
            SAP::EndPoint &p = points[point_id];
            if (new_value <= p.getValue())
                break;
        }

        --point_id;
        i32 count = point_id-from_id;

        if (count>0) {
#ifdef DEBUG_BUILD
            u32 b_inner_id = points[from_id].getBoxId();
#endif
            SAP::EndPoint tmp_point = points[from_id];
            memmove(&points[from_id], &points[from_id+1], sizeof(SAP::EndPoint)*count);
            for (; from_id<point_id; ++from_id) {
                SAP::EndPoint& p = points[from_id];
                u32 b2_inner_id = p.getBoxId();
                Box& b2 = manager_->boxes_.accItemWithInnerIndex(b2_inner_id);
                b2.setEndPointId(this, axis, from_id, p.getIsMax());

                if (p.getIsMax()) {
                    ASSERT(b_inner_id != b2_inner_id);
                    manager_->overlaps_.removed_.push_back(b2_inner_id);
                }
            }
            points[point_id] = tmp_point;
        }
        points[point_id].setValue(new_value);
        return u32(point_id);
    }

    SEG_TPL
    inline u32 SEG_TYPE::moveMaxRight_(u32 point_id, f32 new_value, u32 axis) {
        SAP::Points& points = points_[axis];

        u32 from_id = point_id;
        for (++point_id; point_id <points.size(); ++point_id) {
            SAP::EndPoint &p = points[point_id];
            if (new_value <= p.getValue())
                break;
        }

        --point_id;
        i32 count = point_id-from_id;

        if (count>0) {
#ifdef DEBUG_BUILD
            u32 b_inner_id = points[from_id].getBoxId();
#endif
            SAP::EndPoint tmp_point = points[from_id];
            memmove(&points[from_id], &points[from_id+1], sizeof(SAP::EndPoint)*count);
            for (; from_id<point_id; ++from_id) {
                SAP::EndPoint& p = points[from_id];
                u32 b2_inner_id = p.getBoxId();
                Box& b2 = manager_->boxes_.accItemWithInnerIndex(b2_inner_id);
                b2.setEndPointId(this, axis, from_id, p.getIsMax());

                if (!p.getIsMax()) {
                    ASSERT(b_inner_id != b2_inner_id);
                    manager_->overlaps_.possibly_added_.push_back(b2_inner_id);
                }
            }
            points[point_id] = tmp_point;
        }
        points[point_id].setValue(new_value);
        return u32(point_id);
    }

    SEG_TPL
    inline u32 SEG_TYPE::moveMinLeft_(i32 point_id, f32 new_value, u32 axis) {
        SAP::Points& points = points_[axis];

        i32 from_id = point_id;
        for (--point_id; point_id >=0; --point_id) {
            SAP::EndPoint& p = points[point_id];
            if (new_value >= p.getValue())
                break;
        }

        ++point_id;
        i32 count = from_id-point_id;
        if (count>0) {
#ifdef DEBUG_BUILD
            u32 b_inner_id = points[from_id].getBoxId();
#endif
            SAP::EndPoint tmp_point = points[from_id];
            memmove(&points[point_id+1], &points[point_id], sizeof(SAP::EndPoint)*count);
            for (; from_id>point_id; --from_id) {
                SAP::EndPoint& p = points[from_id];
                u32 b2_inner_id = p.getBoxId();
                Box& b2 = manager_->boxes_.accItemWithInnerIndex(b2_inner_id);
                b2.setEndPointId(this, axis, (u32)from_id, p.getIsMax());

                if (p.getIsMax()) {
                    ASSERT(b_inner_id != b2_inner_id);
                    manager_->overlaps_.possibly_added_.push_back(b2_inner_id);
                }
            }
            points[from_id] = tmp_point;
        }
        points[from_id].setValue(new_value);
        return u32(point_id);
    }

    SEG_TPL
    inline u32 SEG_TYPE::moveMaxLeft_(i32 point_id, f32 new_value, u32 axis) {
        SAP::Points& points = points_[axis];

        i32 from_id = point_id;
        for (--point_id; point_id >=0; --point_id) {
            SAP::EndPoint& p = points[point_id];
            if (new_value >= p.getValue())
                break;
        }

        ++point_id;
        i32 count = from_id-point_id;
        if (count>0) {
#ifdef DEBUG_BUILD
            u32 b_inner_id = points[from_id].getBoxId();
#endif
            SAP::EndPoint tmp_point = points[from_id];
            memmove(&points[point_id+1], &points[point_id], sizeof(SAP::EndPoint)*count);
            for (; from_id>point_id; --from_id) {
                SAP::EndPoint& p = points[from_id];
                u32 b2_inner_id = p.getBoxId();
                Box& b2 = manager_->boxes_.accItemWithInnerIndex(b2_inner_id);
                b2.setEndPointId(this, axis, (u32)from_id, p.getIsMax());

                if (!p.getIsMax()) {
                    ASSERT(b_inner_id != b2_inner_id);
                    manager_->overlaps_.removed_.push_back(b2_inner_id);
                }
            }
            points[from_id] = tmp_point;
        }
        points[from_id].setValue(new_value);
        return u32(point_id);
    }

    SEG_TPL
    inline void SEG_TYPE::moveMinMaxPoints_(Box& box, Index box_id, SAP::MinMax* old_min_max_ids, f32* move_vec, u32 axis, f32 low, f32 high, bool& crossing_out, bool& oos_out) {
        f32 new_min = box.getMinValue(axis);
        f32 new_max = box.getMaxValue(axis);

        u32 min_id = old_min_max_ids[axis].v[0];
        SAP::EndPoint& old_min = points_[axis][min_id];
        u32 max_id = old_min_max_ids[axis].v[1];
        SAP::EndPoint& old_max = points_[axis][max_id];

        u32 new_min_id;
        u32 new_max_id;

        if (move_vec[axis] > 0) {
            if (new_max > high) {
                if (old_max.getValue()<=high) {
                    crossing_out = true;
                }
                if (new_min > high) {
                    oos_out = true;
                }
            }
            new_max_id = moveMaxRight_(max_id, new_max, axis);
            new_min_id = moveMinRight_(min_id, new_min, axis);
        }
        else {
            if (new_min < low) {
                if (old_min.getValue()>=low) {
                    crossing_out = true;
                }
                if (new_max < low) {
                    oos_out = true;
                }
            }
            new_min_id = moveMinLeft_(min_id, new_min, axis);
            new_max_id = moveMaxLeft_(max_id, new_max, axis);
        }

        if (min_id != new_min_id || max_id != new_max_id) {
            box.setMinMaxId(this, axis, new_min_id, new_max_id);
        }
    }

    SEG_TPL
    inline void SEG_TYPE::updateMinMaxPoints_(Box& box, Index box_id, SAP::MinMax* old_min_max_ids, u32 axis, f32 low, f32 high, bool& crossing_out) {
        f32 new_min = box.getMinValue(axis);
        f32 new_max = box.getMaxValue(axis);

        u32 min_id = old_min_max_ids[axis].v[0];
        SAP::EndPoint& old_min = points_[axis][min_id];
        u32 max_id = old_min_max_ids[axis].v[1];
        SAP::EndPoint& old_max = points_[axis][max_id];

        u32 new_min_id;
        u32 new_max_id;
        if (old_min.getValue() > new_min) {
            if (new_min < low && old_min.getValue()>=low ) {
                crossing_out = true;
            }
            new_min_id = moveMinLeft_(min_id, new_min, axis);

            // max
            if (old_max.getValue() < new_max) {
                if (new_max > high && old_max.getValue()<=high ) {
                    crossing_out = true;
                }
                new_max_id = moveMaxRight_(max_id, new_max, axis);
            }
            else {
                new_max_id = moveMaxLeft_(max_id, new_max, axis);
            }
        }
        else {
            // max
            if (old_max.getValue() < new_max) {
                if (new_max > high && old_max.getValue()<=high ) {
                    crossing_out = true;
                }
                new_max_id = moveMaxRight_(max_id, new_max, axis);
            }
            else {
                new_max_id = moveMaxLeft_(max_id, new_max, axis);
            }

            new_min_id = moveMinRight_(min_id, new_min, axis);
        }

        if (min_id != new_min_id || max_id != new_max_id) {
            box.setMinMaxId(this, axis, new_min_id, new_max_id);
        }
    };

    SEG_TPL
    inline void SEG_TYPE::split_() {
        ASSERT(!isSplit());

        u32 boxes_count = getBoxesCount();

        enum {
            F_GO_TO_FIRST = 1,
            F_GO_TO_SECOND = 2,
            F_GO_TO_BOTH = F_GO_TO_FIRST | F_GO_TO_SECOND
        };

        struct {
            f32 val;
            u32 axis;
            u32 i;

            u32 split1_cnt;
            u32 crossed_cnt;
            fast_vector<u8> flags[AXES_COUNT];
        } best;

        best.val = std::numeric_limits<f32>::max();
        best.i = InvalidId();
        best.axis = 0;

        u32 largest_r = 0;
        f32 ranges[AXES_COUNT];

        for (u32 a = 0; a < AXES_COUNT; ++a) {
            best.flags[a].resize(boxes_count*2, u8(F_GO_TO_SECOND));

            f32 low, high;
            if (!getLowBorder(a, low)) {
                low = findLowestPointRec_(a);
            }
            if (!getHighBorder(a, high)) {
                high = findHighestPointRec_(a);
            }
            ranges[a] = high - low;
            if (ranges[a] > ranges[largest_r])
                largest_r = a;
        }
        f32 range_coeffs[AXES_COUNT];
        for (u32 a = 0; a < AXES_COUNT; ++a) {
            range_coeffs[a] = ranges[largest_r]/ranges[a];
        }

        u8 parent_split = parent_?parent_->getSplitAxis(): SAP::InvalidAxis;
        // find best split position
        for (u32 tested_a=0; tested_a<AXES_COUNT; ++tested_a) {
            if (best.val == 0 && best.axis != parent_split)
                // value cant get better, dont check other axes
                break;

            SAP::Points& points = points_[tested_a];
            u32 split1_cnt = 0;
            u32 crossed_cnt = 0;

            for (u32 i=1; i<points.size()-1; ++i) {
                SAP::EndPoint& p = points[i];

                if (!p.getIsMax()) {
                    ++split1_cnt;
                    ++crossed_cnt;
                }
                else {
                    --crossed_cnt;
                }

                int split2_cnt = boxes_count-split1_cnt+crossed_cnt;
                int splits_dif = split1_cnt - split2_cnt;
                f32 val = abs(splits_dif) + crossed_cnt*SAP::CROSSED_SPLIT_PENALTY_COEF;
                val *= range_coeffs[tested_a];

                bool better_split = (val < best.val)
                                    || (val == best.val && (i32)tested_a != parent_split);       // prioritize different axis than parent's

                if (better_split) {
                    if (best.axis != tested_a) {
                        // clear flags
                        for (u32 a = 0; a < AXES_COUNT; ++a) {
                            best.flags[a].assign(boxes_count*2, u8(F_GO_TO_SECOND));
                        }
                        best.i = InvalidId();
                    }

                    for (u32 j=best.i+1; j<=i; ++j) {
                        // update flags to new best state
                        SAP::EndPoint& p2 = points[j];
                        Box& b = manager_->boxes_.accItemWithInnerIndex(p2.getBoxId());
                        if (!p2.getIsMax()) {
                            for (u32 a=0; a<AXES_COUNT; ++a) {
                                best.flags[a][b.getMinId(this, a)] = u8(F_GO_TO_BOTH);
                                best.flags[a][b.getMaxId(this, a)] = u8(F_GO_TO_BOTH);
                            }
                        }
                        else {
                            for (u32 a=0; a<AXES_COUNT; ++a) {
                                best.flags[a][b.getMinId(this, a)] = u8(F_GO_TO_FIRST);
                                best.flags[a][b.getMaxId(this, a)] = u8(F_GO_TO_FIRST);
                            }
                        }
                    }

                    best.i = i;
                    best.axis = tested_a;
                    best.val = val;
                    best.split1_cnt = split1_cnt;
                    best.crossed_cnt = crossed_cnt;
                }
                else if (splits_dif > best.val )
                    // no need to test other positions, val would only get higher
                    break;
            }
        }

        u32 split1_boxes_cnt = best.split1_cnt;
        u32 split2_boxes_cnt = boxes_count - best.split1_cnt +best.crossed_cnt;

        f32 split_val = (points_[best.axis][best.i].getValue() + points_[best.axis][best.i+1].getValue())/2;

        dout("splitting: VAL=" << split_val << ", F=" << best.split1_cnt << ", S=" << (boxes_count - best.split1_cnt +best.crossed_cnt) << ", C=" << best.crossed_cnt << std::endl);
        if (split1_boxes_cnt>SAP::MAX_BOXES_IN_SEGMENT || split2_boxes_cnt>SAP::MAX_BOXES_IN_SEGMENT) {
            dout(" skipped - would not reduce boxes count." << std::endl);
            return;
        }

        f32 upper_limit;
        if (getHighBorder((u32)best.axis, upper_limit) && split_val > upper_limit) {
            //split point is inside some crossed box which is out of this segment (dont split there)
            dout(" skipped - split would be outside segment borders" << std::endl);
            return;
        }

        split_value_ = split_val;
        split_axis_ = (u8)best.axis;

        children_[0] = new SEG_TYPE(*manager_, this);
        children_[1] = new SEG_TYPE(*manager_, this);
        children_[0]->setDebugName_();
        children_[1]->setDebugName_();
        children_[0]->calcBorders_();
        children_[1]->calcBorders_();

        for (u32 i=0; i<points_[0].size(); ++i) {
            // remove boxes from splitted segment
            if (!points_[0][i].getIsMax()) {
                Box& b = manager_->boxes_.accItemWithInnerIndex(points_[0][i].getBoxId());
                b.removeOccurence(this);
            }
        }

        for (u32 a=0; a<AXES_COUNT; ++a) {
            children_[0]->points_[a].reserve(split1_boxes_cnt*2);
            children_[1]->points_[a].reserve(split2_boxes_cnt*2);

            for (u32 i=0; i<best.flags[a].size(); ++i) {
                if (best.flags[a][i] & F_GO_TO_FIRST) {
                    pointToChild_(children_[0], a, i);
                }
                if (best.flags[a][i] & F_GO_TO_SECOND) {
                    pointToChild_(children_[1], a, i);
                }
            }
            points_[a].clear();
            longest_sides_[a].length = 0.0f;
        }
    }

    SEG_TPL
    inline void SEG_TYPE::merge_(Segment* removed_child) {
        ASSERT(!removed_child->isSplit());

#ifdef DEBUG_BUILD
        std::cout << "merging " << debug_name_ << std::endl;
#endif

        Segment* other_child = (removed_child==children_[0])?children_[1]:children_[0];

        // move all relevant from other child to this
        split_axis_ = other_child->split_axis_;
        split_value_ = other_child->split_value_;

        children_[0] = other_child->children_[0];
        children_[1] = other_child->children_[1];

        if (other_child->isSplit()) {
            children_[0]->parent_ = children_[1]->parent_ = this;
            other_child->children_[0] = other_child->children_[1] = NULL;

#ifdef DEBUG_BUILD
            u32 pos = debug_name_.size();
            fast_vector<Segment*> segs = {children_[0], children_[1]};
            while (!segs.empty()) {
                Segment* s = segs.back();
                segs.pop_back();
                s->debug_name_.erase(s->debug_name_.begin()+pos);
                if (s->isSplit()) {
                    segs.push_back(s->children_[0]);
                    segs.push_back(s->children_[1]);
                }
            }
#endif
        }
        else {
            for (u32 a=0; a<AXES_COUNT; ++a) {
                points_[a] = std::move(other_child->points_[a]);
                longest_sides_[a] = other_child->longest_sides_[a];
            }

            for (u32 i=0; i<points_[0].size(); ++i) {
                SAP::EndPoint& p = points_[0][i];
                if (!p.getIsMax()) {
                    Box& b = manager_->boxes_.accItemWithInnerIndex(p.getBoxId());
                    b.changeOccurence(other_child, this);
                }
            }
        }

        delete other_child;

        // move boxes from removed child
        for (u32 i=0; i<removed_child->points_[0].size(); ++i) {
            SAP::EndPoint& p = removed_child->points_[0][i];
            if (!p.getIsMax()) {
                u32 box_inner_id = p.getBoxId();
                Box& b = manager_->boxes_.accItemWithInnerIndex(box_inner_id);
                b.removeOccurence(removed_child);

                fast_vector<Segment*> segs{this};
                while (!segs.empty()) {
                    Segment *s = segs.back();
                    segs.pop_back();

                    if (!s->isSplit()) {
                        if (b.findOccurence(s) == InvalidId()) {
                            s->addBoxInner_(b, box_inner_id);
                        }
                    }
                    else if (b.getMaxValue(s->getSplitAxis()) < s->getSplitValue()) {
                        // put to first
                        segs.push_back(s->getChild(0));
                    }
                    else if (b.getMinValue(s->getSplitAxis()) > s->getSplitValue()) {
                        // put to second
                        segs.push_back(s->getChild(1));
                    }
                    else {
                        // put to both
                        segs.push_back(s->getChild(0));
                        segs.push_back(s->getChild(1));
                    }
                }
            }
        }

        delete removed_child;

        // recalc borders for subtree
        fast_vector<Segment*> segs{this};
        while (!segs.empty()) {
            Segment *s = segs.back();
            segs.pop_back();

            s->calcBorders_();
            if (s->isSplit()) {
                segs.push_back(s->getChild(0));
                segs.push_back(s->getChild(1));
            }
        }
    }

    SEG_TPL
    inline void SEG_TYPE::pointToChild_(Segment* child, u32 a, u32 point_id) {
        SAP::EndPoint& p = points_[a][point_id];
        Box& b = manager_->boxes_.accItemWithInnerIndex(p.getBoxId());

        u32 child_point_id = child->points_[a].size();
        child->points_[a].push_back(p);
        b.setEndPointId(child, a, child_point_id, p.getIsMax());
        if (p.getIsMax()) {
            f32 side_len = p.getValue() - b.getMinValue(a);
            if (child->longest_sides_[a].length < side_len) {
                child->longest_sides_[a].box_id = p.getBoxId();
                child->longest_sides_[a].length = side_len;
            }
        }
    }

    SEG_TPL
    inline void SEG_TYPE::setDebugName_() {
#ifdef DEBUG_BUILD
        if (!parent_)
            debug_name_ = "R";
        else
            debug_name_ = parent_->debug_name_ + ((parent_->children_[0] == this)?"0":"1");
#endif
    }

    SEG_TPL
    inline void SEG_TYPE::calcBorders_() {
        for (u32 a = 0; a<AXES_COUNT; ++a) {
            borders_[a].high = std::numeric_limits<f32>::max();
            borders_[a].low = -std::numeric_limits<f32>::max();
            borders_[a].has_high = calcHighBorder_(a, borders_[a].high);
            borders_[a].has_low = calcLowBorder_(a, borders_[a].low);
        }
    };

    SEG_TPL
    inline bool SEG_TYPE::calcLowBorder_(u32 axis, f32& val) {
        Segment* n = getPrevNeighbor(axis);
        if (n) {
            val = n->getParent()->getSplitValue();
        }

        return (bool)n;
    }

    SEG_TPL
    inline bool SEG_TYPE::calcHighBorder_(u32 axis, f32& val) {
        Segment* n = getNextNeighbor(axis);
        if (n) {
            val = n->getParent()->getSplitValue();
        }

        return (bool)n;
    }

    SEG_TPL
    inline f32 SEG_TYPE::findLowestPointRec_(u32 a) {
        if (isSplit()) {
            if ((u32)getSplitAxis() == a) {
                return getChild(0)->findLowestPointRec_(a);
            }
            else
                return std::min(getChild(0)->findLowestPointRec_(a), getChild(1)->findLowestPointRec_(a));
        }
        // else
        return points_[a][0].getValue();
    }

    SEG_TPL
    inline f32 SEG_TYPE::findHighestPointRec_(u32 a) {
        if (isSplit()) {
            if ((u32)getSplitAxis() == a) {
                return getChild(1)->findHighestPointRec_(a);
            }
            else
                return std::max(getChild(0)->findHighestPointRec_(a), getChild(1)->findHighestPointRec_(a));
        }
        // else
        return points_[a].back().getValue();
    }

}

#undef GET_MIN
#undef GET_MAX
#undef SEG_TPL
#undef SEG_TYPE