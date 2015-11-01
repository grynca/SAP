#include "SAPManager.h"
#include <algorithm>
#include <cassert>

namespace grynca {

    template <typename ClientData, uint32_t AXES_COUNT>
    inline SAPManager<ClientData, AXES_COUNT>::SAPManager() {
        for (uint32_t i=0; i<MAX_COLLISION_GROUPS; ++i) {
            collision_groups_matrix_[i].set();
        }
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    inline GroupFlags & SAPManager<ClientData, AXES_COUNT>::getCollisionFlags(uint32_t group_id) {
        return collision_groups_matrix_[group_id];
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    void SAPManager<ClientData, AXES_COUNT>::setCollisionFlags(uint32_t group_id, bool value, const fast_vector<uint32_t>& group_ids) {
        collision_flags_changed_[group_id] = true;
        for (uint32_t i=0; i<group_ids.size(); ++i) {
            uint32_t gid = group_ids[i];
            collision_groups_matrix_[group_id][gid] = value;
        }
    };


    template <typename ClientData, uint32_t AXES_COUNT>
    inline uint32_t SAPManager<ClientData, AXES_COUNT>::addBox(Extent* extents, uint32_t coll_group_id, const ClientData& client_data) {
#ifndef NDEBUG
        for (uint32_t a=0; a<AXES_COUNT; ++a) {
            assert(extents[a].min < extents[a].max);
        }
#endif

        uint32_t new_box_id = boxes_.add();
        Box& new_box = boxes_.get(new_box_id);
        new_box.setClientData(client_data);
        new_box.setCollisionGroup(coll_group_id);

        clearInterestingFlags();

        for (uint32_t a=0; a<AXES_COUNT; ++a) {
            insertSingleAxis(new_box_id, extents[a].min, extents[a].max, uint32_t(a));
            // update longest side if neccessary
            float side_len = extents[a].max - extents[a].min;
            longest_sides_[a].length = side_len;
            longest_sides_[a].box_id = new_box_id;
        }

        for (uint32_t a=0; a<AXES_COUNT; ++a) {
            uint32_t group1 = new_box.getCollisionGroup();
            for (uint32_t i=0; i<interesting_ids_.size(); ++i) {
                uint32_t b2_id = interesting_ids_[i];
                Box& b2 = boxes_.get(b2_id);
                uint32_t group2 = b2.getCollisionGroup();
                bool b1_cares = collision_groups_matrix_[group1][group2];
                bool b2_cares = collision_groups_matrix_[group2][group1];
                if (b1_cares || b2_cares) {
                    if (boxesOverlap_(new_box, b2)) {
                        if (b1_cares)
                            new_box.addOverlap(b2_id);
                        if (b2_cares)
                            b2.addOverlap(new_box_id);
                    }
                }
            }
        }

        return new_box_id;
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    inline void SAPManager<ClientData, AXES_COUNT>::updateBox(uint32_t box_id, Extent* extents) {
#ifndef NDEBUG
        for (uint32_t a=0; a<AXES_COUNT; ++a) {
            assert(extents[a].min < extents[a].max);
        }
#endif
        clearInterestingFlags();
        for (uint32_t a=0; a<AXES_COUNT; ++a) {
            updateMinMaxPoints_(box_id, extents[a].min, extents[a].max, uint32_t(a));
            float side_len = extents[a].max - extents[a].min;
            if (side_len > longest_sides_[a].length) {
                longest_sides_[a].length = side_len;
                longest_sides_[a].box_id = box_id;
            }
            else if (box_id == longest_sides_[a].box_id) {
                longest_sides_[a] = findLongestSide(uint32_t(a));
            }
        }

        if (!interesting_ids_.empty()) {
            Box& b1 = boxes_.get(box_id);
            uint32_t group1 = b1.getCollisionGroup();
            for (uint32_t i=0; i<interesting_ids_.size(); ++i) {
                uint32_t b2_id = interesting_ids_[i];
                if (b2_id == box_id)
                    continue;
                Box& b2 = boxes_.get(b2_id);
                uint32_t group2 = b2.getCollisionGroup();
                bool b1_cares = collision_groups_matrix_[group1][group2];
                bool b2_cares = collision_groups_matrix_[group2][group1];
                if (b1_cares || b2_cares) {
                    if (boxesOverlap_(b1, b2)) {
                        if (b1_cares)
                            b1.addOverlap(b2_id);
                        if (b2_cares)
                            b2.addOverlap(box_id);
                    }
                    else {
                        if (b1_cares)
                            b1.removeOverlap(b2_id);
                        if (b2_cares)
                            b2.removeOverlap(box_id);
                    }
                }
            }
        }
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    inline void SAPManager<ClientData, AXES_COUNT>::removeBox(uint32_t box_id) {
        Box& b = boxes_.get(box_id);
        for (uint32_t i=0; i<b.getOverlaps().size(); ++i) {
            Box& b2 = boxes_.get(b.getOverlaps()[i]);
            b2.removeOverlap(box_id);
        }


        Extent extents[AXES_COUNT];
        for (uint32_t a=0; a<AXES_COUNT; ++a) {
            uint32_t axis = uint32_t(a);
            extents[a].min = b.getMinId(axis);
            extents[a].max = b.getMaxId(axis);
        }
        boxes_.remove(box_id);


        for (uint32_t a=0; a<AXES_COUNT; ++a) {
            uint32_t axis = uint32_t(a);

            // fix ids for other boxes
            for (uint32_t i=0; i<boxes_.size(); ++i) {
                Box& b2 = boxes_.getAtPos(i);
                uint32_t min_id = b2.getMinId(axis);
                if (min_id > extents[a].max) {
                    b2.setMinId(axis, min_id-2);
                }
                else if (min_id > extents[a].min) {
                    b2.setMinId(axis, min_id-1);
                }

                uint32_t max_id = b2.getMaxId(axis);
                if (max_id > extents[a].max) {
                    b2.setMaxId(axis, max_id-2);
                }
                else if (max_id > extents[a].min) {
                    b2.setMaxId(axis, max_id-1);
                }
            }

            points_[a].erase(points_[a].begin()+extents[a].max);
            points_[a].erase(points_[a].begin()+extents[a].min);
            if (longest_sides_[a].box_id == box_id) {
                longest_sides_[a] = findLongestSide(axis);
            }
        }
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    inline void SAPManager<ClientData, AXES_COUNT>::getBox(uint32_t box_id, Extent* extents) {
        Box& b = boxes_.get(box_id);
        for (uint32_t a=0; a<AXES_COUNT; ++a) {
            extents[a].min = points_[a][b.getMinId(uint32_t(a))].getValue();
            extents[a].max = points_[a][b.getMaxId(uint32_t(a))].getValue();
        }
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    inline uint32_t SAPManager<ClientData, AXES_COUNT>::getBoxesCount() {
        return boxes_.size();
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    inline void SAPManager<ClientData, AXES_COUNT>::updateCollisionFlags() {
        if (collision_flags_changed_.any()) {
            for (uint32_t i=0; i<boxes_.size(); ++i) {
                Box& box = boxes_.getAtPos(i);
                uint32_t group = box.getCollisionGroup();
                if (collision_flags_changed_[group]) {
                    box.getOverlaps().clear();
                    clearInterestingFlags();
                    uint32_t box_id = boxes_.getIndexForPos(i);
                    for (uint32_t a=0; i<AXES_COUNT; ++a) {
                        findOverlapsOnAxis(box_id, a);
                    }

                    for (uint32_t a=0; a<AXES_COUNT; ++a) {
                        for (uint32_t i=0; i<interesting_ids_.size(); ++i) {
                            uint32_t b2_id = interesting_ids_[i];
                            Box& b2 = boxes_.get(b2_id);
                            uint32_t group2 = b2.getCollisionGroup();
                            bool b1_cares = collision_groups_matrix_[group][group2];
                            if (b1_cares && boxesOverlap_(box, b2)) {
                                box.addOverlap(b2_id);
                            }
                        }
                    }
                }
            }
            collision_flags_changed_.reset();
        }
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    inline void SAPManager<ClientData, AXES_COUNT>::validate() {
        for (uint32_t i=0; i<boxes_.size(); ++i) {
            Box& b = boxes_.getAtPos(i);
            for (uint32_t a=0; a<AXES_COUNT; ++a) {
                uint32_t axis = uint32_t(a);
                uint32_t min_id = b.getMinId(axis);
                uint32_t max_id = b.getMaxId(axis);
                assert(min_id < max_id);
                assert(points_[a][min_id].getBoxId() == boxes_.getIndexForPos(i));
                assert(points_[a][max_id].getBoxId() == boxes_.getIndexForPos(i));
            }
        }
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    inline uint32_t SAPManager<ClientData, AXES_COUNT>::getOverlapsCount() {
        uint32_t count = 0;
        for (uint32_t i=0; i<boxes_.size(); ++i) {
            count += boxes_.getAtPos(i).getOverlaps().size();
        }
        return count;
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    inline ClientData& SAPManager<ClientData, AXES_COUNT>::getClientData(uint32_t box_id) {
        Box& b = boxes_.get(box_id);
        return b.getClientData();
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    inline const fast_vector<uint32_t>& SAPManager<ClientData, AXES_COUNT>::getOverlaps(uint32_t box_id) {
        Box &b = boxes_.get(box_id);

        return b.getOverlaps();
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    inline uint32_t SAPManager<ClientData, AXES_COUNT>::getGroupId(uint32_t box_id) {
        Box &b = boxes_.get(box_id);
        return b.getCollisionGroup();
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    inline void SAPManager<ClientData, AXES_COUNT>::updateMinMaxPoints_(uint32_t box_id, float min_value, float max_value, uint32_t axis) {
        Box& b = boxes_.get(box_id);
        // min
        uint32_t min_id = b.getMinId(axis);
        SAP_internal::EndPoint& min = points_[axis][min_id];
        uint32_t new_id;
        if (min.getValue() < min_value) {
            new_id = movePointRight_(min_id, min_value, axis);
        }
        else {
            new_id = movePointLeft_(min_id, min_value, axis);
        }
        b.setMinId(axis, new_id);
        // max
        uint32_t max_id = b.getMaxId(axis);
        SAP_internal::EndPoint& max = points_[axis][max_id];
        if (max.getValue() < max_value) {
            new_id = movePointRight_(max_id, max_value, axis);
        }
        else {
            new_id = movePointLeft_(max_id, max_value, axis);
        }
        b.setMaxId(axis, new_id);
    };

    template <typename ClientData, uint32_t AXES_COUNT>
    inline uint32_t SAPManager<ClientData, AXES_COUNT>::movePointRight_(int32_t point_id, float new_value, uint32_t axis) {
        SAP_internal::Points& points = points_[axis];

        int32_t from_id = point_id;
        for (++point_id; point_id <points.size(); ++point_id) {
            SAP_internal::EndPoint &p = points[point_id];
            if (new_value <= p.getValue())
                break;
        }

        --point_id;
        int32_t count = point_id-from_id;

        if (count>0) {
            SAP_internal::EndPoint tmp_point = points[from_id];
            uint32_t box_id = points[from_id].getBoxId();
            memmove(&points[from_id], &points[from_id+1], sizeof(SAP_internal::EndPoint)*count);
            for (; from_id<point_id; ++from_id) {
                SAP_internal::EndPoint& p = points[from_id];
                uint32_t b2_id = p.getBoxId();
                Box& b2 = boxes_.get(b2_id);
                if (p.getIsMin()) {
                    b2.setMinId(axis, from_id);
                }
                else {
                    b2.setMaxId(axis, from_id);
                }
                if (!interesting_ids_flags_[b2_id]) {
                    interesting_ids_flags_[b2_id] = true;
                    interesting_ids_.push_back(b2_id);
                }
            }
            points[point_id] = tmp_point;
        }
        points[point_id].setValue(new_value);
        return uint32_t(point_id);
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    inline uint32_t SAPManager<ClientData, AXES_COUNT>::movePointLeft_(int32_t point_id, float new_value, uint32_t axis) {
        SAP_internal::Points& points = points_[axis];

        int32_t from_id = point_id;
        for (--point_id; point_id >=0; --point_id) {
            SAP_internal::EndPoint& p = points[point_id];
            if (new_value >= p.getValue())
                break;
        }

        ++point_id;
        int32_t count = from_id-point_id;
        if (count>0) {
            SAP_internal::EndPoint tmp_point = points[from_id];
            memmove(&points[point_id+1], &points[point_id], sizeof(SAP_internal::EndPoint)*count);
            for (; from_id>point_id; --from_id) {
                SAP_internal::EndPoint& p = points[from_id];
                uint32_t b2_id = p.getBoxId();
                Box& b2 = boxes_.get(b2_id);
                if (p.getIsMin()) {
                    b2.setMinId(axis, from_id);
                }
                else {
                    b2.setMaxId(axis, from_id);
                }
                if (!interesting_ids_flags_[b2_id]) {
                    interesting_ids_flags_[b2_id] = true;
                    interesting_ids_.push_back(b2_id);
                }
            }
            points[from_id] = tmp_point;
        }
        points[from_id].setValue(new_value);
        return uint32_t(point_id);
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    inline void SAPManager<ClientData, AXES_COUNT>::insertSingleAxis(uint32_t new_box_id, float min_val, float max_val, uint32_t axis) {
        SAP_internal::Points& ps = points_[axis];

        uint32_t new_min_id, new_max_id;
        uint32_t points_count = uint32_t(ps.size());
        if (!ps.empty()) {
            new_min_id = bisectInsertFind(ps, min_val, 0, points_count-1);
            if (new_min_id == points_count)
                new_max_id = new_min_id+1;
            else
                new_max_id = bisectInsertFind(ps, max_val, new_min_id, points_count-1) +1;
        }
        else {
            new_min_id = 0;
            new_max_id = 1;
        }

        ps.emplace(ps.begin()+new_min_id, new_box_id, true, min_val);
        ps.emplace(ps.begin()+new_max_id, new_box_id, false, max_val);

        Box& new_box = boxes_.get(new_box_id);
        new_box.setMinId(axis, new_min_id);
        new_box.setMaxId(axis, new_max_id);

        findOverlapsOnAxis(new_box_id, axis);

        // fix ids for other boxes
        for (uint32_t i=0; i<boxes_.size(); ++i) {
            Box& b = boxes_.getAtPos(i);
            if (boxes_.getIndexForPos(i) == new_box_id)
                continue;

            uint32_t min_id = b.getMinId(axis);
            if (min_id >= new_min_id)
                ++min_id;
            if (min_id >= new_max_id)
                ++min_id;
            b.setMinId(axis, min_id);

            uint32_t max_id = b.getMaxId(axis);
            if (max_id >= new_min_id)
                ++max_id;
            if (max_id >= new_max_id)
                ++max_id;
            b.setMaxId(axis, max_id);
        }
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    inline void SAPManager<ClientData, AXES_COUNT>::findOverlapsOnAxis(uint32_t box_id, uint32_t axis) {
        Box& b = boxes_.get(box_id);
        SAP_internal::Points& ps = points_[axis];
        uint32_t min_id = b.getMinId(axis);
        uint32_t max_id = b.getMaxId(axis);

        uint32_t from = getScanStartId_(min_id, axis);

        for (uint32_t i=from; i< min_id; ++i) {
            uint32_t bid = ps[i].getBoxId();
            if (ps[i].getIsMin()) {
                if (!interesting_ids_flags_[bid]) {
                    interesting_ids_flags_[bid] = true;
                    interesting_ids_.push_back(bid);
                }
            }
        }

        for (uint32_t i= min_id +1; i<max_id; ++i) {
            uint32_t bid = ps[i].getBoxId();
            if (ps[i].getIsMin()) {
                if (!interesting_ids_flags_[bid]) {
                    interesting_ids_flags_[bid] = true;
                    interesting_ids_.push_back(bid);
                }
            }
        }
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    inline uint32_t SAPManager<ClientData, AXES_COUNT>::bisectInsertFind(SAP_internal::Points& points, float val, uint32_t from, uint32_t to) {
        uint32_t half = from + (to-from)/2;
        if (val < points[half].getValue()) {
            if (half == from)
                return from;
            else
                return bisectInsertFind(points, val, from, half-1);
        }
        else {
            if (half == to)
                return to+1;
            else
                return bisectInsertFind(points, val, half+1, to);
        }
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    inline SAP_internal::LongestSide SAPManager<ClientData, AXES_COUNT>::findLongestSide(uint32_t axis) {
        float longest_side = 0.0f;
        uint32_t longest_id = uint32_t(-1);
        for (uint32_t i = 0; i<boxes_.size(); ++i) {
            Box& b = boxes_.getAtPos(i);
            float min_val = points_[axis][b.getMinId(axis)].getValue();
            float max_val = points_[axis][b.getMaxId(axis)].getValue();
            float side = max_val-min_val;
            if (side > longest_side) {
                longest_side = side;
                longest_id = boxes_.getIndexForPos(i);
            }
        }
        return SAP_internal::LongestSide{longest_id, longest_side};
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    inline bool SAPManager<ClientData, AXES_COUNT>::boxesOverlap_(Box& b1, Box& b2) {
        for (uint32_t a=0; a<AXES_COUNT; ++a) {
            uint32_t axis = uint32_t(a);
            if (!intervalsOverlap_(b1.getMinId(axis), b1.getMaxId(axis), b2.getMinId(axis), b2.getMaxId(axis)))
                return false;
        }
        return true;
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    inline bool SAPManager<ClientData, AXES_COUNT>::intervalsOverlap_(uint32_t min1, uint32_t max1,
                                                          uint32_t min2, uint32_t max2) {
        return max1 > min2 && min1 < max2;
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    inline uint32_t SAPManager<ClientData, AXES_COUNT>::getScanStartId_(uint32_t min_id, uint32_t axis) {
        SAP_internal::Points& ps = points_[axis];
        float min_val = ps[min_id].getValue();
        // find from where we must scan for overlaps
        float longest_len = longest_sides_[axis].length;
        if (longest_len != 0.0f) {
            for (int32_t from= min_id -1; from>0; --from) {
                float len = min_val - ps[from].getValue();
                if (len > longest_len)
                    return uint32_t(from);
            }
        }
        return 0;
    }

    template <typename ClientData, uint32_t AXES_COUNT>
    inline void SAPManager<ClientData, AXES_COUNT>::clearInterestingFlags() {
        interesting_ids_flags_.clear();
        interesting_ids_flags_.resize(boxes_.size(), false);
        interesting_ids_.clear();
    }

}