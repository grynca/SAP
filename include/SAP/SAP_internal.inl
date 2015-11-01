#include "SAP_internal.h"

namespace grynca {
    namespace SAP_internal {

        inline EndPoint::EndPoint(uint32_t box_id, bool is_min, float value)
         : pack_data_(0), value_(value)
        {
            setBoxId(box_id);
            setIsMin(is_min);
        }

        inline bool EndPoint::getIsMin() {
            return (pack_data_>>31)&1;
        }

        inline void EndPoint::setIsMin(bool v) {
            if (v)
                pack_data_ |= 1<<31;
            else
                pack_data_ &= ~(1<<31);
        }

        inline uint32_t EndPoint::getBoxId() {
            return pack_data_ & ~(1<<31);
        }

        inline void EndPoint::setBoxId(uint32_t bi) {
            pack_data_ |= bi & ~(1<<31);
        }

        inline float EndPoint::getValue() {
            return value_;
        }

        inline void EndPoint::setValue(float v) {
            value_ = v;
        }

        inline bool EndPoint::compareDesc(EndPoint& ep1, EndPoint& ep2) {
        //static
            return ep1.getValue()>ep2.getValue();
        }


        template <typename ClientData, uint32_t AXES_COUNT>
        inline void Box_<ClientData, AXES_COUNT>::setMinId(uint32_t a, uint32_t mid) {
            min_max_ids_[a].v1 = mid;
        }

        template <typename ClientData, uint32_t AXES_COUNT>
        inline void Box_<ClientData, AXES_COUNT>::setMaxId(uint32_t a, uint32_t mid) {
            min_max_ids_[a].v2 = mid;
        }

        template <typename ClientData, uint32_t AXES_COUNT>
        inline uint32_t Box_<ClientData, AXES_COUNT>::getMinId(uint32_t a) {
            return min_max_ids_[a].v1;
        }

        template <typename ClientData, uint32_t AXES_COUNT>
        inline uint32_t Box_<ClientData, AXES_COUNT>::getMaxId(uint32_t a) {
            return min_max_ids_[a].v2;
        }

        template <typename ClientData, uint32_t AXES_COUNT>
        inline uint32_t Box_<ClientData, AXES_COUNT>::getCollisionGroup() {
            return collision_group_;
        }

        template <typename ClientData, uint32_t AXES_COUNT>
        inline void Box_<ClientData, AXES_COUNT>::setCollisionGroup(uint32_t group_id) {
            collision_group_ = group_id;
        }

        template <typename ClientData, uint32_t AXES_COUNT>
        inline void Box_<ClientData, AXES_COUNT>::setClientData(const ClientData& cd) {
            client_data_ = cd;
        }

        template <typename ClientData, uint32_t AXES_COUNT>
        inline ClientData& Box_<ClientData, AXES_COUNT>::getClientData() {
            return client_data_;
        }

        template <typename ClientData, uint32_t AXES_COUNT>
        inline void Box_<ClientData, AXES_COUNT>::addOverlap(uint32_t box_id) {
            fast_vector<uint32_t>::iterator it = std::find(overlaps_.begin(), overlaps_.end(), box_id);
            if (it == overlaps_.end()) {
                overlaps_.push_back(box_id);
            }
        }

        template <typename ClientData, uint32_t AXES_COUNT>
        inline void Box_<ClientData, AXES_COUNT>::removeOverlap(uint32_t box_id) {
            fast_vector<uint32_t>::iterator it = std::find(overlaps_.begin(), overlaps_.end(), box_id);
            if (it != overlaps_.end()) {
                *it = overlaps_.back();
                overlaps_.pop_back();
            }
        }

        template <typename ClientData, uint32_t AXES_COUNT>
        inline fast_vector<uint32_t>& Box_<ClientData, AXES_COUNT>::getOverlaps() {
            return overlaps_;
        }

    }
}