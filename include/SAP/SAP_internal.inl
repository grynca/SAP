#include "SAP_internal.h"
#include "SAPSegment.h"

#define BOX_TPL template <typename ClientData, uint32_t AXES_COUNT>
#define BOX_TYPE Box_<ClientData, AXES_COUNT>

namespace grynca {
    namespace SAP {

        inline EndPoint::EndPoint(uint32_t box_id, bool is_max, float value)
         : pack_data_(0), value_(value)
        {
            setBoxId(box_id);
            setIsMax(is_max);
        }

        inline uint32_t EndPoint::getIsMax() {
            return (pack_data_>>31)&1;
        }

        inline void EndPoint::setIsMax(bool v) {
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

        inline uint32_t EndPoint::getPackData() {
            return pack_data_;
        }

        inline bool EndPoint::compareDesc(EndPoint& ep1, EndPoint& ep2) {
        //static
            return ep1.getValue()>ep2.getValue();
        }


        BOX_TPL
        inline uint32_t BOX_TYPE::getOccurencesCount() {
            return occurences_count_;
        }

        BOX_TPL
        inline uint32_t BOX_TYPE::findOccurence(Segment* segment) {
            for (uint32_t i=0; i<getOccurencesCount(); ++i) {
                if (occurences_[i].segment_ == segment) {
                    return i;
                }
            }
            return InvalidId();
        }

        BOX_TPL
        inline void BOX_TYPE::removeOccurence(uint32_t id) {
            ASSERT(getOccurencesCount());
            occurences_[id] = occurences_[occurences_count_-1];
            --occurences_count_;
        }

        BOX_TPL
        inline void BOX_TYPE::removeOccurence(Segment* segment) {
            uint32_t id = findOccurence(segment);
            ASSERT_M(id!=InvalidId(), "No such occurence");
            removeOccurence(id);
        }

        BOX_TPL
        inline void BOX_TYPE::changeOccurence(Segment* old_seg, Segment* new_seg) {
            uint32_t id = findOccurence(old_seg);
            ASSERT_M(id!=InvalidId(), "No such occurence");
            occurences_[id].segment_ = new_seg;
        }

        BOX_TPL
        inline void BOX_TYPE::setMinMaxId(Segment* segment, uint32_t a, uint32_t min_id, uint32_t max_id) {
            for (uint32_t i=0; i<getOccurencesCount(); ++i) {
                if (occurences_[i].segment_ == segment) {
                    occurences_[i].min_max_ids_[a].v[0] = min_id;
                    occurences_[i].min_max_ids_[a].v[1] = max_id;
                    return;
                }
            }
            ASSERT(occurences_count_ < SAP::MAX_BOX_OCCURENCES);
            occurences_[occurences_count_].segment_ = segment;
            occurences_[occurences_count_].min_max_ids_[a].v[0] = min_id;
            occurences_[occurences_count_].min_max_ids_[a].v[1] = max_id;
            ++occurences_count_;
        }

        BOX_TPL
        inline void BOX_TYPE::setEndPointId(Segment* segment, uint32_t a, uint32_t epid, uint32_t min_or_max) {
            for (uint32_t i=0; i<getOccurencesCount(); ++i) {
                if (occurences_[i].segment_ == segment) {
                    occurences_[i].min_max_ids_[a].v[min_or_max] = epid;
                    return;
                }
            }

            ASSERT(occurences_count_ < SAP::MAX_BOX_OCCURENCES);
            occurences_[occurences_count_].segment_ = segment;
            occurences_[occurences_count_].min_max_ids_[a].v[min_or_max] = epid;
            ++occurences_count_;
        }

        BOX_TPL
        inline uint32_t BOX_TYPE::getMinId(Segment* segment, uint32_t a) {
            uint32_t id = findOccurence(segment);
            ASSERT_M(id!=InvalidId(), "No such occurence");
            return occurences_[id].min_max_ids_[a].accMin();
        }

        BOX_TPL
        inline uint32_t BOX_TYPE::getMaxId(Segment* segment, uint32_t a) {
            uint32_t id = findOccurence(segment);
            ASSERT_M(id!=InvalidId(), "No such occurence");
            return occurences_[id].min_max_ids_[a].accMax();
        }

        BOX_TPL
        inline void BOX_TYPE::getMinsMaxs(Segment* segment, Pair* mins_maxs_out) {
            uint32_t id = findOccurence(segment);
            ASSERT_M(id!=InvalidId(), "No such occurence");
            memcpy(mins_maxs_out, occurences_[id].min_max_ids_, sizeof(Pair)*AXES_COUNT);
        }

        BOX_TPL
        inline float BOX_TYPE::getMinValue(uint32_t a) {
            ASSERT(getOccurencesCount() && "Box must have at least 1 occurence");
            Segment* seg = occurences_[0].segment_;
            return seg->points_[a][occurences_[0].min_max_ids_[a].accMin()].getValue();
        }

        BOX_TPL
        inline float BOX_TYPE::getMaxValue(uint32_t a) {
            ASSERT(getOccurencesCount() && "Box must have at least 1 occurence");
            Segment* seg = occurences_[0].segment_;
            return seg->points_[a][occurences_[0].min_max_ids_[a].accMax()].getValue();
        }

        BOX_TPL
        inline void BOX_TYPE::getMinMaxValue(uint32_t a, float& min, float& max) {
            ASSERT(getOccurencesCount() && "Box must have at least 1 occurence");
            Segment* seg = occurences_[0].segment_;
            Pair min_max_id = occurences_[0].min_max_ids_[a];
            min = seg->points_[a][min_max_id.v[0]].getValue();
            max = seg->points_[a][min_max_id.v[1]].getValue();
        }

        BOX_TPL
        inline void BOX_TYPE::setClientData(const ClientData& cd) {
            client_data_ = cd;
        }

        BOX_TPL
        inline ClientData& BOX_TYPE::getClientData() {
            return client_data_;
        }
    }
}

#undef BOX_TPL
#undef BOX_TYPE