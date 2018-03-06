#include "SAP_internal.h"
#include "SAPSegment.h"

#define BOX_TPL template <typename SAPDomain>
#define BOX_TYPE SAPBox<SAPDomain>

namespace grynca {
    namespace SAP {

        inline EndPoint::EndPoint(u32 box_id, bool is_max, f32 value)
         : pack_data_(box_id), value_(value)
        {
            ASSERT(box_id < (1<<31));
            setIsMax(is_max);
        }

        inline u32 EndPoint::getIsMax() {
            return (pack_data_>>31)&1;
        }

        inline void EndPoint::setIsMax(bool v) {
            pack_data_ = SET_BITV(pack_data_, 31, v);
        }

        inline u32 EndPoint::getBoxId() {
            return pack_data_ & ~(1<<31);
        }

        inline f32 EndPoint::getValue() {
            return value_;
        }

        inline void EndPoint::setValue(f32 v) {
            value_ = v;
        }

        inline u32 EndPoint::getPackData() {
            return pack_data_;
        }

        inline bool EndPoint::compareDesc(EndPoint& ep1, EndPoint& ep2) {
        //static
            return ep1.getValue()>ep2.getValue();
        }

        inline CollPair::CollPair()
#ifdef DEBUG_BUILD
         : id1(InvalidId())
#endif
        {}

        inline CollPair::CollPair(u32 i1, u32 i2)
         : id1(i1), id2(i2)
        {
            if (id1>id2)
                std::swap(id1, id2);
        }

        inline bool CollPair::operator==(const CollPair& cp)const {
            return id1==cp.id1 && id2==cp.id2;
        }

        inline u32 CollPair::Hasher::operator()(const CollPair& cp)const {
            return calcHash32(cp.id);
        }

        BOX_TPL
        inline BOX_TYPE::SAPBox() : occurences_count_(0) {}

        BOX_TPL
        inline void BOX_TYPE::setClientData(const BoxDataT& cd) {
            client_data_ = cd;
        }

        BOX_TPL
        inline const typename BOX_TYPE::BoxDataT& BOX_TYPE::getClientData()const {
            return client_data_;
        }

        BOX_TPL
        inline typename BOX_TYPE::BoxDataT& BOX_TYPE::accClientData() {
            return client_data_;
        }

        BOX_TPL
        inline const f32* BOX_TYPE::getBounds()const {
            return bounds_;
        }

        BOX_TPL
        inline f32 BOX_TYPE::getMinValue(u32 a) {
            return bounds_[a];
        }

        BOX_TPL
        inline f32 BOX_TYPE::getMaxValue(u32 a) {
            return bounds_[AXES_COUNT+a];
        }

        BOX_TPL
        inline u32 BOX_TYPE::getOccurencesCount() {
            return occurences_count_;
        }

        BOX_TPL
        inline typename BOX_TYPE::Occurence& BOX_TYPE::getOccurence(u32 id) {
            return occurences_[id];
        }

        BOX_TPL
        inline u32 BOX_TYPE::findOccurence(Segment* segment) {
            for (u32 i=0; i<getOccurencesCount(); ++i) {
                if (occurences_[i].segment_ == segment) {
                    return i;
                }
            }
            return InvalidId();
        }

        BOX_TPL
        inline void BOX_TYPE::removeOccurence(u32 id) {
            ASSERT(getOccurencesCount());
            occurences_[id] = occurences_[occurences_count_-1];
            --occurences_count_;
        }

        BOX_TPL
        inline void BOX_TYPE::removeOccurence(Segment* segment) {
            u32 id = findOccurence(segment);
            ASSERT_M(id!=InvalidId(), "No such occurence");
            removeOccurence(id);
        }

        BOX_TPL
        inline void BOX_TYPE::changeOccurence(Segment* old_seg, Segment* new_seg) {
            u32 id = findOccurence(old_seg);
            ASSERT_M(id!=InvalidId(), "No such occurence");
            occurences_[id].segment_ = new_seg;
        }

        BOX_TPL
        inline void BOX_TYPE::setMinMaxId(Segment* segment, u32 a, u32 min_id, u32 max_id) {
            for (u32 i=0; i<getOccurencesCount(); ++i) {
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
        inline void BOX_TYPE::setEndPointId(Segment* segment, u32 a, u32 epid, u32 min_or_max) {
            for (u32 i=0; i<getOccurencesCount(); ++i) {
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
        inline u32 BOX_TYPE::getMinId(Segment* segment, u32 a) {
            u32 id = findOccurence(segment);
            ASSERT_M(id!=InvalidId(), "No such occurence");
            return occurences_[id].min_max_ids_[a].accMin();
        }

        BOX_TPL
        inline u32 BOX_TYPE::getMaxId(Segment* segment, u32 a) {
            u32 id = findOccurence(segment);
            ASSERT_M(id!=InvalidId(), "No such occurence");
            return occurences_[id].min_max_ids_[a].accMax();
        }

        BOX_TPL
        inline void BOX_TYPE::getMinsMaxs(Segment* segment, MinMax* mins_maxs_out) {
            u32 id = findOccurence(segment);
            ASSERT_M(id!=InvalidId(), "No such occurence");
            memcpy(mins_maxs_out, occurences_[id].min_max_ids_, sizeof(MinMax)*AXES_COUNT);
        }

    }
}

#undef BOX_TPL
#undef BOX_TYPE