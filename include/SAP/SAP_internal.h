#ifndef SAP_INTERNAL_H
#define SAP_INTERNAL_H

#include <stdint.h>
#include <bitset>
#include "types/containers/HashSet.h"
#include "SAP_config.h"

namespace grynca {

    // fw
    template <typename, u32> class SAPSegment;

    namespace SAP {

        static const u8 InvalidAxis = u8(-1);

        class EndPoint {
        public:
            EndPoint(u32 box_id, bool is_max, f32 value);

            u32 getIsMax();
            void setIsMax(bool v);

            u32 getBoxId();
            void setBoxId(u32 bi);

            f32 getValue();
            void setValue(f32 v);
            u32 getPackData();

            static bool compareDesc(EndPoint& ep1, EndPoint& ep2);

        private:
            u32 pack_data_;        // 1b isMax, 31b boxId
            f32 value_;
        };

        struct CollPair {
            CollPair();
            CollPair(u32 i1, u32 i2);

            bool operator==(const CollPair& cp)const;

            u32 id1;
            u32 id2;

            struct Hasher {
                u32 operator()(const CollPair& cp)const;
            };
        };

        struct MinMax {
            u32& accMin() { return v[0]; }
            u32& accMax() { return v[1]; }

            u32 v[2];
        };

        struct LongestSide {
            LongestSide(u32 bid, f32 l) : box_id(bid), length(l) {}
            LongestSide() : box_id(), length(0.0f) {}

            u32 box_id;
            f32 length;
        };

        template <typename ClientData, u32 AXES_COUNT>
        class Box_ {
            typedef SAPSegment<ClientData, AXES_COUNT> Segment;
        public:
            struct Occurence {
                Segment* segment_;
                MinMax min_max_ids_[AXES_COUNT];
            };
        public:
            Box_() : occurences_count_(0) {}

            u32 getOccurencesCount();
            Occurence& getOccurence(u32 id) { return occurences_[id]; }
            u32 findOccurence(Segment* segment);
            void removeOccurence(u32 id);
            void removeOccurence(Segment* segment);
            void changeOccurence(Segment* old_seg, Segment* new_seg);
            void setMinMaxId(Segment* segment, u32 a, u32 min_id, u32 max_id);
            void setEndPointId(Segment* segment, u32 a, u32 epid, u32 min_or_max /* min == 0, max == 1*/);
            u32 getMinId(Segment* segment, u32 a);
            u32 getMaxId(Segment* segment, u32 a);
            void getMinsMaxs(Segment* segment, MinMax* mins_maxs_out);
            f32 getMinValue(u32 a);
            f32 getMaxValue(u32 a);
            void getMinMaxValue(u32 a, f32& min, f32& max);
            void setClientData(const ClientData& cd);
            ClientData& getClientData();
        private:
            template <typename, u32> friend class SAPManager;
            template <typename, u32> friend class SAPSegment;

            ClientData client_data_;
            Occurence occurences_[SAP::MAX_BOX_OCCURENCES];
            u32 occurences_count_;
        };

        struct Overlaps {
            Overlaps() : pm(PM_INITIAL_SIZE) {}

            // temp ids
            fast_vector<u32> possibly_added_;
            fast_vector<u32> removed_;

            HashSet<CollPair, CollPair::Hasher> pm;
        };

        typedef fast_vector<EndPoint> Points;
    }
}

#include "SAP_internal.inl"
#endif //SAP_INTERNAL_H
