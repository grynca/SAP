#ifndef SAP_INTERNAL_H
#define SAP_INTERNAL_H

#include <stdint.h>
#include <bitset>
#include "types/containers/HashSet.h"
#include "SAP_config.h"

namespace grynca {

    // fw
    template <typename, uint32_t> class SAPSegment;

    namespace SAP {

        class EndPoint {
        public:
            EndPoint(uint32_t box_id, bool is_max, float value);

            uint32_t getIsMax();
            void setIsMax(bool v);

            uint32_t getBoxId();
            void setBoxId(uint32_t bi);

            float getValue();
            void setValue(float v);
            uint32_t getPackData();

            static bool compareDesc(EndPoint& ep1, EndPoint& ep2);

        private:
            uint32_t pack_data_;        // 1b isMax, 31b boxId
            float value_;
        };

        struct CollPair {
            CollPair();
            CollPair(uint32_t i1, uint32_t i2);

            bool operator==(const CollPair& cp)const;

            uint32_t id1;
            uint32_t id2;

            struct Hasher {
                uint32_t operator()(const CollPair& cp)const;
            };
        };

        struct MinMax {
            uint32_t& accMin() { return v[0]; }
            uint32_t& accMax() { return v[1]; }

            uint32_t v[2];
        };

        struct LongestSide {
            LongestSide(uint32_t bid, float l) : box_id(bid), length(l) {}
            LongestSide() : box_id(), length(0.0f) {}

            uint32_t box_id;
            float length;
        };

        template <typename ClientData, uint32_t AXES_COUNT>
        class Box_ {
            typedef SAPSegment<ClientData, AXES_COUNT> Segment;
        public:
            struct Occurence {
                Segment* segment_;
                MinMax min_max_ids_[AXES_COUNT];
            };
        public:
            Box_() : occurences_count_(0) {}

            uint32_t getOccurencesCount();
            Occurence& getOccurence(uint32_t id) { return occurences_[id]; }
            uint32_t findOccurence(Segment* segment);
            void removeOccurence(uint32_t id);
            void removeOccurence(Segment* segment);
            void changeOccurence(Segment* old_seg, Segment* new_seg);
            void setMinMaxId(Segment* segment, uint32_t a, uint32_t min_id, uint32_t max_id);
            void setEndPointId(Segment* segment, uint32_t a, uint32_t epid, uint32_t min_or_max /* min == 0, max == 1*/);
            uint32_t getMinId(Segment* segment, uint32_t a);
            uint32_t getMaxId(Segment* segment, uint32_t a);
            void getMinsMaxs(Segment* segment, MinMax* mins_maxs_out);
            float getMinValue(uint32_t a);
            float getMaxValue(uint32_t a);
            void getMinMaxValue(uint32_t a, float& min, float& max);
            void setClientData(const ClientData& cd);
            ClientData& getClientData();
        private:
            template <typename, uint32_t> friend class SAPManager;
            template <typename, uint32_t> friend class SAPSegment;

            ClientData client_data_;
            Occurence occurences_[SAP::MAX_BOX_OCCURENCES];
            uint32_t occurences_count_;
        };

        struct Overlaps {
            Overlaps() : pm(PM_INITIAL_SIZE) {}

            // temp ids
            fast_vector<uint32_t> possibly_added_;
            fast_vector<uint32_t> removed_;

            HashSet<CollPair, CollPair::Hasher> pm;
        };

        typedef fast_vector<EndPoint> Points;
    }
}

#include "SAP_internal.inl"
#endif //SAP_INTERNAL_H
