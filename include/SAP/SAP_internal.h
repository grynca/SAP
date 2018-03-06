#ifndef SAP_INTERNAL_H
#define SAP_INTERNAL_H

#include <stdint.h>
#include <bitset>
#include "types/containers/HashMap.h"
#include "SAP_config.h"
#include "SAP_domain.h"

namespace grynca {

    namespace SAP {

        static const u8 InvalidAxis = u8(-1);

        class EndPoint {
        public:
            EndPoint(u32 box_id, bool is_max, f32 value);

            u32 getIsMax();
            void setIsMax(bool v);

            u32 getBoxId();

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

            struct Hasher {
                u32 operator()(const CollPair& cp)const;
            };

            union {
                struct {
                    u32 id1;
                    u32 id2;
                };
                u64 id;
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

        template <typename SAPDomain>
        class SAPBox {
        public:
            SAP_DOMAIN_TYPES(SAPDomain);

            SAPBox();

            void setClientData(const BoxDataT& cd);
            const BoxDataT& getClientData()const;
            BoxDataT& accClientData();

            const f32* getBounds()const;
            f32 getMinValue(u32 a);
            f32 getMaxValue(u32 a);
        protected:
            friend Segment;
            friend Manager;

            struct Occurence {
                Segment* segment_;
                MinMax min_max_ids_[AXES_COUNT];
            };

            u32 getOccurencesCount();
            Occurence& getOccurence(u32 id);
            u32 findOccurence(Segment* segment);
            void removeOccurence(u32 id);
            void removeOccurence(Segment* segment);
            void changeOccurence(Segment* old_seg, Segment* new_seg);
            void setMinMaxId(Segment* segment, u32 a, u32 min_id, u32 max_id);
            void setEndPointId(Segment* segment, u32 a, u32 epid, u32 min_or_max /* min == 0, max == 1*/);
            u32 getMinId(Segment* segment, u32 a);
            u32 getMaxId(Segment* segment, u32 a);
            void getMinsMaxs(Segment* segment, MinMax* mins_maxs_out);

            f32 bounds_[2*AXES_COUNT];
            BoxDataT client_data_;
            Occurence occurences_[SAP::MAX_BOX_OCCURENCES];
            u32 occurences_count_;
        };

        template <typename ClientDataCollision>
        struct Overlaps {
            Overlaps() : pm(PM_INITIAL_SIZE) {}

            // temp ids
            fast_vector<u32> possibly_added_;
            fast_vector<u32> removed_;

            HashMap<ClientDataCollision, CollPair, CollPair::Hasher> pm;
        };

        typedef fast_vector<EndPoint> Points;
    }
}

#include "SAP_internal.inl"
#endif //SAP_INTERNAL_H
