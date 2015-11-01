#ifndef SAP_INTERNAL_H
#define SAP_INTERNAL_H

#include <stdint.h>
#include <bitset>
#include "types/containers/fast_vector.h"

namespace grynca {
    namespace SAP_internal {

        class EndPoint {
        public:
            EndPoint(uint32_t box_id, bool is_min, float value);

            bool getIsMin();
            void setIsMin(bool v);

            uint32_t getBoxId();
            void setBoxId(uint32_t bi);

            float getValue();
            void setValue(float v);

            static bool compareDesc(EndPoint& ep1, EndPoint& ep2);

        private:
            uint32_t pack_data_;        // 1b isMin, 31b boxId
            float value_;
        };

        struct Pair {
            uint32_t v1, v2;
        };

        struct LongestSide {
            LongestSide(uint32_t bid, float l) : box_id(bid), length(l) {}
            LongestSide() : box_id(uint32_t(-1)), length(0.0f) {}

            uint32_t box_id;
            float length;
        };

        template <typename ClientData, uint32_t AXES_COUNT>
        class Box_ {
        public:
            void setMinId(uint32_t a, uint32_t mid);
            void setMaxId(uint32_t a, uint32_t mid);
            uint32_t getMinId(uint32_t a);
            uint32_t getMaxId(uint32_t a);
            uint32_t getCollisionGroup();
            void setCollisionGroup(uint32_t group_id);
            void setClientData(const ClientData& cd);
            ClientData& getClientData();

            void addOverlap(uint32_t box_id);
            void removeOverlap(uint32_t box_id);

            fast_vector<uint32_t>& getOverlaps();
        private:
            Pair min_max_ids_[AXES_COUNT];
            ClientData client_data_;
            fast_vector<uint32_t> overlaps_;
            uint32_t collision_group_;
        };

        typedef fast_vector<EndPoint> Points;
    }
}

#include "SAP_internal.inl"
#endif //SAP_INTERNAL_H
