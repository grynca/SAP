#ifndef SAPRAYCASTER_H
#define SAPRAYCASTER_H

#include <stdint.h>
#include <functional>

namespace grynca {

    //fw
    template <typename CD, uint32_t AC> class SAPManager;

    template <typename ClientData, uint32_t AXES_COUNT>
    class SAPRaycaster {
        typedef SAPSegment<ClientData, AXES_COUNT> Segment;
        typedef SAPManager<ClientData, AXES_COUNT> Manager;
    public:
        typedef std::function<bool(uint32_t/*box_id*/, float /*dt*/)> HitCallback;

        // dir is not normalized
        void setRay(float* origin, float* dir);

        // cb returns false if no more hits are desired
        void getHits(const HitCallback& cb);

    private:
        template <typename, uint32_t> friend class SAPManager;

        SAPRaycaster(Manager& mgr);
        bool overlapBox_(float* bounds, float& t_out);
        bool overlapSegment_(Segment* seg, float& t_out);
        bool getHitsRec_(const HitCallback& cb, Segment* seg);

        struct Ray {
            float origin[AXES_COUNT];
            float dir[AXES_COUNT];
            float inv_dir[AXES_COUNT];
            uint8_t dir_sgn[AXES_COUNT];
        };

        struct BoxOp {
            uint32_t box_id;
            float t;


            static bool compare(BoxOp& b1, BoxOp& b2) {
                return b1.t < b2.t;
            }
        };

        Manager* mgr_;
        Ray ray_;
        fast_vector<BoxOp> overlaps_in_curr_seg_;
        uint32_t prev_box_;
    };

}


#include "SAPRaycaster.inl"
#endif //SAPRAYCASTER_H
