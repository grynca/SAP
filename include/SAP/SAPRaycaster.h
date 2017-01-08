#ifndef SAPRAYCASTER_H
#define SAPRAYCASTER_H

#include <stdint.h>
#include <functional>

namespace grynca {

    //fw
    template <typename CD, u32 AC> class SAPManager;

    template <typename ClientData, u32 AXES_COUNT>
    class SAPRaycaster {
        typedef SAPSegment<ClientData, AXES_COUNT> Segment;
        typedef SAPManager<ClientData, AXES_COUNT> Manager;
    public:
        typedef std::function<bool(u32/*box_id*/, f32 /*dt*/)> HitCallback;

        // dir is not normalized
        void setRay(f32* origin, f32* dir);

        // cb returns false if no more hits are desired
        void getHits(const HitCallback& cb);

    private:
        template <typename, u32> friend class SAPManager;

        SAPRaycaster(Manager& mgr);
        bool overlapBox_(f32* bounds, f32& t_out);
        bool overlapSegment_(Segment* seg, f32& t_out);
        bool getHitsRec_(const HitCallback& cb, Segment* seg);

        struct Ray {
            f32 origin[AXES_COUNT];
            f32 dir[AXES_COUNT];
            f32 inv_dir[AXES_COUNT];
            u8 dir_sgn[AXES_COUNT];
        };

        struct BoxOp {
            u32 box_id;
            f32 t;


            static bool compare(BoxOp& b1, BoxOp& b2) {
                return b1.t < b2.t;
            }
        };

        Manager* mgr_;
        Ray ray_;
        fast_vector<BoxOp> overlaps_in_curr_seg_;
        u32 prev_box_;
    };

}


#include "SAPRaycaster.inl"
#endif //SAPRAYCASTER_H
