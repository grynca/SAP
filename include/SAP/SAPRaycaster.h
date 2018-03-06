#ifndef SAPRAYCASTER_H
#define SAPRAYCASTER_H

#include "SAP_internal.h"
#include <functional>

namespace grynca {

    template <typename SAPDomain>
    class SAPRaycaster {
    public:
        SAP_DOMAIN_TYPES(SAPDomain);

        // dir is not normalized
        void setRay(f32* origin, f32* dir);

        // cb returns false if no more hits are desired
        // bool cb(u32 box_id, f32 dt)
        template <typename HitCallback>
        void getHits(const HitCallback& cb);

    private:
        friend Manager;

        SAPRaycaster(const Manager& mgr);
        bool overlapBox_(const f32* bounds, f32& t_out)const;
        bool overlapSegment_(Segment* seg, f32& t_out)const;
        template <typename HitCallback>
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

        const Manager* mgr_;
        Ray ray_;
        fast_vector<BoxOp> overlaps_in_curr_seg_;
        u32 prev_box_;
    };

}


#include "SAPRaycaster.inl"
#endif //SAPRAYCASTER_H
