#include "SAPRaycaster.h"
#include "SAPManager.h"
#include <cstring>
#include "base.h"

#define SRC_TPL template <typename ClientData, u32 AXES_COUNT>
#define SRC_TYPE SAPRaycaster<ClientData, AXES_COUNT>
#define GET_COORD(BOUNDS, POINT, AXIS) BOUNDS[(POINT)*AXES_COUNT + AXIS]

namespace grynca {

    SRC_TPL
    inline void SRC_TYPE::setRay(f32* origin, f32* dir) {
        memcpy(ray_.dir, dir, AXES_COUNT*sizeof(f32));
        memcpy(ray_.origin, origin, AXES_COUNT*sizeof(f32));
        for (u32 a=0; a<AXES_COUNT; ++a) {
            ray_.inv_dir[a] = 1.0f/ray_.dir[a];
            ray_.dir_sgn[a] = (u8)(ray_.inv_dir[a] < 0);
        }
    }

    SRC_TPL
    inline void SRC_TYPE::getHits(const HitCallback& cb) {
        getHitsRec_(cb, mgr_->getRootSegment());
    }

    SRC_TPL
    inline SRC_TYPE::SAPRaycaster(Manager& mgr)
     : mgr_(&mgr), prev_box_(InvalidId())
    {
    }


    SRC_TPL
    inline bool SRC_TYPE::overlapBox_(f32* bounds, f32& t_out) {

        f32 tmin = (GET_COORD(bounds, ray_.dir_sgn[0], 0) - ray_.origin[0])*ray_.inv_dir[0];
        f32 tmax = (GET_COORD(bounds, 1-ray_.dir_sgn[0], 0) - ray_.origin[0])*ray_.inv_dir[0];

        for (u32 a=1; a<AXES_COUNT; ++a) {

            f32 tymin = (GET_COORD(bounds, ray_.dir_sgn[a], a) - ray_.origin[a]) * ray_.inv_dir[a];
            if (tymin > tmax)
                return false;
            f32 tymax = (GET_COORD(bounds, 1-ray_.dir_sgn[a], a) - ray_.origin[a]) * ray_.inv_dir[a];
            if (tmin > tymax)
                return false;

            if (tymin > tmin)
                tmin = tymin;
            if (tymax < tmax)
                tmax = tymax;
        }

        if (tmin < 0) {
            if (tmax < 0)
                return false;
            //inside box
            t_out = tmax;
        }
        else {
            if (tmin > 1.0f)
                return false;
            t_out = tmin;
        }
        return true;
    }

    SRC_TPL
    inline bool SRC_TYPE::overlapSegment_(Segment* seg, f32& t_out) {
        f32 bounds[AXES_COUNT*2];
        for (u32 a=0; a<AXES_COUNT; ++a) {
            seg->getLowBorder(a, bounds[a]);
            seg->getHighBorder(a, bounds[AXES_COUNT+a]);
        }
        return overlapBox_(bounds, t_out);
    }

    SRC_TPL
    inline bool SRC_TYPE::getHitsRec_(const HitCallback& cb, Segment* seg) {
        if (seg->isSplit()) {
            f32 t1, t2;
            bool in1 = overlapSegment_(seg->getChild(0), t1);
            bool in2 = overlapSegment_(seg->getChild(1), t2);
            if (in1) {
                if (in2) {
                    if (t1>t2) {
                        if (!getHitsRec_(cb, seg->getChild(1)))
                            return false;
                        if (!getHitsRec_(cb, seg->getChild(0)))
                            return false;
                    }
                    else {
                        if (!getHitsRec_(cb, seg->getChild(0)))
                            return false;
                        if (!getHitsRec_(cb, seg->getChild(1)))
                            return false;
                    }
                }
                else {
                    if (!getHitsRec_(cb, seg->getChild(0)))
                        return false;
                }
            }
            else if (in2) {
                if (!getHitsRec_(cb, seg->getChild(1)))
                    return false;
            }
        }
        else {
            f32 bounds[AXES_COUNT*2];
            for (u32 pid=0; pid<seg->points_[0].size(); ++pid) {
                SAP::EndPoint &p = seg->points_[0][pid];
                if (p.getIsMax()) {
                    u32 bid = p.getBoxId();
                    mgr_->getBox(bid, bounds);
                    f32 t;
                    if (overlapBox_(bounds, t)) {
                        overlaps_in_curr_seg_.push_back({bid, t});
                    }
                }
            }
            std::sort(overlaps_in_curr_seg_.begin(), overlaps_in_curr_seg_.end(), BoxOp::compare);
            for (u32 i=0; i<overlaps_in_curr_seg_.size(); ++i) {
                if (overlaps_in_curr_seg_[i].box_id == prev_box_)
                    continue;
                prev_box_ = overlaps_in_curr_seg_[i].box_id;
                if (!cb(overlaps_in_curr_seg_[i].box_id, overlaps_in_curr_seg_[i].t)) {
                    overlaps_in_curr_seg_.clear();
                    prev_box_ = InvalidId();
                    return false;
                }
            }
            overlaps_in_curr_seg_.clear();
        }
        return true;
    }
}

#undef SRC_TPL
#undef SRC_TYPE
#undef GET_COORD