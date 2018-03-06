#include "SAPManagerC.h"
#include "SAPSegment.h"
#include "SAPRaycaster.h"
#include "base.h"
#include <algorithm>
#include <cassert>
#include <sstream>

#define SMB_TPL template <typename SAPDomain>
#define SMB_TYPE SAPManagerBase<SAPDomain>
#define SM_TPL template <typename Derived, typename SAPDomain>
#define SM_TYPE SAPManagerC<Derived, SAPDomain>
#define GET_MIN(BOUNDS, AXIS) BOUNDS[AXIS]
#define GET_MAX(BOUNDS, AXIS) BOUNDS[AXES_COUNT+AXIS]

namespace grynca {

    SMB_TPL
    inline SMB_TYPE::SAPManagerBase()
     : root_(new Segment(*this, NULL))
    {
        root_->setDebugName_();
        root_->calcBorders_();
    }

    SMB_TPL
    inline SMB_TYPE::~SAPManagerBase() {
        delete root_;
        boxes_.clear();
    }

    SMB_TPL
    inline bool SMB_TYPE::containsBox(Index box_id)const {
        return boxes_.isValidIndex(box_id);
    }

    SMB_TPL
    inline typename SMB_TYPE::Box& SMB_TYPE::accBox(Index box_id) {
        return boxes_.accItem(box_id);
    }

    SMB_TPL
    inline const typename SMB_TYPE::Box& SMB_TYPE::getBox(Index box_id)const {
        return boxes_.getItem(box_id);
    }

    SMB_TPL
    inline u32 SMB_TYPE::getBoxesCount()const {
        return boxes_.size();
    }

    SMB_TPL
    inline typename SMB_TYPE::Segment* SMB_TYPE::getRootSegment()const {
        return root_;
    }

    SMB_TPL
    inline void SMB_TYPE::validate() {
        u32 boxes_in_segments = 0;
        fast_vector<Segment*> s{root_};
        while (!s.empty()) {
            Segment* seg = s.back();
            s.pop_back();

            if (seg->isSplit()) {
                s.push_back(seg->getChild(0));
                s.push_back(seg->getChild(1));
            }
            else {
                boxes_in_segments += seg->getBoxesCount();
            }
        }

        u32 boxes_cnt = 0;
        for (u32 box_pos=0; box_pos<boxes_.size(); ++box_pos) {
            Box& box = boxes_.accItemAtPos2(box_pos);
            Index box_id = boxes_.getIndexForPos(box_pos);
            ASSERT(box.getOccurencesCount());
            boxes_cnt += box.getOccurencesCount();
            for (u32 i=0; i<box.getOccurencesCount(); ++i) {
                Segment* seg = box.getOccurence(i).segment_;

                ASSERT(!seg->isSplit());

                for (u32 a=0; a<AXES_COUNT; ++a) {
                    u32 min_id = box.getOccurence(i).min_max_ids_[a].v[0];
                    u32 max_id = box.getOccurence(i).min_max_ids_[a].v[1];
                    ASSERT(min_id < max_id);
                    ASSERT(min_id < seg->points_[a].size());
                    ASSERT(max_id < seg->points_[a].size());
                    ASSERT(seg->points_[a][min_id].getBoxId() == box_id.getIndex());
                    ASSERT(seg->points_[a][max_id].getBoxId() == box_id.getIndex());

                    f32 border_val;
                    ASSERT(!seg->getHighBorder(a, border_val) || seg->points_[a][min_id].getValue() < border_val);
                    ASSERT(!seg->getLowBorder(a, border_val) || seg->points_[a][max_id].getValue() > border_val);
                }
            }
        }
#ifdef SAP_VALIDATE_OVERLAPS
        for (u32 b1_pos=0; b1_pos<boxes_.size(); ++b1_pos) {
            Box& b1 = boxes_.accItemAtPos2(b1_pos);
            Index b1_id = boxes_.getIndexForPos(b1_pos);
            for (u32 b2_pos=b1_pos+1; b2_pos<boxes_.size(); ++b2_pos) {
                Box& b2 = boxes_.accItemAtPos2(b2_pos);
                Index b2_id = boxes_.getIndexForPos(b2_pos);
                bool overlaps_in_sap = bool(overlaps_.pm.findItem(SAP::CollPair(b1_id.getIndex(), b2_id.getIndex())));
                bool overlaps_ground_truth = boxesOverlap_(b1, b2);
                if (overlaps_in_sap != overlaps_ground_truth) {
                    overlaps_ground_truth = boxesOverlap_(b1, b2);
                }
                ASSERT(overlaps_in_sap == overlaps_ground_truth);
            }
        }
#endif
    }

    SMB_TPL
    inline u32 SMB_TYPE::getOverlapsCount()const {
        return overlaps_.pm.getItemsCount();
    }

    SMB_TPL
    inline void SMB_TYPE::getOverlap(u32 overlap_id, Index& b1_id_out, Index& b2_id_out)const {
        const SAP::CollPair& p = overlaps_.pm.getKey(overlap_id);
        b1_id_out = boxes_.getFullIndex(p.id1);
        b2_id_out = boxes_.getFullIndex(p.id2);
    }

    SMB_TPL
    inline void SMB_TYPE::getOverlapWithData(u32 overlap_id, Index& b1_id_out, Index& b2_id_out, OverlapDataT*& coll_data_ptr_out) {
        const SAP::CollPair& p = overlaps_.pm.getKey(overlap_id);
        b1_id_out = boxes_.getFullIndex(p.id1);
        b2_id_out = boxes_.getFullIndex(p.id2);
        coll_data_ptr_out = &overlaps_.pm.accItem(overlap_id);
    }

    SMB_TPL
    inline typename SMB_TYPE::OverlapDataT* SMB_TYPE::findOverlap(Index b1_id, Index b2_id) {
        return overlaps_.pm.findItem(SAP::CollPair(b1_id.getIndex(), b2_id.getIndex()));
    }

    SMB_TPL
    inline typename SMB_TYPE::OverlapDataT* SMB_TYPE::findOverlap(Box& b1, Box& b2) {
        u32 b1_pos = boxes_.getItemPos(&b1);
        u32 b2_pos = boxes_.getItemPos(&b2);
        u32 b1_inner_id = boxes_.getPool().getInnerIndexForPos(b1_pos);
        u32 b2_inner_id = boxes_.getPool().getInnerIndexForPos(b2_pos);
        return overlaps_.pm.findItem(SAP::CollPair(b1_inner_id, b2_inner_id));
    }

    SMB_TPL
    inline SAPRaycaster<SAPDomain> SMB_TYPE::getRayCaster()const {
        return Raycaster(*this);
    }

    SMB_TPL
    inline void SMB_TYPE::calcBounds(f32* bounds) {
        ASSERT(getBoxesCount());
        for (u32 a=0; a<AXES_COUNT; ++a) {
            bounds[a] = root_->findLowestPointRec_(a);
            bounds[AXES_COUNT+a] = root_->findHighestPointRec_(a);
        }
    }

    SMB_TPL
    inline u32 SMB_TYPE::getGroupId(u32 box_id)const {
        return boxes_.getItem(box_id).getCollisionGroup();
    }

    SMB_TPL
    inline bool SMB_TYPE::boxesOverlap_(Box& b1, Box& b2) {
        for (u32 a=0; a<AXES_COUNT; ++a) {
            u32 axis = u32(a);

            f32 b2_min = b2.getMinValue(axis);
            f32 b1_max = b1.getMaxValue(axis);
            if (b2_min > b1_max)
                return false;
            f32 b1_min = b1.getMinValue(axis);
            f32 b2_max = b2.getMaxValue(axis);
            if (b2_max < b1_min)
                return false;
        }
        return true;
    }

    SMB_TPL
    inline std::string SMB_TYPE::debugPrint() {
        u32 boxes_cnt = boxes_.size();
        u32 boxes_occurences_cnt = 0;
        for (u32 box_pos=0; box_pos<boxes_.size(); ++box_pos) {
            Box& b = boxes_.accItemAtPos2(box_pos);
            boxes_occurences_cnt += b.getOccurencesCount();
        }

        u32 segs_cnt = 0;
        std::stringstream ss1;
        std::vector<bool> path;
        u32 splits_count[AXES_COUNT];
        memset(splits_count, 0, sizeof(splits_count));

        debugPrintSegmentRec_("Root", path, root_, ss1, segs_cnt, splits_count);
        std::stringstream ss2;
        ss2 << "Boxes=" << boxes_occurences_cnt << "(unique: " << boxes_cnt << ")" << ", Segments=" << segs_cnt << std::endl;
        ss2 << "Splits: ";
        u32 all_splits = 0;
        for (u32 a=0; a<AXES_COUNT; ++a) {
            all_splits += splits_count[a];
            f32 proc = f32(splits_count[a])*100/segs_cnt;
            proc = floorf(proc * 1000.0f) / 1000.0f;
            ss2 << splits_count[a] << "[" << proc << "%] ";
        }
        f32 proc = f32(all_splits)*100/segs_cnt;
        proc = floorf(proc * 1000.0f) / 1000.0f;
        ss2 << ", Leafs: " << all_splits << "[" << proc << "%] " << std::endl;
        ss2 << ss1.str();
        return ss2.str();
    }

#ifdef USE_SDL2
    SMB_TPL
    inline Image::Ref SMB_TYPE::debugImage() {
        ASSERT_M(AXES_COUNT == 2, "Can only visualize 2D");

        f32 bounds[AXES_COUNT*2];
        calcBounds(bounds);

        f32 range[AXES_COUNT] = {bounds[AXES_COUNT] - bounds[0], bounds[AXES_COUNT+1] - bounds[1]};

        f32 aspect = range[0]/range[1];
        u32 image_size[AXES_COUNT];
        image_size[0] = 1024;
        image_size[1] = (u32)std::ceil((f32)image_size[0]/aspect);

        Image::Ref img = new Image(image_size[0], image_size[1], GL_RGBA);
        img->fillWithColor(Color::Black());

        for (u32 box_pos=0; box_pos< boxes_.size(); ++box_pos) {
            Box& b = boxes_.accItemAtPos2(box_pos);
            // convert to image coords
            int lefti = (u32)roundf(((b.getMinValue(0)-bounds[0])/range[0])*image_size[0]);
            int topi = (u32)roundf(((b.getMinValue(1)-bounds[1])/range[1])*image_size[1]);
            int righti = (u32)roundf(((b.getMaxValue(0)-bounds[0])/range[0])*image_size[0]);
            int bottomi = (u32)roundf(((b.getMaxValue(1)-bounds[1])/range[1])*image_size[1]);

            u32 left = (u32)clampToRange(lefti, 0, int(image_size[0]-1));
            u32 right = (u32)clampToRange(righti, 0, int(image_size[0]-1));
            u32 top = (u32)clampToRange(topi, 0, int(image_size[1]-1));
            u32 bottom = (u32)clampToRange(bottomi, 0, int(image_size[1]-1));

            for (u32 x=left; x<=right; ++x) {
                img->setPixel(x, top, Color::Red());
                img->setPixel(x, bottom, Color::Red());
            }
            for (u32 y=top; y<=bottom; ++y) {
                img->setPixel(left, y, Color::Red());
                img->setPixel(right, y, Color::Red());
            }
        }

        fast_vector<Segment*> s{root_};
        while (!s.empty()) {
            Segment* seg = s.back();
            s.pop_back();
            if (seg->isSplit()) {
                s.push_back(seg->getChild(0));
                s.push_back(seg->getChild(1));

                u32 sa = (u32)seg->getSplitAxis();
                u32 pa = (u32)1-sa;

                u32 split_pos = (u32)roundf(((seg->getSplitValue()-bounds[sa])/range[sa])*image_size[sa]);
                u32 low, high;
                f32 low_f, high_f;
                if (seg->getLowBorder(pa, low_f)) {
                    low = (u32)roundf(((low_f-bounds[pa])/range[pa])*image_size[pa]);
                }
                else {
                    low = 0;
                }

                if (seg->getHighBorder(pa, high_f)) {
                    high = (u32)roundf(((high_f-bounds[pa])/range[pa])*image_size[pa]);
                }
                else {
                    high = image_size[pa]-1;
                }

                if (seg->getSplitAxis() == 0) {
                    for (u32 y=low; y<=high; ++y) {
                        img->setPixel(split_pos, y, Color::White());
                    }
                }
                else {
                    for (u32 x=low; x<=high; ++x) {
                        img->setPixel(x, split_pos, Color::White());
                    }
                }

            }
        }

        return img;
    }

    SMB_TPL
    inline void SMB_TYPE::debugRender(SDL_Window* w, SDL_Renderer* r, const f32* bounds, const Color& box_color, const Color& segment_color) {
        if (getBoxesCount() == 0)
            return;

        ASSERT_M(AXES_COUNT == 2, "Can only visualize 2D");

        f32 auto_bounds[AXES_COUNT*2];

        if (!bounds) {
            calcBounds(auto_bounds);
            bounds = auto_bounds;
        }

        f32 range[AXES_COUNT] = {bounds[AXES_COUNT] - bounds[0], bounds[AXES_COUNT+1] - bounds[1]};

        int w_size[AXES_COUNT];
        SDL_GetWindowSize(w, &w_size[0], &w_size[1]);

        for (u32 box_pos=0; box_pos< boxes_.size(); ++box_pos) {
            Box& b = boxes_.accItemAtPos2(box_pos);
            // convert to image coords
            int lefti = (u32)roundf(((b.getMinValue(0)-bounds[0])/range[0])*w_size[0]);
            int topi = (u32)roundf(((b.getMinValue(1)-bounds[1])/range[1])*w_size[1]);
            int righti = (u32)roundf(((b.getMaxValue(0)-bounds[0])/range[0])*w_size[0]);
            int bottomi = (u32)roundf(((b.getMaxValue(1)-bounds[1])/range[1])*w_size[1]);

            u32 left = (u32)clampToRange(lefti, 0, int(w_size[0]-1));
            u32 right = (u32)clampToRange(righti, 0, int(w_size[0]-1));
            u32 top = (u32)clampToRange(topi, 0, int(w_size[1]-1));
            u32 bottom = (u32)clampToRange(bottomi, 0, int(w_size[1]-1));

            SDL_SetRenderDrawColor(r, box_color.r, box_color.g, box_color.b, box_color.a);
            SDL_Rect rect;
            rect.x = left;
            rect.y = top;
            rect.w = right-left;
            rect.h = bottom-top;
            SDL_RenderDrawRect(r, &rect);
        }

        SDL_SetRenderDrawColor(r, segment_color.r, segment_color.g, segment_color.b, segment_color.a);
        fast_vector<Segment*> s{root_};
        while (!s.empty()) {
            Segment* seg = s.back();
            s.pop_back();
            if (seg->isSplit()) {
                s.push_back(seg->getChild(0));
                s.push_back(seg->getChild(1));

                u32 sa = (u32)seg->getSplitAxis();
                u32 pa = (u32)1-sa;

                u32 split_pos = (u32)roundf(((seg->getSplitValue()-bounds[sa])/range[sa])*w_size[sa]);
                u32 low, high;
                f32 low_f, high_f;
                if (seg->getLowBorder(pa, low_f)) {
                    low = (u32)roundf(((low_f-bounds[pa])/range[pa])*w_size[pa]);
                }
                else {
                    low = 0;
                }

                if (seg->getHighBorder(pa, high_f)) {
                    high = (u32)roundf(((high_f-bounds[pa])/range[pa])*w_size[pa]);
                }
                else {
                    high = (u32)w_size[pa]-1;
                }

                if (seg->getSplitAxis() == 0) {
                    SDL_RenderDrawLine(r, split_pos, low, split_pos, high);
                }
                else {
                    SDL_RenderDrawLine(r, low, split_pos, high, split_pos);
                }
            }
        }
    }
#endif

    SMB_TPL
    inline void SMB_TYPE::clear() {
        delete root_;
        boxes_.clear();
        overlaps_.pm.clear();
        root_ = new Segment(*this, NULL);
        root_->setDebugName_();
        root_->calcBorders_();
    }

    SMB_TPL
    template <typename Derived>
    inline typename SMB_TYPE::Box& SMB_TYPE::addBoxInner_(Index& box_id_out, f32* bounds, const BoxDataT& box_data) {
#ifdef DEBUG_BUILD
        for (u32 a=0; a<AXES_COUNT; ++a) {
            ASSERT(GET_MIN(bounds, a) <= GET_MAX(bounds, a));
        }
#endif
        Box& new_box  = boxes_.add2(box_id_out);
        memcpy(new_box.bounds_, bounds, AXES_COUNT*2*sizeof(f32));
        new_box.setClientData(box_data);


        root_->addBoxTree_(bounds, [box_id_out, &new_box] (Segment* seg) {
            seg->addBox(new_box, box_id_out);
        });

        addOverlaps_<Derived>(new_box, box_id_out);
        ASSERT(overlaps_.removed_.empty());

#ifdef SAP_VALIDATE_ALL_THE_TIME
        validate();
#endif
        return new_box;
    }

    SMB_TPL
    template <typename Derived>
    inline void SMB_TYPE::updateBoxInner_(Index box_id, f32* bounds) {
#ifdef DEBUG_BUILD
        for (u32 a=0; a<AXES_COUNT; ++a) {
            ASSERT(GET_MIN(bounds, a) <= GET_MAX(bounds, a));
        }
#endif
        Box& b = boxes_.accItem(box_id);
        memcpy(b.bounds_, bounds, AXES_COUNT*2*sizeof(f32));

        for (u32 i=0; i<b.getOccurencesCount(); ++i) {
            bool oos = b.getOccurence(i).segment_->updateBox(b, box_id, b.getOccurence(i).min_max_ids_, deferred_after_update_);
            if (oos)
                --i;
        }

        afterUpdate_<Derived>(b, box_id, bounds);
    }

    SMB_TPL
    template <typename Derived>
    inline void SMB_TYPE::moveBoxInner_(Index box_id, f32* move_vec) {
        Box& b = boxes_.accItem(box_id);

        for (u32 a=0; a<AXES_COUNT; ++a) {
            GET_MIN(b.bounds_, a) += move_vec[a];
            GET_MAX(b.bounds_, a) += move_vec[a];
        }

        ASSERT(b.getOccurencesCount());
        for (u32 i=0; i<b.getOccurencesCount(); ++i) {
            bool oos = b.getOccurence(i).segment_->moveBox(b, box_id, b.getOccurence(i).min_max_ids_, move_vec, deferred_after_update_);
            if (oos)
                --i;
        }
        afterUpdate_<Derived>(b, box_id, b.getBounds());
    }

    SMB_TPL
    template <typename Derived>
    inline void SMB_TYPE::removeBoxInner_(Index box_id) {
        Box& b = boxes_.accItem(box_id);

        for (i32 i = b.getOccurencesCount()-1; i>=0; --i) {
            Segment* seg = b.getOccurence(i).segment_;
            seg->removeBox(b, box_id, b.getOccurence(i).min_max_ids_);
        }

        removeOverlaps_<Derived>(b, box_id);
        ASSERT(overlaps_.possibly_added_.empty());

        boxes_.removeItem(box_id);
#ifdef SAP_VALIDATE_ALL_THE_TIME
        validate();
#endif
    }

    SMB_TPL
    template <typename Derived>
    inline void SMB_TYPE::afterUpdate_(Box& box, Index box_id, const f32* bounds) {
        PROFILE_BLOCK("afterUpdate");

        if (deferred_after_update_.crossing_) {
            PROFILE_BLOCK("crossing");
            root_->addBoxTree_(bounds, [&box, box_id, bounds] (Segment* seg) {
                if (box.findOccurence(seg) == InvalidId()) {
                    seg->addBox(box, box_id);
                }
            });
            deferred_after_update_.crossing_ = false;
        }

        if (!deferred_after_update_.merges_.empty()) {
            PROFILE_BLOCK("merging");
            for (u32 i=0; i<deferred_after_update_.merges_.size(); ++i) {
                Segment* s = deferred_after_update_.merges_[i];
                // merge will destroy this and neighboring segment -> remove neighboring segment from looped occurences if it is also scheduled for merge
                Segment* sn = s->getSplitNeighbor();
                for (u32 j=i+1; j<deferred_after_update_.merges_.size(); ++j) {
                    if (deferred_after_update_.merges_[j] == sn) {
                        deferred_after_update_.merges_[j] = deferred_after_update_.merges_.back();
                        deferred_after_update_.merges_.pop_back();
                        break;
                    }
                }

                s->parent_->merge_(s);
            }

            deferred_after_update_.merges_.clear();
        }

        {
            PROFILE_BLOCK("Overlaps add/remove");
            removeOverlaps_<Derived>(box, box_id);
            addOverlaps_<Derived>(box, box_id);
        }

#ifdef SAP_VALIDATE_ALL_THE_TIME
        validate();
#endif
    }

    SMB_TPL
    template <typename Derived>
    inline void SMB_TYPE::addOverlaps_(Box& box, Index box_id) {
        for (u32 i=0; i<overlaps_.possibly_added_.size(); ++i) {
            u32 b2_inner_id = overlaps_.possibly_added_[i];
            ASSERT(b2_inner_id != box_id.getIndex());
            Box& b2 = boxes_.accItemWithInnerIndex(b2_inner_id);
            if (boxesOverlap_(box, b2) && getAs_<Derived>().beforeBoxesOverlap_(box, b2)) {
                bool was_added;
                OverlapDataT* cl_data = overlaps_.pm.findOrAddItem(SAP::CollPair(box_id.getIndex(), b2_inner_id), was_added);
                if (was_added) {
                    new (cl_data) OverlapDataT();
                }
            }
        }
        overlaps_.possibly_added_.clear();
    }

    SMB_TPL
    template <typename Derived>
    inline void SMB_TYPE::removeOverlaps_(Box& box, Index box_id) {
        for (u32 i=0; i<overlaps_.removed_.size(); ++i) {
            u32 b2_inner_id = overlaps_.removed_[i];
            ASSERT(b2_inner_id != box_id.getIndex());
            overlaps_.pm.removeItem(SAP::CollPair(box_id.getIndex(), b2_inner_id), [&]() {
                Box& b2 = boxes_.accItemWithInnerIndex(b2_inner_id);
                getAs_<Derived>().afterBoxesOverlap_(box, b2);
            });
        }
        overlaps_.removed_.clear();
    }

    SMB_TPL
    inline void SMB_TYPE::debugPrintSegmentRec_(const std::string& name, std::vector<bool>& path, Segment* s, std::ostream& os, u32& segs_cnt, u32* splits_count) {
        ++segs_cnt;
        std::string ind;
        ind.reserve(path.size()*2);
        for (u32 i=0; i<path.size(); ++i) {
            if (!path[i])
                ind += "| ";
            else
                ind += "  ";
        }
        std::string ind2 = ind;
        if (ind2.size() >= 2) {
            ind2[ind2.size()-1] = '-';
            ind2[ind2.size()-2] = '+';
        }

        if (s->split_axis_ != SAP::InvalidAxis) {
            f32 low, high;

            if (!s->getLowBorder(s->getSplitAxis(), low)) {
                low = s->findLowestPointRec_(s->getSplitAxis());
            }
            if (!s->getHighBorder(s->getSplitAxis(), high)) {
                high = s->findHighestPointRec_(s->getSplitAxis());
            }
            f32 split_val_perc = 100*(s->split_value_ - low)/(high-low);
            ++splits_count[s->split_axis_];
            fast_vector<u32> crossed;
            s->getCrossedBoxes(crossed);
            os << ind2 << name << ":"  << " Split, A: " << (i32)s->split_axis_ << ", V: " << s->split_value_
               << "(" << split_val_perc << "%), crossed: " << crossed.size() << std::endl;
            path.push_back(0);
            debugPrintSegmentRec_("Child 0 [" + s->children_[0]->getDebugName() + "]", path, s->children_[0], os, segs_cnt, splits_count);
            path.pop_back();
            path.push_back(1);
            debugPrintSegmentRec_("Child 1 [" + s->children_[1]->getDebugName() + "]", path, s->children_[1], os, segs_cnt, splits_count);
            path.pop_back();
        }
        else {
            os << ind2 << name << ":" << " Leaf, boxes: " << s->getBoxesCount() << std::endl;
        }
    }

    SM_TPL
    inline typename SM_TYPE::Box& SM_TYPE::addBox(Index& box_id_out, f32* bounds, const BoxDataT& box_data) {
        return this->template addBoxInner_<Derived>(box_id_out, bounds, box_data);
    }

    SM_TPL
    inline void SM_TYPE::updateBox(Index box_id, f32* bounds) {
        this->template updateBoxInner_<Derived>(box_id, bounds);
    }

    SM_TPL
    inline void SM_TYPE::moveBox(Index box_id, f32* move_vec) {
        this->template moveBoxInner_<Derived>(box_id, move_vec);
    }

    SM_TPL
    inline void SM_TYPE::removeBox(Index box_id) {
        this->template removeBoxInner_<Derived>(box_id);
    }
}

#undef SMB_TPL
#undef SMB_TYPE
#undef SM_TPL
#undef SM_TYPE
#undef GET_MIN
#undef GET_MAX