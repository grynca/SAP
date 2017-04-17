#include "SAPManager.h"
#include "base.h"
#include <algorithm>
#include <cassert>
#include <sstream>

#define SM_TPL template <typename ClientData, u32 AXES_COUNT>
#define SM_TYPE SAPManager<ClientData, AXES_COUNT>
#define GET_MIN(BOUNDS, AXIS) bounds[AXIS]
#define GET_MAX(BOUNDS, AXIS) bounds[AXES_COUNT+AXIS]

namespace grynca {

    SM_TPL
    inline SM_TYPE::SAPManager()
     : root_(new Segment(*this, NULL)),
       boxes_(sizeof(Box))
    {
        root_->setDebugName_();
        root_->calcBorders_();
    }

    SM_TPL
    inline SM_TYPE::~SAPManager() {
        delete root_;
    }

    SM_TPL
    inline u32 SM_TYPE::addBox(f32* bounds, const ClientData& client_data) {

#ifdef DEBUG_BUILD
        for (u32 a=0; a<AXES_COUNT; ++a) {
            ASSERT(GET_MIN(bounds, a) <= GET_MAX(bounds, a));
        }
#endif
        u8* place;
        u32 new_box_id = boxes_.add(place).getIndex();
        Box* new_box = new (place) Box();;
        new_box->setClientData(client_data);

        fast_vector<Segment*> segs{root_};
        while (!segs.empty()) {
            Segment* s = segs.back();
            segs.pop_back();

            if (!s->isSplit()) {
                s->addBox(new_box, new_box_id, bounds);
            }
            else if (GET_MAX(bounds, s->getSplitAxis()) < s->getSplitValue()) {
                // put to first
                segs.push_back(s->getChild(0));
            }
            else if (GET_MIN(bounds, s->getSplitAxis()) > s->getSplitValue()) {
                // put to second
                segs.push_back(s->getChild(1));
            }
            else {
                // put to both
                segs.push_back(s->getChild(0));
                segs.push_back(s->getChild(1));
            }
        }

        for (u32 i=0; i<overlaps_.possibly_added_.size(); ++i) {
            u32 b2_id = overlaps_.possibly_added_[i];
            ASSERT(b2_id != new_box_id);
            Box* b2 = getBox_(b2_id);
            if (boxesOverlap_(*new_box, *b2)) {
                overlaps_.pm.addItem(SAP::CollPair(new_box_id, b2_id));
            }
        }
        overlaps_.possibly_added_.clear();
        ASSERT(overlaps_.removed_.empty());

#ifdef VALIDATE_ALL_THE_TIME
        validate();
#endif
        return new_box_id;
    }

    SM_TPL
    inline void SM_TYPE::updateBox(u32 box_id, f32* bounds) {
#ifdef DEBUG_BUILD
        for (u32 a=0; a<AXES_COUNT; ++a) {
            ASSERT(GET_MIN(bounds, a) <= GET_MAX(bounds, a));
        }
#endif
        Box* b = getBox_(box_id);
        ASSERT(b);

        for (u32 i=0; i<b->getOccurencesCount(); ++i) {
            bool oos = b->getOccurence(i).segment_->updateBox(b, box_id, b->getOccurence(i).min_max_ids_, bounds, deferred_after_update_);
            if (oos)
                --i;
        }

        afterUpdate_(b, box_id, bounds);
    }

    SM_TPL
    inline void SM_TYPE::moveBox(u32 box_id, f32* move_vec) {
        Box* b = getBox_(box_id);
        ASSERT(b);
        ASSERT(b->getOccurencesCount());

        typename Box::Occurence& occ = b->getOccurence(0);
        f32 bounds[AXES_COUNT*2];
        for (u32 a=0; a<AXES_COUNT; ++a) {
            SAP::MinMax min_max_id = occ.min_max_ids_[a];
            GET_MIN(bounds, a) = occ.segment_->points_[a][min_max_id.v[0]].getValue() + move_vec[a];
            GET_MAX(bounds, a) = occ.segment_->points_[a][min_max_id.v[1]].getValue() + move_vec[a];
        }

        for (u32 i=0; i<b->getOccurencesCount(); ++i) {
            bool oos = b->getOccurence(i).segment_->moveBox(b, box_id, b->getOccurence(i).min_max_ids_, bounds, move_vec, deferred_after_update_);
            if (oos)
                --i;
        }

        afterUpdate_(b, box_id, bounds);
    }

    SM_TPL
    inline void SM_TYPE::removeBox(u32 box_id) {

        Box* b = getBox_(box_id);

        for (i32 i = b->getOccurencesCount()-1; i>=0; --i) {
            Segment* seg = b->getOccurence(i).segment_;
            seg->removeBox(b, box_id, b->getOccurence(i).min_max_ids_);
        }

        for (u32 i=0; i<overlaps_.removed_.size(); ++i) {
            u32 b2_id = overlaps_.removed_[i];
            ASSERT(b2_id != box_id);
            overlaps_.pm.removeItem(SAP::CollPair(box_id, b2_id));
        }
        overlaps_.removed_.clear();
        ASSERT(overlaps_.possibly_added_.empty());

        boxes_.removeAtPos(box_id, Type<Box>::destroy);
#ifdef VALIDATE_ALL_THE_TIME
        validate();
#endif
    }

    SM_TPL
    inline void SM_TYPE::getBox(u32 box_id, f32* bounds) {
        Box* b = getBox_(box_id);
        for (u32 a=0; a<AXES_COUNT; ++a) {
            b->getMinMaxValue(a, GET_MIN(bounds, a), GET_MAX(bounds, a));
        }
    }

    SM_TPL
    inline bool SM_TYPE::tryGetBox(u32 box_id, f32* bounds) {
        Box* b = getBox_(box_id);
        if (!b)
            return false;
        for (u32 a=0; a<AXES_COUNT; ++a) {
            b->getMinMaxValue(a, GET_MAX(bounds, a), GET_MIN(bounds, a));
        }
        return true;
    }

    SM_TPL
    inline u32 SM_TYPE::getBoxesCount() {
        return boxes_.calcOccupiedSize();
    }

    SM_TPL
    inline u32 SM_TYPE::getBoxesPoolSize() {
        return boxes_.size();
    }

    SM_TPL
    inline typename SM_TYPE::Segment* SM_TYPE::getRootSegment() {
        return root_;
    }

    SM_TPL
    inline void SM_TYPE::validate() {
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
        for (u32 box_id=0; box_id<boxes_.size(); ++box_id) {
            Box* b = getBox_(box_id);
            if (!b) {
                // check that no endpoint points to non-existant box id
                fast_vector<Segment*> s{root_};
                while (!s.empty()) {
                    Segment* seg = s.back();
                    s.pop_back();

                    if (seg->isSplit()) {
                        s.push_back(seg->getChild(0));
                        s.push_back(seg->getChild(1));
                    }

                    for (u32 a=0; a<AXES_COUNT; ++a) {
                        for (u32 i=0; i < seg->points_[a].size(); ++i) {
                            ASSERT(seg->points_[a][i].getBoxId() != box_id);
                        }
                    }
                }
            }
            else {
                ASSERT(b->getOccurencesCount());
                boxes_cnt += b->getOccurencesCount();
                for (u32 i=0; i<b->getOccurencesCount(); ++i) {
                    Segment* seg = b->getOccurence(i).segment_;

                    ASSERT(!seg->isSplit());

                    for (u32 a=0; a<AXES_COUNT; ++a) {
                        u32 min_id = b->getOccurence(i).min_max_ids_[a].v[0];
                        u32 max_id = b->getOccurence(i).min_max_ids_[a].v[1];
                        ASSERT(min_id < max_id);
                        ASSERT(min_id < seg->points_[a].size());
                        ASSERT(max_id < seg->points_[a].size());
                        ASSERT(seg->points_[a][min_id].getBoxId() == box_id);
                        ASSERT(seg->points_[a][max_id].getBoxId() == box_id);
                    }
                }
            }
        }
#ifdef VALIDATE_OVERLAPS
        for (u32 b1_id=0; b1_id<boxes_.size(); ++b1_id) {
            Box* b1 = getBox_(b1_id);
            if (!b1)
                continue;
            for (u32 b2_id=b1_id+1; b2_id<boxes_.size(); ++b2_id) {
                Box* b2 = getBox_(b2_id);
                if (!b2)
                    continue;
                bool overlaps_in_sap = bool(overlaps_.pm.findItem(SAP::CollPair(b1_id, b2_id)));
                bool overlaps_ground_truth = boxesOverlap_(*b1, *b2);
                if (overlaps_in_sap != overlaps_ground_truth) {
                    overlaps_ground_truth = boxesOverlap_(*b1, *b2);
                }
                ASSERT(overlaps_in_sap == overlaps_ground_truth);
            }
        }
#endif
    }

    SM_TPL
    inline u32 SM_TYPE::getOverlapsCount() {
        return overlaps_.pm.getItemsCount();
    }

    SM_TPL
    inline void SM_TYPE::getOverlap(u32 overlap_id, u32& b1_id_out, u32& b2_id_out) {
        const SAP::CollPair& p = overlaps_.pm.getItem(overlap_id);
        b1_id_out = p.id1;
        b2_id_out = p.id2;
    }

    SM_TPL
    inline SAPRaycaster<ClientData, AXES_COUNT> SM_TYPE::getRayCaster() {
        return Raycaster(*this);
    }

    SM_TPL
    inline void SM_TYPE::calcBounds(f32* bounds) {
        ASSERT(getBoxesCount());
        for (u32 a=0; a<AXES_COUNT; ++a) {
            bounds[a] = root_->findLowestPointRec_(a);
            bounds[AXES_COUNT+a] = root_->findHighestPointRec_(a);
        }
    }

    SM_TPL
    inline ClientData& SM_TYPE::getClientData(u32 box_id) {
        return getBox_(box_id)->getClientData();
    }

    SM_TPL
    inline u32 SM_TYPE::getGroupId(u32 box_id) {
        Box* b = getBox_(box_id);
        return b->getCollisionGroup();
    }

    SM_TPL
    inline typename SM_TYPE::Box* SM_TYPE::getBox_(u32 box_id) {
        u8* ptr = boxes_.getAtPos(box_id);
        return (Box*)ptr;
    }

    SM_TPL
    inline bool SM_TYPE::boxesOverlap_(Box& b1, Box& b2) {
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

    SM_TPL
    inline std::string SM_TYPE::debugPrint() {
        u32 boxes_cnt = boxes_.calcOccupiedSize();
        u32 boxes_occurences_cnt = 0;
        for (u32 i=0; i<boxes_.size(); ++i) {
            Box* b = getBox_(i);
            if (b) {
                boxes_occurences_cnt += b->getOccurencesCount();
            }
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
    SM_TPL
    inline Image::Ref SM_TYPE::debugImage() {
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

        for (u32 i=0; i< boxes_.size(); ++i) {
            Box *b = getBox_(i);
            if (!b)
                continue;

            // convert to image coords
            int lefti = (u32)roundf(((b->getMinValue(0)-bounds[0])/range[0])*image_size[0]);
            int topi = (u32)roundf(((b->getMinValue(1)-bounds[1])/range[1])*image_size[1]);
            int righti = (u32)roundf(((b->getMaxValue(0)-bounds[0])/range[0])*image_size[0]);
            int bottomi = (u32)roundf(((b->getMaxValue(1)-bounds[1])/range[1])*image_size[1]);

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

    SM_TPL
    inline void SM_TYPE::debugRender(SDL_Window* w, SDL_Renderer* r) {
        ASSERT_M(AXES_COUNT == 2, "Can only visualize 2D");

        f32 bounds[AXES_COUNT*2];
        calcBounds(bounds);
        f32 range[AXES_COUNT] = {bounds[AXES_COUNT] - bounds[0], bounds[AXES_COUNT+1] - bounds[1]};

        int w_size[AXES_COUNT];
        SDL_GetWindowSize(w, &w_size[0], &w_size[1]);

        for (u32 i=0; i< boxes_.size(); ++i) {
            Box *b = getBox_(i);
            if (!b)
                continue;

            // convert to image coords
            int lefti = (u32)roundf(((b->getMinValue(0)-bounds[0])/range[0])*w_size[0]);
            int topi = (u32)roundf(((b->getMinValue(1)-bounds[1])/range[1])*w_size[1]);
            int righti = (u32)roundf(((b->getMaxValue(0)-bounds[0])/range[0])*w_size[0]);
            int bottomi = (u32)roundf(((b->getMaxValue(1)-bounds[1])/range[1])*w_size[1]);

            u32 left = (u32)clampToRange(lefti, 0, int(w_size[0]-1));
            u32 right = (u32)clampToRange(righti, 0, int(w_size[0]-1));
            u32 top = (u32)clampToRange(topi, 0, int(w_size[1]-1));
            u32 bottom = (u32)clampToRange(bottomi, 0, int(w_size[1]-1));

            Color box_clr = Color::Red();
            SDL_SetRenderDrawColor(r, box_clr.r, box_clr.g, box_clr.b, box_clr.a);
            SDL_Rect rect;
            rect.x = left;
            rect.y = top;
            rect.w = right-left;
            rect.h = bottom-top;
            SDL_RenderDrawRect(r, &rect);
        }

        SDL_SetRenderDrawColor(r, Color::White().r, Color::White().g, Color::White().b, Color::White().a);
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

    SM_TPL
    inline void SM_TYPE::debugPrintSegmentRec_(const std::string& name, std::vector<bool>& path, Segment* s, std::ostream& os, u32& segs_cnt, u32* splits_count) {
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
    inline void SM_TYPE::afterUpdate_(Box* box, u32 box_id, f32* bounds) {
        bool deferred_work = !deferred_after_update_.crossings_.empty() || !deferred_after_update_.merges_.empty();
        if (deferred_work) {
            for (u32 i=0; i<deferred_after_update_.crossings_.size(); ++i) {
                if (box->findOccurence(deferred_after_update_.crossings_[i]) == InvalidId()) {
                    deferred_after_update_.crossings_[i]->addBox(box, box_id, bounds);
                }
            }
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

            deferred_after_update_.crossings_.clear();
            deferred_after_update_.merges_.clear();
        }

        for (u32 i=0; i<overlaps_.removed_.size(); ++i) {
            u32 b2_id = overlaps_.removed_[i];
            ASSERT(b2_id != box_id);
            overlaps_.pm.removeItem(SAP::CollPair(box_id, b2_id));
        }
        for (u32 i=0; i<overlaps_.possibly_added_.size(); ++i) {
            u32 b2_id = overlaps_.possibly_added_[i];
            ASSERT(b2_id != box_id);
            Box* b2 = getBox_(b2_id);
            if (boxesOverlap_(*box, *b2)) {
                overlaps_.pm.addItem(SAP::CollPair(box_id, b2_id));
            }
        }
        overlaps_.possibly_added_.clear();
        overlaps_.removed_.clear();

#ifdef VALIDATE_ALL_THE_TIME
        validate();
#endif
    }
}

#undef SM_TPL
#undef SM_TYPE
#undef NEW_SEGMENT
#undef GET_MIN
#undef GET_MAX