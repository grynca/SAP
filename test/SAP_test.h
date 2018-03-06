#ifndef SAP_TEST_H
#define SAP_TEST_H

//#define FRAMES 100
//#define CONSTANT_DT 1.0f/30
//#define SAP_VALIDATE_ALL_THE_TIME

#include "SAP.h"
#include "maths.h"

typedef SAPDomain2D<int> MySAPDomain;

//class MySAPManager : public SAPManagerC<MySAPManager, MySAPDomain> {
//    typedef SAPManagerC<MySAPManager, MySAPDomain> Base;
//private:
//    friend Base;
//
//    bool beforeBoxesOverlap_(Base::Box&, Base::Box&) { std::cout << "overlap begin" << std::endl; return true; }
//    void afterBoxesOverlap_(Base::Box&, Base::Box&) { std::cout << "overlap end" << std::endl; }
//};

class SAPTestFixture : public SDLTest {
public:
    SAPManagerSimple2D<MySAPDomain> sap;
    u32 boxes_count;
    i32 space_size;
    i32 box_size_max;
    f32 speed_min;
    f32 speed_max;
    fast_vector<Index> box_ids;

    bool mouse_down, all_ray_hits;
    i32 ray_start_x, ray_start_y, ray_end_x, ray_end_y;
    u32 ray_overlaps;
    u32 frame;



    void init(Config::ConfigSectionMap& cfg) {
        boxes_count = u32(loadCfgValue(cfg, "boxes_count", 2000));
        space_size = loadCfgValue(cfg, "space_size", 10000);
        box_size_max = loadCfgValue(cfg, "box_size_max", 500);
        speed_min = loadCfgValue(cfg, "speed_min", 0.05f);
        speed_max = loadCfgValue(cfg, "speed_max", 0.5f);

        {
            grynca::BlockMeasure m("Creation");
            box_ids.reserve(boxes_count);
            f32 bounds[6];
            for (u32 i=0; i< boxes_count; ++i) {
                bounds[0] = ((f32)rand()/RAND_MAX)*space_size;
                bounds[1] = ((f32)rand()/RAND_MAX)*space_size;
                bounds[2] = bounds[0] + ((f32)rand()/RAND_MAX)*box_size_max + 0.1f;
                bounds[3] = bounds[1] + ((f32)rand()/RAND_MAX)*box_size_max + 0.1f;
                //std::cout << "adding box: [" << e[0].min << ", " << e[1].min<< "], [" << e[0].max << ", " << e[1].max << "]" << std::endl;

                Index box_id;
                sap.addBox(box_id, bounds, i);
                box_ids.push_back(box_id);
            }
        }

        frame = 0;
        mouse_down = false;
        ray_start_x = ray_start_y = ray_end_x = ray_end_y = 0;
        ray_overlaps = 0;
        all_ray_hits = true;
        speed_min = space_size*speed_min;
        speed_min *= randFloat()*2 - 1;
        speed_max = space_size*speed_max;
        speed_max *= randFloat()*2 - 1;
    }

    void close() {
        fast_vector<u32> destruction_order;
        destruction_order.reserve(boxes_count);
        randomPickN(destruction_order, boxes_count, boxes_count);
        {
            grynca::BlockMeasure m("Removal");
            for (u32 i=0; i<destruction_order.size(); ++i) {
                Index box_id = box_ids[destruction_order[i]];
                sap.removeBox(box_id);
            }
        }
    }

    void handleEvent(SDL_Event& evt) {
        switch (evt.type) {
            case (SDL_KEYDOWN): {
                switch (evt.key.keysym.sym) {
                    case (SDLK_r): {
                        all_ray_hits = !all_ray_hits;
                    }break;
                }
            }break;
            case (SDL_MOUSEBUTTONDOWN): {
                if (evt.button.button == SDL_BUTTON_LEFT) {
                    mouse_down = true;
                    ray_start_x = evt.button.x;
                    ray_start_y = evt.button.y;
                    ray_end_x = evt.button.x;
                    ray_end_y = evt.button.y;
                }
            }break;
            case (SDL_MOUSEBUTTONUP): {
                if (evt.button.button == SDL_BUTTON_LEFT && mouse_down) {
                    ray_end_x = evt.button.x;
                    ray_end_y = evt.button.y;
                    mouse_down = false;
                }
            }break;
            case (SDL_MOUSEMOTION): {
                if (mouse_down) {
                    ray_end_x = evt.button.x;
                    ray_end_y = evt.button.y;
                }
            }break;
        }
    }

    void update(SDL_Renderer* r, f32 dt) {
#ifdef CONSTANT_DT
        dt = CONSTANT_DT;
#endif
        for (u32 i = 0; i<box_ids.size(); ++i) {
            PROFILE_BLOCK("SAP update");
            f32 move_vec[2];
            move_vec[0] = randFloat(speed_min, speed_max)*dt;
            move_vec[1] = randFloat(speed_min, speed_max)*dt;
            sap.moveBox(box_ids[i], move_vec);
        }

#if 1
        {
            PROFILE_BLOCK("Draw");
            SDL_SetRenderDrawColor(r, 0, 0, 0, 255);
            SDL_RenderClear(r);
            SDL_Window* w = accTestBench().getWindow();
            sap.debugRender(w, r);
            ray_overlaps = 0;
            int win_width, win_height;
            SDL_GetRendererOutputSize(r, &win_width, &win_height);
            std::string info = ssu::formatA("boxes: %u\noverlapping pairs: %u", sap.getBoxesCount(), sap.getOverlapsCount());
            if ((ray_end_x-ray_start_x)!=0 || (ray_end_y-ray_start_y)!=0) {
                SDL_SetRenderDrawColor(r, Color::Green().r, Color::Green().g, Color::Green().b, Color::Green().a);
                SDL_RenderDrawLine(r, ray_start_x, ray_start_y, ray_end_x, ray_end_y);

                f32 bounds[2*2];
                sap.calcBounds(bounds);
                f32 range[2] = {bounds[2] - bounds[0], bounds[2+1] - bounds[1]};

                // convert ray to world coords
                f32 sx = range[0]/win_width;
                f32 sy = range[1]/win_height;

                f32 ray_start[2] = {ray_start_x*sx +bounds[0], ray_start_y*sy +bounds[1]};
                f32 ray_end[2] = {ray_end_x*sx +bounds[0], ray_end_y*sy +bounds[1]};
                f32 ray_dir[2] = {ray_end[0]-ray_start[0], ray_end[1]-ray_start[1]};
                {
                    PROFILE_BLOCK("raycast");
                    auto rc = sap.getRayCaster();
                    rc.setRay(ray_start , ray_dir);
                    rc.getHits([this](u32 bid, f32 t) {
                        ++ray_overlaps;
                        return all_ray_hits;
                    });
                }
                info += ssu::formatA("\nray_overlaps: %u", ray_overlaps);
            }
            F8x8::SDL2Text info_lbl(r, info);
            info_lbl.setColor(255, 255, 255, 255);
            info_lbl.draw(win_width - 5 - info_lbl.getWidth(), 5);
        }
#endif
        ++frame;
    }
};

#endif //SAP_TEST_H
