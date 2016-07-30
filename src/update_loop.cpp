#include "base.h"
#include "update_loop.h"
#include "test_cfg.h"
#include "SDL2/SDL.h"
using namespace grynca;

#ifdef VISUAL_UPDATE
#   define WIDTH 1024
#   define HEIGHT 768
#endif

#ifdef WEB
#include <emscripten/emscripten.h>
    static void dispatch_main(void* fp) {
        std::function<void()>* func = (std::function<void()>*)fp;
        (*func)();
    }
#endif

void start_update_loop(SAPManager2D<int>& sap, float space_size) {
#ifdef VISUAL_UPDATE
    SDL_Window *win = SDL_CreateWindow("SAP Updating", 100, 100, WIDTH, HEIGHT, 0);
    SDL_Renderer *renderer = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    SDL_Rect texr;
    texr.x = 0; texr.y = 0; texr.w = WIDTH; texr.h = HEIGHT;

    bool mouse_down = false;
    int ray_start_x = 0, ray_start_y = 0;
    int ray_end_x = 0, ray_end_y = 0;
    uint32_t ray_overlaps = 0;
    float raycast_dt = 0.0f;
    bool all_ray_hits = true;
#endif
    float min_speed = space_size*(float)SPEED_MIN;
    min_speed *= randFloat()*2 - 1;
    float max_speed = space_size*(float)SPEED_MAX;
    max_speed *= randFloat()*2 - 1;

    // main loop
    bool exit = false;
    uint32_t frame_id = 0;
    grynca::Measure m("update");

    std::function<void()> main_loop = [&]() {
        m.incCounter();
#ifdef CONSTANT_DT
        float dt = CONSTANT_DT;
#else
        float dt = m.calcDt();
#endif
        if (m.getCounter() == 50) {
            m.print();
            std::cout << " overlapping boxes:: " << sap.getOverlapsCount() << std::endl;
            if ((ray_end_x-ray_start_x)!=0 || (ray_end_y-ray_start_y)!=0) {
                std::cout << " raycast : " << raycast_dt << "sec (overlaps: " << ray_overlaps << ")" << std::endl;
            }
            m.reset();
        }

#ifdef VISUAL_UPDATE
        SDL_Event evt;
        if ( SDL_PollEvent(&evt) ) {
            switch (evt.type) {
                case (SDL_QUIT): {
                    exit = true;
                }break;
                case (SDL_KEYDOWN): {
                    switch (evt.key.keysym.sym) {
                        case (SDLK_r): {
                            all_ray_hits = !all_ray_hits;
                        }break;
                    }
                }break;
                case (SDL_KEYUP): {
                    switch (evt.key.keysym.sym) {
                        case (SDLK_ESCAPE): {
                            exit = true;
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
#endif

        for (uint32_t bid=0; bid<sap.getBoxesPoolSize(); ++bid) {

            float move_vec[2];
            move_vec[0] = randFloatMinMax(min_speed, max_speed)*dt;
            move_vec[1] = randFloatMinMax(min_speed, max_speed)*dt;
            sap.moveBox(bid, move_vec);
        }


//        for (uint32_t bid=0; bid<sap.getBoxesPoolSize(); ++bid) {
//            SAP::Extent e[2];
//            if (!sap.tryGetBox(bid, e))
//                continue;
//
//            float dx = randFloatMinMax(min_speed, max_speed)*dt;
//            float dy = randFloatMinMax(min_speed, max_speed)*dt;
//            //float dz = ((float)rand()/RAND_MAX)*0.5f;
//            e[0].min +=dx;
//            e[0].max +=dx;
//            e[1].min +=dy;
//            e[1].max +=dy;
////            e[2].min +=dz;
////            e[2].max +=dz;
//            sap.updateBox(bid, e);
//            //std::cout << " updated " << bid << std::endl;
//        }

#ifdef VISUAL_UPDATE
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        sap.debugRender(win, renderer);
        ray_overlaps = 0;
        if ((ray_end_x-ray_start_x)!=0 || (ray_end_y-ray_start_y)!=0) {
            SDL_SetRenderDrawColor(renderer, Color::Green().r, Color::Green().g, Color::Green().b, Color::Green().a);
            SDL_RenderDrawLine(renderer, ray_start_x, ray_start_y, ray_end_x, ray_end_y);


            float bounds[2*2];
            sap.calcBounds(bounds);
            float range[2] = {bounds[2] - bounds[0], bounds[2+1] - bounds[1]};

            // convert ray to world coords
            float sx = range[0]/WIDTH;
            float sy = range[1]/HEIGHT;

            float ray_start[2] = {ray_start_x*sx +bounds[0], ray_start_y*sy +bounds[1]};
            float ray_end[2] = {ray_end_x*sx +bounds[0], ray_end_y*sy +bounds[1]};
            float ray_dir[2] = {ray_end[0]-ray_start[0], ray_end[1]-ray_start[1]};

            grynca::Measure m2("raycast");
            auto rc = sap.getRayCaster();
            rc.setRay(ray_start , ray_dir);
            rc.getHits([&ray_overlaps, &all_ray_hits](uint32_t bid, float t) {
                ++ray_overlaps;
                return all_ray_hits;
            });

            m2.incCounter();
            raycast_dt = m2.calcDt();
//
//            int rsx = ((b->getMinValue(0)-bounds[0])/range[0])*image_size[0];
//            int rsy = ((b->getMinValue(1)-bounds[1])/range[1])*image_size[1];
//            int rex = ((b->getMaxValue(0)-bounds[0])/range[0])*image_size[0];
//            int rey = ((b->getMaxValue(1)-bounds[1])/range[1])*image_size[1];
//
//            uint32_t left = (uint32_t)clampToRange(lefti, 0, int(image_size[0]-1));
//            uint32_t right = (uint32_t)clampToRange(righti, 0, int(image_size[0]-1));
//            uint32_t top = (uint32_t)clampToRange(topi, 0, int(image_size[1]-1));
//            uint32_t bottom = (uint32_t)clampToRange(bottomi, 0, int(image_size[1]-1));

        }
        SDL_RenderPresent(renderer);
#endif
        ++frame_id;
    };

#ifdef WEB
    emscripten_set_main_loop_arg(dispatch_main, &main_loop,
                                     0 /* use browser's framerate */,
                                     1 /* simulate infinite loop */);
#else
    while (!exit) {
        main_loop();
    }
#endif

#ifdef VISUAL_UPDATE
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(win);
#endif
}
