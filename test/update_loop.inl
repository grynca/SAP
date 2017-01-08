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

void start_update_loop(SAPManager2D<int>& sap, f32 space_size) {
#ifdef VISUAL_UPDATE
    SDL_Window *win = SDL_CreateWindow("SAP Updating", 100, 100, WIDTH, HEIGHT, 0);
    SDL_Renderer *renderer = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    bool mouse_down = false;
    int ray_start_x = 0, ray_start_y = 0;
    int ray_end_x = 0, ray_end_y = 0;
    u32 ray_overlaps = 0;
    bool all_ray_hits = true;
#endif
    f32 min_speed = space_size*(f32)SPEED_MIN;
    min_speed *= randFloat()*2 - 1;
    f32 max_speed = space_size*(f32)SPEED_MAX;
    max_speed *= randFloat()*2 - 1;

    // main loop
    bool exit = false;
    u32 frame_id = 0;
    Measure m_update, m_raycast;

    float dt = 0.0f;
    std::function<void()> main_loop = [&]() {
        m_update.from();
        if ((frame_id+1)%50 == 0) {
            m_update.print("Update");
            std::cout << " overlapping boxes:: " << sap.getOverlapsCount() << std::endl;
            if ((ray_end_x-ray_start_x)!=0 || (ray_end_y-ray_start_y)!=0) {
                std::cout << " raycast : " << m_raycast.calcAvgDt() << "sec (overlaps: " << ray_overlaps << ")" << std::endl;
            }
        }

#ifdef VISUAL_UPDATE
        SDL_Event evt;
        while (SDL_PollEvent(&evt) ) {
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

        for (u32 bid=0; bid<sap.getBoxesCount(); ++bid) {

            f32 move_vec[2];
            move_vec[0] = randFloatMinMax(min_speed, max_speed)*dt;
            move_vec[1] = randFloatMinMax(min_speed, max_speed)*dt;
            sap.moveBox(bid, move_vec);
        }

#ifdef VISUAL_UPDATE
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        sap.debugRender(win, renderer);
        ray_overlaps = 0;
        if ((ray_end_x-ray_start_x)!=0 || (ray_end_y-ray_start_y)!=0) {
            SDL_SetRenderDrawColor(renderer, Color::Green().r, Color::Green().g, Color::Green().b, Color::Green().a);
            SDL_RenderDrawLine(renderer, ray_start_x, ray_start_y, ray_end_x, ray_end_y);


            f32 bounds[2*2];
            sap.calcBounds(bounds);
            f32 range[2] = {bounds[2] - bounds[0], bounds[2+1] - bounds[1]};

            // convert ray to world coords
            f32 sx = range[0]/WIDTH;
            f32 sy = range[1]/HEIGHT;

            f32 ray_start[2] = {ray_start_x*sx +bounds[0], ray_start_y*sy +bounds[1]};
            f32 ray_end[2] = {ray_end_x*sx +bounds[0], ray_end_y*sy +bounds[1]};
            f32 ray_dir[2] = {ray_end[0]-ray_start[0], ray_end[1]-ray_start[1]};

            m_raycast.from();
            auto rc = sap.getRayCaster();
            rc.setRay(ray_start , ray_dir);
            rc.getHits([&ray_overlaps, &all_ray_hits](u32 bid, f32 t) {
                ++ray_overlaps;
                return all_ray_hits;
            });

            m_raycast.to();
        }
        SDL_RenderPresent(renderer);
#endif
        ++frame_id;
        m_update.to();
#ifdef CONSTANT_DT
        dt = CONSTANT_DT;
#else
        dt = m_update.calcAvgDt();
#endif
    };

#ifdef WEB
    emscripten_set_main_loop_arg(dispatch_main, &main_loop,
                                     0 /* use browser's framerate */,
                                     1 /* simulate infinite loop */);
#else
    while (!exit) {
#ifdef FRAMES
        if (frame_id == FRAMES)
            break;
#endif
        main_loop();
    }
#endif

#ifdef VISUAL_UPDATE
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(win);
#endif
}
