#include "base.h"
#include "update_loop.h"
#include "test_cfg.h"
#include "SDL2/SDL.h"
using namespace grynca;

#ifdef VISUAL_UPDATE
#   define WIDTH 1024
#   define HEIGHT 768
#endif

void start_update_loop(SAPManager2D<int>& sap, float space_size) {
#ifdef VISUAL_UPDATE
    SDL_Window *win = SDL_CreateWindow("SAP Updating", 100, 100, WIDTH, HEIGHT, 0);
    SDL_Renderer *renderer = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    SDL_Rect texr;
    texr.x = 0; texr.y = 0; texr.w = WIDTH; texr.h = HEIGHT;
#endif

    float min_speed = space_size*SPEED_MIN;
    min_speed *= randFloat()*2 - 1;
    float max_speed = space_size*SPEED_MAX;
    max_speed *= randFloat()*2 - 1;

    // main loop
    uint32_t frame_id = 0;
    grynca::Measure m("update");
    while (1) {
        m.incCounter();
#ifdef CONSTANT_DT
        float dt = CONSTANT_DT;
#else
        float dt = m.calcDt();
#endif
        if (m.getCounter() == 1) {
            m.print();
            std::cout << " overlaps count: " << sap.getOverlapsCount() << std::endl;
            m.reset();
        }
#ifdef VISUAL_UPDATE
        SDL_Event evt;
        if ( SDL_PollEvent(&evt) ) {
            if (evt.type == SDL_QUIT)
                break;
            else if (evt.type == SDL_KEYUP && evt.key.keysym.sym == SDLK_ESCAPE)
                break;
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
        SDL_RenderClear(renderer);          // clear the screen

        Image::Ref dbgimg = sap.debugImage();
        SDL_Texture* tex = SDL_CreateTextureFromSurface(renderer, dbgimg->getInternal());
        SDL_RenderCopy(renderer, tex, NULL, &texr);
        SDL_RenderPresent(renderer);        // flip the backbuffer
        SDL_DestroyTexture(tex);
#endif
        ++frame_id;
    }
#ifdef VISUAL_UPDATE
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(win);
#endif
}
