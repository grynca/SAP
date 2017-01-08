#include "test_cfg.h"
#include "../include/SAP.h"
#include "maths.h"
#include "update_loop.h"

using namespace std;
using namespace grynca;

void circles_test(u32 n, u32 size) {

    std::cout << "Circles(" << n << "):" << std::endl;
    Array<Circle> circles;
    circles.reserve(n);
    {
        BlockMeasure m(" creation");
        for (u32 i=0; i<n; ++i) {
            f32 x  = ((f32)rand()/RAND_MAX)*size;
            f32 y = ((f32)rand()/RAND_MAX)*size;
            f32 r = ((f32)rand()/RAND_MAX)*5 + 0.1f;
            circles.add(Vec2{x, y}, r);
        }
    }

    u32 overlaps = 0;
    {
        BlockMeasure m(" update + collide");
        for (u32 i=0; i<n; ++i) {
            f32 dx  = ((f32)rand()/RAND_MAX)*0.5f;
            f32 dy = ((f32)rand()/RAND_MAX)*0.5f;
            Circle* c = circles.getAtPos(i);
            c->setCenter(c->getCenter()+Vec2{dx, dy});
        }

        for (u32 i=0; i<n; ++i) {
            Circle* c1 = circles.getAtPos(i);
            for (u32 j=i+1; j<n; ++j) {
                Circle* c2 = circles.getAtPos(j);
                if (c1->overlaps(*c2))
                    ++overlaps;
            }
        }
    }
    std::cout << "   overlaps: " << overlaps << std::endl;

    fast_vector<unsigned int> destruction_order;
    destruction_order.reserve(n);
    randomPickN(destruction_order, n, n);
    {
        grynca::BlockMeasure m(" removal");
        for (u32 i=0; i<destruction_order.size(); ++i) {
            u32 circle_id = destruction_order[i];
            circles.removeAtPos(circle_id);
        }
    }
}

void SAP_test(u32 n, f32 space_size, f32 box_size_max) {
    std::cout << "SAP("<< n << "): " << std::endl;
    SAPManager2D<int> sap;

    fast_vector<u32> box_ids;
    {
        grynca::BlockMeasure m(" creation");
        box_ids.reserve(n);
        f32 bounds[6];
        for (u32 i=0; i< n; ++i) {
            bounds[0] = ((f32)rand()/RAND_MAX)*space_size;
            bounds[1] = ((f32)rand()/RAND_MAX)*space_size;
            bounds[2] = bounds[0] + ((f32)rand()/RAND_MAX)*box_size_max + 0.1f;
            bounds[3] = bounds[1] + ((f32)rand()/RAND_MAX)*box_size_max + 0.1f;
            //std::cout << "adding box: [" << e[0].min << ", " << e[1].min<< "], [" << e[0].max << ", " << e[1].max << "]" << std::endl;

            box_ids.push_back(sap.addBox(bounds, i));
//            if (i%1000 == 0)
//                std::cout << i << std::endl;
        }
    }

#ifndef WEB
    std::cout << sap.debugPrint();
    sap.debugImage()->saveToPNG("dbg.png");
    std::cout << "overlaps: " << sap.getOverlapsCount() << std::endl;
#endif

    {
        grynca::BlockMeasure m(" Ray Cast");
        auto rc = sap.getRayCaster();
        f32 ro[2] = {4000, 4000};
        f32 rd[2] = {3, 1};
        rc.setRay(ro , rd);
        u32 overlaps_cnt = 0;
        rc.getHits([&overlaps_cnt](u32 bid, f32 t) {
            ++overlaps_cnt;
            return true;
        });
        std::cout << " ray overlaps: " << overlaps_cnt << std::endl;
    }

    start_update_loop(sap, space_size);

    fast_vector<unsigned int> destruction_order;
    destruction_order.reserve(n);
    randomPickN(destruction_order, n, n);
    {
        grynca::BlockMeasure m(" removal");
        for (u32 i=0; i<destruction_order.size(); ++i) {
            u32 box_id = box_ids[destruction_order[i]];
            sap.removeBox(box_id);
        }
    }
}

int main(int argc, char* argv[]) {
        //srand(time(NULL));

//    circles_test(1000, 100);
//    SAP_test(1000, 100);
//
//    circles_test(5000, 500);
//    SAP_test(5000, 500);

//    circles_test(10000, 1000);
//    SAP_test(10000, 1000);

//    circles_test(50000, 5000);
    SAP_test(N_BOXES, SPACE_SIZE, BOXES_SIZE);

    WAIT_FOR_KEY_ON_WIN();
    return 0;
}