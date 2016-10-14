#include "../include/SAP.h"
#include "base.h"
#include "maths.h"
#include <ctime>

#include "update_loop.h"
#include "test_cfg.h"

using namespace std;
using namespace grynca;

void circles_test(uint32_t n, uint32_t size) {

    std::cout << "Circles(" << n << "):" << std::endl;
    Array<Circle> circles;
    circles.reserve(n);
    {
        BlockMeasure m(" creation");
        for (uint32_t i=0; i<n; ++i) {
            float x  = ((float)rand()/RAND_MAX)*size;
            float y = ((float)rand()/RAND_MAX)*size;
            float r = ((float)rand()/RAND_MAX)*5 + 0.1f;
            circles.add(Vec2{x, y}, r);
        }
    }

    uint32_t overlaps = 0;
    {
        BlockMeasure m(" update + collide");
        for (uint32_t i=0; i<n; ++i) {
            float dx  = ((float)rand()/RAND_MAX)*0.5f;
            float dy = ((float)rand()/RAND_MAX)*0.5f;
            Circle* c = circles.getAtPos(i);
            c->setCenter(c->getCenter()+Vec2{dx, dy});
        }

        for (uint32_t i=0; i<n; ++i) {
            Circle* c1 = circles.getAtPos(i);
            for (uint32_t j=i+1; j<n; ++j) {
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
        for (uint32_t i=0; i<destruction_order.size(); ++i) {
            uint32_t circle_id = destruction_order[i];
            circles.removeAtPos(circle_id);
        }
    }
}

void SAP_test(uint32_t n, float space_size, float box_size_max) {
    std::cout << "SAP("<< n << "): " << std::endl;
    SAPManager2D<int> sap;

    fast_vector<uint32_t> box_ids;
    {
        grynca::BlockMeasure m(" creation");
        box_ids.reserve(n);
        float bounds[6];
        for (uint32_t i=0; i< n; ++i) {
            bounds[0] = ((float)rand()/RAND_MAX)*space_size;
            bounds[1] = ((float)rand()/RAND_MAX)*space_size;
            bounds[2] = bounds[0] + ((float)rand()/RAND_MAX)*box_size_max + 0.1f;
            bounds[3] = bounds[1] + ((float)rand()/RAND_MAX)*box_size_max + 0.1f;
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
        float ro[2] = {4000, 4000};
        float rd[2] = {3, 1};
        rc.setRay(ro , rd);
        uint32_t overlaps_cnt = 0;
        rc.getHits([&overlaps_cnt](uint32_t bid, float t) {
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
        for (uint32_t i=0; i<destruction_order.size(); ++i) {
            uint32_t box_id = box_ids[destruction_order[i]];
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