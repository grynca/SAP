#include "include/SAP.h"
#include "base.h"
#include "maths.h"
#include <ctime>

using namespace std;
using namespace grynca;

void circles_test(uint32_t n, uint32_t size) {

    std::cout << "Circles(" << n << "):" << std::endl;
    UnsortedVector<Circle> circles;
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
            Circle& c = circles.getAtPos(i);
            c.setCenter(c.getCenter()+Vec2{dx, dy});
        }

        for (uint32_t i=0; i<n; ++i) {
            Circle& c1 = circles.getAtPos(i);
            for (uint32_t j=i+1; j<n; ++j) {
                Circle& c2 = circles.getAtPos(j);
                if (c1.overlaps(c2))
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
            circles.remove(circle_id);
        }
    }
}

void SAP_test(uint32_t n, uint32_t size) {
    std::cout << "SAP("<< n << "): " << std::endl;
    SAPManager<int, 2> sap;

    fast_vector<uint32_t> box_ids;
    {
        grynca::BlockMeasure m(" creation");
        box_ids.reserve(n);
        Extent e[3];
        for (uint32_t i=0; i< n; ++i) {
            e[0].min = ((float)rand()/RAND_MAX)*size;
            e[1].min = ((float)rand()/RAND_MAX)*size;
            e[2].min = ((float)rand()/RAND_MAX)*size;
            e[0].max = e[0].min + ((float)rand()/RAND_MAX)*5 + 0.1f;
            e[1].max = e[1].min + ((float)rand()/RAND_MAX)*5 + 0.1f;
            e[2].max = e[2].min + ((float)rand()/RAND_MAX)*5 + 0.1f;
            uint32_t group_id = rand()%5;
            box_ids.push_back(sap.addBox(e, group_id, i));
        }
    }

    uint32_t overlaps = 0;
    {
        sap.updateCollisionFlags();
        grynca::BlockMeasure m(" update + collide");
        for (uint32_t i=0; i<box_ids.size(); ++i) {
            Extent e[3];
            sap.getBox(box_ids[i], e);

            float dx = ((float)rand()/RAND_MAX)*0.5f;
            float dy = ((float)rand()/RAND_MAX)*0.5f;
            float dz = ((float)rand()/RAND_MAX)*0.5f;
            e[0].min +=dx;
            e[0].max +=dx;
            e[1].min +=dy;
            e[1].max +=dy;
            e[2].min +=dz;
            e[2].max +=dz;
            sap.updateBox(box_ids[i], e);
        }
        overlaps = sap.getOverlapsCount();
    }

    std::cout << "   overlaps: " << overlaps << std::endl;


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


void addBox(SAPManager<int>& sap, float left, float right, float top, float bottom, int id) {
    Extent e[3];
    e[0].min = left;
    e[0].max = right;
    e[1].min = top;
    e[1].max = bottom;
    sap.addBox(e, 0, id);
}

int main() {
    srand(time(NULL));

//    circles_test(1000, 100);
//    SAP_test(1000, 100);
//
//    circles_test(5000, 500);
//    SAP_test(5000, 500);

    circles_test(10000, 1000);
    SAP_test(10000, 1000);

//    circles_test(50000, 5000);
//    SAP_test(50000, 5000);

    return 0;
}