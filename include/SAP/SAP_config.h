#ifndef SAP_CONFIG_H
#define SAP_CONFIG_H

#include <stdint.h>

//#define VALIDATE_ALL_THE_TIME
//#define VALIDATE_OVERLAPS

namespace grynca{

    namespace SAP {
        static const uint32_t MAX_BOXES_IN_SEGMENT = 1000;
        static const uint32_t MIN_BOXES_IN_SEGMENT = 200;
        static const float CROSSED_SPLIT_PENALTY_COEF = 2.0f;
        static const uint32_t PM_INITIAL_SIZE = 1024;   // must be 2^x
        static const uint32_t MAX_BOX_OCCURENCES = 4;
    }

}

#endif //SAP_CONFIG_H
