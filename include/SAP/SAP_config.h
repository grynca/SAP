#ifndef SAP_CONFIG_H
#define SAP_CONFIG_H

#include <stdint.h>

//#define SAP_VALIDATE_ALL_THE_TIME
//#define SAP_VALIDATE_OVERLAPS

namespace grynca{

    namespace SAP {
        static const u32 MAX_BOXES_IN_SEGMENT = 100;
        static const u32 MIN_BOXES_IN_SEGMENT = 20;
        static const f32 CROSSED_SPLIT_PENALTY_COEF = 20.0f;
        static const u32 PM_INITIAL_SIZE = 1024;   // must be 2^x
        static const u32 MAX_BOX_OCCURENCES = 8;
    }

}

#endif //SAP_CONFIG_H
