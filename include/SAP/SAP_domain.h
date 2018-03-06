#ifndef SAP_DOMAIN_H
#define SAP_DOMAIN_H

#include "types/Type.h"

#define SAP_DOMAIN_TYPES(DOMAIN) \
    static constexpr u32 AXES_COUNT = DOMAIN::AXES_COUNT; \
    typedef typename DOMAIN::BoxDataT BoxDataT; \
    typedef typename DOMAIN::OverlapDataT OverlapDataT; \
    typedef SAPManagerBase<DOMAIN> Manager; \
    typedef SAPSegment<DOMAIN> Segment; \
    typedef SAPRaycaster<DOMAIN> Raycaster; \
    typedef SAP::SAPBox<DOMAIN> Box;

namespace grynca {

    // fw
    template <typename> class SAPManagerBase;
    template <typename> class SAPSegment;
    template <typename> class SAPRaycaster;
    namespace SAP { template <typename> class SAPBox; }

    template <typename BoxData, typename OverlapData = DummyType>
    struct SAPDomain2D {
        enum { AXES_COUNT = 2 };
        typedef BoxData BoxDataT;
        typedef OverlapData OverlapDataT;
    };

    template <typename BoxData, typename OverlapData = DummyType>
    struct SAPDomain3D {
        enum { AXES_COUNT = 3 };
        typedef BoxData BoxDataT;
        typedef OverlapData OverlapDataT;
    };
}

#endif //SAP_DOMAIN_H
