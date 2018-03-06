#define PROFILE_IN_RELEASE

#include "base.h"
using namespace grynca;
#include "SAP_test.h"


int main(int argc, char* argv[]) {
    srand(time(NULL));
    SDLTestBenchSton::create(1024, 768, true);
    SDLTestBench& testbench = SDLTestBenchSton::get();

    SAPTestFixture f;
    std::string name = "SAP example";
    testbench.addTest(name, &f);

    std::stringstream ss;
#ifdef WEB
    ss << "boxes_count = 2000" << std::endl;
    ss << "space_size = 100000" << std::endl;
    ss << "box_size_max = 2000" << std::endl;
    ss << "speed_min = 0.005" << std::endl;
    ss << "speed_max = 0.05" << std::endl;
#else
    ss << "boxes_count = 100000" << std::endl;
    ss << "space_size = 10000000" << std::endl;
    ss << "box_size_max = 1000" << std::endl;
    ss << "speed_min = 0.00005" << std::endl;
    ss << "speed_max = 0.0005" << std::endl;
#endif
    ss >> testbench.accLastTestConfig();

    testbench.runTest(0);
    return 0;
}