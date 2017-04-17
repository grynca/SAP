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
    ss << "boxes_count = 2000" << std::endl;
    ss << "space_size = 10000" << std::endl;
    ss << "box_size_max = 500" << std::endl;
    ss << "speed_min = 0.05" << std::endl;
    ss << "speed_max = 0.5" << std::endl;
    ss >> testbench.accLastTestConfig();

    testbench.runTest(0);
    return 0;
}