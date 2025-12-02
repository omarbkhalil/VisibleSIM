#include <iostream>

#include "robots/slidingCubes/slidingCubesSimulator.h"
#include "robots/slidingCubes/slidingCubesBlockCode.h"

#include "snackeMotionScBlockCode.hpp"

using namespace std;
using namespace SlidingCubes;

int main(int argc, char **argv) {
    try
    {
        createSimulator(argc, argv, SnackeMotionScBlockCode::buildNewBlockCode);
        getSimulator()->printInfo();
        BaseSimulator::getWorld()->printInfo();
        deleteSimulator();
    }
    catch(std::exception const& e)
    {
        cerr << "Uncaught exception: " << e.what();
    }

    return 0;
}
