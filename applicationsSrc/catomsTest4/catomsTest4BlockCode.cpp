#include "catomsTest4BlockCode.hpp"

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <fstream>    // used for file I/O
#include <iostream>   // for cout
#include <filesystem>
#include <sstream>    // for string stream parsing

#include "motion/teleportationEvents.h"
#include "robots/catoms3D/catoms3DMotionEngine.h"
#include "robots/catoms3D/catoms3DRotationEvents.h"

using namespace Catoms3D;


std::priority_queue<
    std::pair<Cell3DPosition, double>,
    std::vector<std::pair<Cell3DPosition, double> >,
    std::function<bool(std::pair<Cell3DPosition, double>, std::pair<Cell3DPosition, double>)>
> CatomsTest4BlockCode::openSet{
    [](std::pair<Cell3DPosition, double> a, std::pair<Cell3DPosition, double> b) {
        return a.second > b.second;
    }
};

std::map<Cell3DPosition, double> CatomsTest4BlockCode::gScore;
std::map<Cell3DPosition, double> CatomsTest4BlockCode::fScore;

std::vector<std::vector<Cell3DPosition> > CatomsTest4BlockCode::globalOptimalPaths;

//Origin/Relative pos
Cell3DPosition CatomsTest4BlockCode::Origin = Cell3DPosition(17 + 18, 5, 6);


std::queue<Cell3DPosition> CatomsTest4BlockCode::startD(
    std::deque{

        Cell3DPosition(Origin[0] + 3, Origin[1] + 1, Origin[2] + 1),
               Cell3DPosition(Origin[0] + 3, Origin[1] + 1, Origin[2] - 1),
               Cell3DPosition(Origin[0] + 3, Origin[1] + 2, Origin[2]),
               Cell3DPosition(Origin[0] + 2, Origin[1] + 1, Origin[2] - 1),
               Cell3DPosition(Origin[0] + 2, Origin[1] + 1, Origin[2] - 2),
               Cell3DPosition(Origin[0] + 2, Origin[1] + 1, Origin[2] + 1),
               Cell3DPosition(Origin[0] + 2, Origin[1] + 1, Origin[2] + 2),
               Cell3DPosition(Origin[0] + 1, Origin[1] + 1, Origin[2] + 2),
               Cell3DPosition(Origin[0] + 1, Origin[1] + 1, Origin[2] - 2),
               Cell3DPosition(Origin[0], Origin[1], Origin[2] + 1),
               Cell3DPosition(Origin[0], Origin[1], Origin[2]),
               Cell3DPosition(Origin[0], Origin[1], Origin[2] - 1),
    }
  );

// Queues for start and target positions.
std::queue<Cell3DPosition> CatomsTest4BlockCode::startQueue(
    std::deque{
        Cell3DPosition(Origin[0] + 3, Origin[1] + 1, Origin[2] + 1),
        Cell3DPosition(Origin[0] + 3, Origin[1] + 1, Origin[2] - 1),
        Cell3DPosition(Origin[0] + 3, Origin[1] + 2, Origin[2]),
        Cell3DPosition(Origin[0] + 2, Origin[1] + 1, Origin[2] - 1),
        Cell3DPosition(Origin[0] + 2, Origin[1] + 1, Origin[2] - 2),
        Cell3DPosition(Origin[0] + 2, Origin[1] + 1, Origin[2] + 1),
        Cell3DPosition(Origin[0] + 2, Origin[1] + 1, Origin[2] + 2),
        Cell3DPosition(Origin[0] + 1, Origin[1] + 1, Origin[2] + 2),
        Cell3DPosition(Origin[0] + 1, Origin[1] + 1, Origin[2] - 2),
        Cell3DPosition(Origin[0], Origin[1], Origin[2] + 1),
        Cell3DPosition(Origin[0], Origin[1], Origin[2]),
        Cell3DPosition(Origin[0], Origin[1], Origin[2] - 1),
        Cell3DPosition(Origin[0] - 1, Origin[1], Origin[2] + 1),
        Cell3DPosition(Origin[0] - 1, Origin[1], Origin[2] - 1),
        Cell3DPosition(Origin[0] - 1, Origin[1], Origin[2]),

        Cell3DPosition(Origin[0] - 2, Origin[1], Origin[2] + 1),
        Cell3DPosition(Origin[0] - 2, Origin[1], Origin[2] - 1),
        Cell3DPosition(Origin[0] - 2, Origin[1] + 1, Origin[2] + 2),
        Cell3DPosition(Origin[0] - 2, Origin[1] + 1, Origin[2] - 2),

        Cell3DPosition(Origin[0] - 3, Origin[1] + 1, Origin[2] + 2),
        Cell3DPosition(Origin[0] - 3, Origin[1] + 1, Origin[2] - 2),
        Cell3DPosition(Origin[0] - 4, Origin[1] + 1, Origin[2] + 1),
        Cell3DPosition(Origin[0] - 4, Origin[1] + 1, Origin[2] - 1),

        Cell3DPosition(Origin[0] - 4, Origin[1] + 2, Origin[2]),
        Cell3DPosition(Origin[0] - 5, Origin[1] + 1, Origin[2] + 1),
        Cell3DPosition(Origin[0] - 5, Origin[1] + 1, Origin[2] - 1),
        Cell3DPosition(Origin[0] - 5, Origin[1] + 2, Origin[2]),
        Cell3DPosition(Origin[0] - 6, Origin[1] + 1, Origin[2] - 1),
        Cell3DPosition(Origin[0] - 6, Origin[1] + 1, Origin[2] + 1),

        Cell3DPosition(Origin[0] - 6, Origin[1] + 1, Origin[2] - 2),
        Cell3DPosition(Origin[0] - 6, Origin[1] + 1, Origin[2] + 2),


        Cell3DPosition(Origin[0] - 7, Origin[1] + 1, Origin[2] - 2),
        Cell3DPosition(Origin[0] - 7, Origin[1] + 1, Origin[2] + 2),


        Cell3DPosition(Origin[0] - 8, Origin[1], Origin[2] - 1),
        Cell3DPosition(Origin[0] - 8, Origin[1], Origin[2] + 1),
        //   Cell3DPosition(12, 6, 7)


        Cell3DPosition(Origin[0] - 8, Origin[1], Origin[2]),

          Cell3DPosition(12, 6, 7)


    }
);


//state 1
std::queue<Cell3DPosition> CatomsTest4BlockCode::targetQueue(
    std::deque{
        Cell3DPosition(Origin[0] - 9, Origin[1], Origin[2] + 1), // (8,5,7)
        Cell3DPosition(Origin[0] - 9, Origin[1], Origin[2] - 1), // (8,5,5)

        Cell3DPosition(Origin[0] - 9, Origin[1], Origin[2]), // (8,5,6)
        // Cell3DPosition(7, 5, 6),
        Cell3DPosition(Origin[0] - 10, Origin[1], Origin[2] + 1), // (7,5,7)

        // Cell3DPosition(6, 5, 7),
        Cell3DPosition(Origin[0] - 10, Origin[1], Origin[2] - 1), // (7,5,5)
        // Cell3DPosition(6, 5, 5),
        Cell3DPosition(Origin[0] - 10, Origin[1] + 1, Origin[2] - 2), // (7,6,4)

        Cell3DPosition(Origin[0] - 10, Origin[1] + 1, Origin[2] + 2), // (7,6,8)
        Cell3DPosition(Origin[0] - 11, Origin[1] + 1, Origin[2] + 2), // (6,6,8)

        Cell3DPosition(Origin[0] - 11, Origin[1] + 1, Origin[2] - 2), // (6,6,4)

        Cell3DPosition(Origin[0] - 12, Origin[1] + 1, Origin[2] + 1), // (5,6,7)

        Cell3DPosition(Origin[0] - 12, Origin[1] + 1, Origin[2] - 1), // (5,6,5)
        Cell3DPosition(Origin[0] - 12, Origin[1] + 2, Origin[2]), // (5,7,6)

        Cell3DPosition(Origin[0] - 13, Origin[1] + 2, Origin[2]), // (4,7,6)
        Cell3DPosition(Origin[0] - 13, Origin[1] + 1, Origin[2] - 1), // (4,6,5)

        Cell3DPosition(Origin[0] - 13, Origin[1] + 1, Origin[2] + 1), // (4,6,7)
        Cell3DPosition(Origin[0] - 14, Origin[1] + 1, Origin[2] + 1), // (3,6,7)

        Cell3DPosition(Origin[0] - 14, Origin[1] + 1, Origin[2] - 1), // (3,6,5)
        Cell3DPosition(Origin[0] - 14, Origin[1] + 1, Origin[2] - 2), // (3,6,4)

        Cell3DPosition(Origin[0] - 14, Origin[1] + 1, Origin[2] + 2), // (3,6,8)
        Cell3DPosition(Origin[0] - 15, Origin[1] + 1, Origin[2] + 2), // (2,6,8)

        Cell3DPosition(Origin[0] - 15, Origin[1] + 1, Origin[2] - 2), // (2,6,4)

        Cell3DPosition(Origin[0] - 16, Origin[1], Origin[2] - 1), // (1,5,5)
        Cell3DPosition(Origin[0] - 16, Origin[1], Origin[2]), // (1,5,6)

        Cell3DPosition(Origin[0] - 16, Origin[1], Origin[2] + 1), // (1,5,7)
        Cell3DPosition(Origin[0] - 17, Origin[1], Origin[2] + 1),
        Cell3DPosition(Origin[0] - 17, Origin[1], Origin[2] - 1),
        Cell3DPosition(Origin[0] - 17, Origin[1], Origin[2]),
        Cell3DPosition(Origin[0] - 18, Origin[1], Origin[2] + 1),
        Cell3DPosition(Origin[0] - 18, Origin[1], Origin[2] - 1),


        Cell3DPosition(Origin[0] - 18, Origin[1] + 1, Origin[2] + 2),
        Cell3DPosition(Origin[0] - 18, Origin[1] + 1, Origin[2] - 2),
        // Cell3DPosition(1, 6, 7)

        Cell3DPosition(Origin[0] - 19, Origin[1] + 1, Origin[2] + 2),
        Cell3DPosition(Origin[0] - 19, Origin[1] + 1, Origin[2] - 2),


        Cell3DPosition(Origin[0] - 20, Origin[1] + 1, Origin[2] + 1),
        Cell3DPosition(Origin[0] - 20, Origin[1] + 1, Origin[2] - 1),

        Cell3DPosition(Origin[0] - 20, Origin[1] + 2, Origin[2]),


    }
);


// state 2
/*
std::queue<Cell3DPosition> CatomsTest3BlockCode::targetQueue(
    std::deque{
        Cell3DPosition(8, 6, 7),
        Cell3DPosition(8, 6, 5),
        Cell3DPosition(8, 7, 6),
        Cell3DPosition(7, 7, 6),
        Cell3DPosition(7, 6, 7),
        Cell3DPosition(6, 6, 4),
        Cell3DPosition(7, 6, 5),
        Cell3DPosition(6, 6, 5),
        Cell3DPosition(6, 4, 6),
        Cell3DPosition(6, 6, 8),
        Cell3DPosition(5, 6, 4),
        Cell3DPosition(5, 6, 8),
        Cell3DPosition(4, 6, 7),
        Cell3DPosition(4, 6, 5),
        Cell3DPosition(4, 7, 6),
        Cell3DPosition(3, 7, 6),
        Cell3DPosition(3, 6, 5),
        Cell3DPosition(3, 6, 7),
        Cell3DPosition(2, 6, 7),
        Cell3DPosition(2, 6, 5),
        Cell3DPosition(2, 6, 4)
    }
);

*/


CatomsTest4BlockCode::CatomsTest4BlockCode(Catoms3DBlock *host) : Catoms3DBlockCode(host) {
    if (!host) return;
    module = static_cast<Catoms3DBlock *>(hostBlock);
}


std::vector<std::vector<Cell3DPosition> > CatomsTest4BlockCode::loadAllOptimalPaths() {
    std::vector<std::vector<Cell3DPosition> > allPaths;
    std::ifstream inFile("optimal_paths.txt");
    if (!inFile.is_open()) {
        console << "Error: Could not open optimal_paths.txt for reading.\n";
        return allPaths;
    }
    std::string line;
    bool readingPath = false;
    std::vector<Cell3DPosition> currentPath;
    while (std::getline(inFile, line)) {
        // Look for the header line.
        if (line.find("Optimal path from start to goal:") != std::string::npos) {
            readingPath = true;
            currentPath.clear();
            continue;
        }
        // End of a path entry when a blank line is encountered.
        if (readingPath && line.empty()) {
            if (!currentPath.empty()) {
                allPaths.push_back(currentPath);
            }
            readingPath = false;
            continue;
        }
        if (readingPath) {
            // Expect line format: (x,y,z)
            if (!line.empty() && line.front() == '(' && line.back() == ')') {
                std::string coords = line.substr(1, line.size() - 2); // remove parentheses
                std::istringstream iss(coords);
                int x, y, z;
                char comma1, comma2;
                if (iss >> x >> comma1 >> y >> comma2 >> z) {
                    currentPath.push_back(Cell3DPosition(x, y, z));
                }
            }
        }
    }
    // In case file did not end with a blank line.
    if (readingPath && !currentPath.empty()) {
        allPaths.push_back(currentPath);
    }
    inFile.close();
    return allPaths;
}

void CatomsTest4BlockCode::startup() {
    console << "start\n";
    std::cout << "Working directory: " << std::filesystem::current_path() << std::endl;

    // Check that the module pointer is valid.
    if (!module) {
        console << "Error: module pointer is null in startup().\n";
        return;
    }

    // Load all saved optimal paths and print them.
    std::vector<std::vector<Cell3DPosition> > loadedPaths = loadAllOptimalPaths();
    if (loadedPaths.empty()) {
        console << "No optimal paths loaded.\n";
    } else {
        console << "Loaded optimal paths:\n";
        for (size_t i = 0; i < loadedPaths.size(); ++i) {
            console << "Path " << (i + 1) << ":\n";
            for (const auto &pos: loadedPaths[i]) {
                console << pos << "\n";
            }
            console << "\n";
        }
        // If there is a saved file, search for a matching path.
        for (const auto &path: loadedPaths) {
            if (!path.empty() && path.front() == module->position) {
                matchingPath = path;
                // Create a reversed copy of the matching path.
                //  reversedPath = path;
                // std::reverse(reversedPath.begin(), reversedPath.end());
                console << "Matching path found for module's current position.\n";
                break;
            }
        }
    }

    static bool hasStarted = false;
    if (hasStarted) {
        distance = -1;
        return;
    }

    if (!startQueue.empty() && (module->position == startQueue.front())) {
        hasStarted = true;
        Cell3DPosition startTarget = startQueue.front();
        startQueue.pop();
        module->setColor(RED);

        Cell3DPosition nextStep = matchingPath[1];

        cout << nextStep << endl;
        getScheduler()->schedule(new Catoms3DRotationStartEvent(getScheduler()->now() + 1000, module, nextStep));
        //getScheduler()->schedule(new TeleportationStartEvent(getScheduler()->now() + 1000, module, nextStep));
        //module->moveTo(nextStep);
    }
}


void CatomsTest4BlockCode::onMotionEnd() {
    cout << "Motion DONE.\n";
}

void CatomsTest4BlockCode::processLocalEvent(EventPtr pev) {
    // Current position of the module.
    Cell3DPosition currentPosition = module->position;
    switch (pev->eventType) {
        case EVENT_ROTATION3D_END:
            x++;

            if (!targetQueue.empty() && currentPosition == targetQueue.front()) {
                targetQueue.pop();
                //   x--;

                initiateNextModulePathfinding();
            } else if (x != matchingPath.size()) {
                Cell3DPosition nextStep = matchingPath[x];
                console << "Matching Path: Teleporting to " << nextStep << "\n";
                getScheduler()->
                        schedule(new Catoms3DRotationStartEvent(getScheduler()->now() + 1000, module, nextStep));
            }

        // If matchingPath has only one element, we have reached the final target and no further move is needed.

            break;
    }
}


void CatomsTest4BlockCode::onBlockSelected() {
    std::cerr << std::endl << "--- PRINT MODULE " << *module << " ---" << std::endl;
}

void CatomsTest4BlockCode::onAssertTriggered() {
    console << " has triggered an assert\n";
}

bool CatomsTest4BlockCode::parseUserCommandLineArgument(int &argc, char **argv[]) {
    if ((argc > 0) && ((*argv)[0][0] == '-')) {
        switch ((*argv)[0][1]) {
            case 'b': {
                std::cout << "-b option provided" << std::endl;
                return true;
            }
            break;
            case '-': {
                std::string varg = std::string((*argv)[0] + 2);
                if (varg == std::string("foo")) {
                    int fooArg;
                    try {
                        fooArg = stoi((*argv)[1]);
                        argc--;
                        (*argv)++;
                    } catch (std::logic_error &) {
                        std::stringstream err;
                        err << "foo must be an integer." << std::endl;
                        throw CLIParsingError(err.str());
                    }
                    std::cout << "--foo option provided with value: " << fooArg << std::endl;
                } else return false;
                return true;
            }
            default:
                std::cerr << "Unrecognized command line argument: " << (*argv)[0] << std::endl;
        }
    }
    return false;
}

std::string CatomsTest4BlockCode::onInterfaceDraw() {
    std::stringstream trace;
    trace << "Distance: " << distance;
    return trace.str();
}

double CatomsTest4BlockCode::heuristic(const Cell3DPosition &current, const Cell3DPosition &goal) {
    return std::abs(current[0] - goal[0]) +
           std::abs(current[1] - goal[1]) +
           std::abs(current[2] - goal[2]);
}


void CatomsTest4BlockCode::initiateNextModulePathfinding() {
    if (startQueue.empty()) {
        console << "No more modules in startQueue.\n";
        return;
    }


    Cell3DPosition nextStart = startQueue.front();
    startQueue.pop();

    //Getting module
    BuildingBlock *nextBlock = BaseSimulator::getWorld()->getBlockByPosition(nextStart);
    if (!nextBlock || !nextBlock->blockCode) {
        console << "Error: Block at position " << nextStart << " is null or has no blockCode.\n";
        return;
    }
    //Pointer
    CatomsTest4BlockCode *nextModule = static_cast<CatomsTest4BlockCode *>(nextBlock->blockCode);

    console << "x next: " << x;
    auto nextStep = nextModule->matchingPath[nextModule->x];
    console << "Moving at " << nextStart << "\n";
    nextModule->setColor(RED);

    console << "Current module host: " << module->blockId << "\n";
    console << "nextBlock->blockId = " << nextBlock->blockId << "\n";


    //getScheduler()->schedule(new TeleportationStartEvent(
    //getScheduler()->now() + 1000, nextModule->hostBlock, nextStart));

    getScheduler()->schedule(new Catoms3DRotationStartEvent(getScheduler()->now() + 1000,
                                                            dynamic_cast<Catoms3DBlock *>(nextModule->hostBlock),
                                                            nextStep));
}
