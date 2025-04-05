#include "catomsTest3BlockCode.hpp"

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

// Global static variables for the A* state.
std::map<Cell3DPosition, std::vector<Cell3DPosition> > CatomsTest3BlockCode::cells;
std::vector<Cell3DPosition> CatomsTest3BlockCode::visited;
std::vector<Cell3DPosition> CatomsTest3BlockCode::teleportedPositions;

std::priority_queue<
    std::pair<Cell3DPosition, double>,
    std::vector<std::pair<Cell3DPosition, double> >,
    std::function<bool(std::pair<Cell3DPosition, double>, std::pair<Cell3DPosition, double>)>
> CatomsTest3BlockCode::openSet{
    [](std::pair<Cell3DPosition, double> a, std::pair<Cell3DPosition, double> b) {
        return a.second > b.second;
    }
};

std::map<Cell3DPosition, double> CatomsTest3BlockCode::gScore;
std::map<Cell3DPosition, double> CatomsTest3BlockCode::fScore;
std::map<Cell3DPosition, Cell3DPosition> CatomsTest3BlockCode::cameFrom;
std::set<Cell3DPosition> CatomsTest3BlockCode::closedSet;

std::vector<std::vector<Cell3DPosition> > CatomsTest3BlockCode::globalOptimalPaths;

//Origin/Relative pos
Cell3DPosition CatomsTest3BlockCode::Origin = Cell3DPosition(17 + 18, 5, 6);


// Queues for start and target positions.
std::queue<Cell3DPosition> CatomsTest3BlockCode::startQueue(
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

    }
);

//state 1
std::queue<Cell3DPosition> CatomsTest3BlockCode::targetQueue(
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


CatomsTest3BlockCode::CatomsTest3BlockCode(Catoms3DBlock *host) : Catoms3DBlockCode(host) {
    if (!host) return;
    module = static_cast<Catoms3DBlock *>(hostBlock);
}


void CatomsTest3BlockCode::startup() {
    console << "start\n";
    std::cout << "Working directory: " << std::filesystem::current_path() << std::endl;

    // Check that the module pointer is valid.
    if (!module) {
        console << "Error: module pointer is null in startup().\n";
        return;
    }


    // Normal A* initialization
    static bool hasStarted = false;
    if (hasStarted) {
        distance = -1;
        hostBlock->setColor(LIGHTGREY);
        return;
    }

    if (!startQueue.empty() && (module->position == startQueue.front())) {
        hasStarted = true;
        Cell3DPosition startTarget = startQueue.front();
        startQueue.pop();
        module->setColor(RED);
        distance = 0;
        Cell3DPosition currentPosition = startTarget;
        cells[currentPosition].clear();
        teleportedPositions.push_back(currentPosition);
        gScore[currentPosition] = 0;
        // Ensure targetQueue is not empty before using front()
        if (targetQueue.empty()) {
            console << "Error: targetQueue is empty in startup().\n";
            return;
        }
        fScore[currentPosition] = heuristic(currentPosition, targetQueue.front());

        for (auto &pos: module->getAllMotions()) {
            cells[currentPosition].push_back(pos.first);
            visited.push_back(pos.first);
        }
        openSet.push({currentPosition, fScore[currentPosition]});
        if (!openSet.empty()) {
            auto nextStep = openSet.top();
            openSet.pop();
            getScheduler()->schedule(new TeleportationStartEvent(getScheduler()->now() + 1000, module, nextStep.first));
            //  getScheduler()->schedule(new Catoms3DRotationStartEvent(getScheduler()->now() + 1000, module, nextStep.first));
            // module->moveTo(nextStep.first);
        }
    } else {
        distance = -1;
        hostBlock->setColor(LIGHTGREY);
    }
}


void CatomsTest3BlockCode::onMotionEnd() {
    console << " has reached its destination\n";
}

void CatomsTest3BlockCode::processLocalEvent(EventPtr pev) {
    // Normal A* processing.
    Cell3DPosition currentPosition = module->position;
    if (cells.find(currentPosition) == cells.end()) {
        std::vector<Cell3DPosition> neighbors;
        for (auto &motion: module->getAllMotions()) {
            neighbors.push_back(motion.first);
        }
        cells[currentPosition] = neighbors;
    }

    switch (pev->eventType) {
        //change this to rotaton event in new class
        //bare in mind every module in real life cant have static maybe configure that later
        case EVENT_TELEPORTATION_END:
            //case EVENT_ROTATION3D_END:

            if (!targetQueue.empty() && currentPosition == targetQueue.front()) {
                targetQueue.pop();
                console << "Goal reached. Reconstructing optimal path...\n";
                discoveredPath.clear();
                while (cameFrom.find(currentPosition) != cameFrom.end()) {
                    discoveredPath.push_back(currentPosition);
                    currentPosition = cameFrom[currentPosition];
                }
                discoveredPath.push_back(currentPosition);
                std::reverse(discoveredPath.begin(), discoveredPath.end());
                console << "Optimal path from start to goal:\n";
                for (auto &pos: discoveredPath) {
                    console << pos << "\n";
                }


                //save


                saveOptimalPath(discoveredPath);


                // Initiate next module's pathfinding so that subsequent modules move.
                initiateNextModulePathfinding();
            } else {
                closedSet.insert(currentPosition);
                for (auto &motion: module->getAllMotions()) {
                    Cell3DPosition neighbor = motion.first;
                    if (closedSet.find(neighbor) != closedSet.end())
                        continue;
                    double tentative_gScore = gScore[currentPosition] + 1;
                    if (gScore.find(neighbor) == gScore.end() || tentative_gScore < gScore[neighbor]) {
                        cameFrom[neighbor] = currentPosition;
                        gScore[neighbor] = tentative_gScore;
                        // Ensure targetQueue is not empty before using front()
                        if (targetQueue.empty()) {
                            console << "Error: targetQueue is empty during A* processing.\n";
                            return;
                        }
                        fScore[neighbor] = tentative_gScore + heuristic(neighbor, targetQueue.front());
                        openSet.push({neighbor, fScore[neighbor]});
                    }
                }

                if (!openSet.empty()) {
                    auto nextStep = openSet.top();
                    openSet.pop();
                    console << "A*: Teleporting to " << nextStep.first << "\n";
                    getScheduler()->schedule(
                        new TeleportationStartEvent(getScheduler()->now() + 1000, module, nextStep.first));
                    //   module->moveTo(nextStep.first);
                    //    getScheduler()->schedule(new Catoms3DRotationStartEvent(getScheduler()->now() + 1000, module, nextStep.first));
                } else {
                    console << "A*: No more nodes to explore. Path not found.\n";
                }
            }

            break;
        default:
            break;
    }
}

void CatomsTest3BlockCode::onBlockSelected() {
    std::cerr << std::endl << "--- PRINT MODULE " << *module << " ---" << std::endl;
}

void CatomsTest3BlockCode::onAssertTriggered() {
    console << " has triggered an assert\n";
}

bool CatomsTest3BlockCode::parseUserCommandLineArgument(int &argc, char **argv[]) {
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

std::string CatomsTest3BlockCode::onInterfaceDraw() {
    std::stringstream trace;
    trace << "Distance: " << distance;
    return trace.str();
}

double CatomsTest3BlockCode::heuristic(const Cell3DPosition &current, const Cell3DPosition &goal) {
    return std::abs(current[0] - goal[0]) +
           std::abs(current[1] - goal[1]) +
           std::abs(current[2] - goal[2]);
}

void CatomsTest3BlockCode::saveOptimalPath(const std::vector<Cell3DPosition> &path) {
    std::ofstream outFile("optimal_paths.txt", std::ios::app);
    if (outFile.is_open()) {
        outFile << "Optimal path from start to goal:\n";
        for (const auto &pos: path) {
            outFile << pos << "\n";
        }
        outFile << "\n"; // Separate entries with a blank line.
        outFile.close();
        console << "Optimal path saved to optimal_paths.txt\n";
    } else {
        console << "Error: Unable to open file for writing optimal path.\n";
    }
}

void CatomsTest3BlockCode::initiateNextModulePathfinding() {
    if (startQueue.empty()) {
        console << "No more modules in startQueue.\n";
        return;
    }

    // Retrieve the next starting position
    Cell3DPosition nextStart = startQueue.front();
    startQueue.pop();

    // Verify that the block exists
    BuildingBlock *nextBlock = BaseSimulator::getWorld()->getBlockByPosition(nextStart);
    if (!nextBlock || !nextBlock->blockCode) {
        console << "Error: Block at position " << nextStart << " is null or has no blockCode.\n";
        return;
    }
    Catoms3DBlockCode *nextModule = static_cast<Catoms3DBlockCode *>(nextBlock->blockCode);


    bool usedPrecomputed = false;


    if (!usedPrecomputed) {
        cells.clear();
        visited.clear();
        teleportedPositions.clear();
        while (!openSet.empty()) {
            openSet.pop();
        }
        gScore.clear();
        fScore.clear();
        cameFrom.clear();
        closedSet.clear();

        console << "Initiating *A* pathfinding* for module at " << nextStart << "\n";
        nextModule->setColor(RED);
        distance = 0;

        Cell3DPosition currentPosition = nextStart;
        cells[currentPosition].clear();
        teleportedPositions.push_back(currentPosition);

        gScore[currentPosition] = 0;

        if (targetQueue.empty()) {
            console << "Error: targetQueue is empty in initiateNextModulePathfinding().\n";
            return;
        }
        fScore[currentPosition] = heuristic(currentPosition, targetQueue.front());

        for (auto &motion: nextModule->hostBlock->getAllMotions()) {
            cells[currentPosition].push_back(motion.first);
            visited.push_back(motion.first);
        }

        openSet.push({currentPosition, fScore[currentPosition]});

        if (!openSet.empty()) {
            auto nextStep = openSet.top();
            openSet.pop();
            getScheduler()->schedule(new TeleportationStartEvent(
                getScheduler()->now() + 1000, nextModule->hostBlock, nextStep.first));

            //nextModule->hostBlock->moveTo(nextStep.first);
        }
    }
}
