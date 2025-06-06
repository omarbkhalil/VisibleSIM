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


std::vector<Cell3DPosition> CatomsTest3BlockCode::dPhasePath;
int CatomsTest3BlockCode::dPhaseIndex = 0;
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
Cell3DPosition CatomsTest3BlockCode::Origin = Cell3DPosition(35, 5, 6);


bool CatomsTest3BlockCode::NFound = false;

std::queue<Cell3DPosition> CatomsTest3BlockCode::startD(
    std::deque{

        Cell3DPosition(Origin[0] + 3, Origin[1] + 1, Origin[2] + 1),
               // Cell3DPosition(Origin[0] + 3, Origin[1] + 1, Origin[2] - 1),
               // Cell3DPosition(Origin[0] + 3, Origin[1] + 2, Origin[2]),
               // Cell3DPosition(Origin[0] + 2, Origin[1] + 1, Origin[2] - 1),
               // Cell3DPosition(Origin[0] + 2, Origin[1] + 1, Origin[2] - 2),
               // Cell3DPosition(Origin[0] + 2, Origin[1] + 1, Origin[2] + 1),
               // Cell3DPosition(Origin[0] + 2, Origin[1] + 1, Origin[2] + 2),
               // Cell3DPosition(Origin[0] + 1, Origin[1] + 1, Origin[2] + 2),
               // Cell3DPosition(Origin[0] + 1, Origin[1] + 1, Origin[2] - 2),
               // Cell3DPosition(Origin[0], Origin[1], Origin[2] + 1),
               // Cell3DPosition(Origin[0], Origin[1], Origin[2]),
               // Cell3DPosition(Origin[0], Origin[1], Origin[2] - 1),
    }
  );

std::queue<Cell3DPosition> CatomsTest3BlockCode::targetD(
    std::deque{
// x should be -1
        Cell3DPosition(Origin[0]  , Origin[1] -1, Origin[2] )
    }
  );

//35, 5, 6 origin
std::queue<Cell3DPosition> CatomsTest3BlockCode::startT(
    std::deque{

        Cell3DPosition(Origin[0]-1, Origin[1] , Origin[2] )
 //         Cell3DPosition(Origin[0] - 1, Origin[1], Origin[2] - 1),
 //         Cell3DPosition(Origin[0] - 1, Origin[1], Origin[2]),
 //
 //         Cell3DPosition(Origin[0] - 2, Origin[1], Origin[2] + 1),
 //         Cell3DPosition(Origin[0] - 2, Origin[1], Origin[2] - 1),
 //         Cell3DPosition(Origin[0] - 2, Origin[1] + 1, Origin[2] + 2),
 //         Cell3DPosition(Origin[0] - 2, Origin[1] + 1, Origin[2] - 2),
 //
 //         Cell3DPosition(Origin[0] - 3, Origin[1] + 1, Origin[2] + 2),
 //         Cell3DPosition(Origin[0] - 3, Origin[1] + 1, Origin[2] - 2),
 //         Cell3DPosition(Origin[0] - 4, Origin[1] + 1, Origin[2] + 1),
 //         Cell3DPosition(Origin[0] - 4, Origin[1] + 1, Origin[2] - 1),
 //
 //         Cell3DPosition(Origin[0] - 4, Origin[1] + 2, Origin[2]),
    }
  );

std::queue<Cell3DPosition> CatomsTest3BlockCode::targetT(
    std::deque{
// y should be + 2
        Cell3DPosition(Origin[0] -4 , Origin[1] + 1, Origin[2] )
    }
  );

std::queue<Cell3DPosition> CatomsTest3BlockCode::startA(
    std::deque{
// y+1
        Cell3DPosition(Origin[0] -5 , Origin[1] +1 , Origin[2] )
    }
  );
std::queue<Cell3DPosition> CatomsTest3BlockCode::targetA(
    std::deque{
// First target
        // -9 0 0
        Cell3DPosition(Origin[0] -9 , Origin[1] , Origin[2] )
        //
        //
        //
    }
  );

// Queues for start and target positions.
// std::queue<Cell3DPosition> CatomsTest3BlockCode::startQueue(
//     std::deque{
//         Cell3DPosition(Origin[0] + 3, Origin[1] + 1, Origin[2] + 1),
//         Cell3DPosition(Origin[0] + 3, Origin[1] + 1, Origin[2] - 1),
//         Cell3DPosition(Origin[0] + 3, Origin[1] + 2, Origin[2]),
//         Cell3DPosition(Origin[0] + 2, Origin[1] + 1, Origin[2] - 1),
//         Cell3DPosition(Origin[0] + 2, Origin[1] + 1, Origin[2] - 2),
//         Cell3DPosition(Origin[0] + 2, Origin[1] + 1, Origin[2] + 1),
//         Cell3DPosition(Origin[0] + 2, Origin[1] + 1, Origin[2] + 2),
//         Cell3DPosition(Origin[0] + 1, Origin[1] + 1, Origin[2] + 2),
//         Cell3DPosition(Origin[0] + 1, Origin[1] + 1, Origin[2] - 2),
//         Cell3DPosition(Origin[0], Origin[1], Origin[2] + 1),
//         Cell3DPosition(Origin[0], Origin[1], Origin[2]),
//         Cell3DPosition(Origin[0], Origin[1], Origin[2] - 1),
//         Cell3DPosition(Origin[0] - 1, Origin[1], Origin[2] + 1),
//         Cell3DPosition(Origin[0] - 1, Origin[1], Origin[2] - 1),
//         Cell3DPosition(Origin[0] - 1, Origin[1], Origin[2]),
//
//         Cell3DPosition(Origin[0] - 2, Origin[1], Origin[2] + 1),
//         Cell3DPosition(Origin[0] - 2, Origin[1], Origin[2] - 1),
//         Cell3DPosition(Origin[0] - 2, Origin[1] + 1, Origin[2] + 2),
//         Cell3DPosition(Origin[0] - 2, Origin[1] + 1, Origin[2] - 2),
//
//         Cell3DPosition(Origin[0] - 3, Origin[1] + 1, Origin[2] + 2),
//         Cell3DPosition(Origin[0] - 3, Origin[1] + 1, Origin[2] - 2),
//         Cell3DPosition(Origin[0] - 4, Origin[1] + 1, Origin[2] + 1),
//         Cell3DPosition(Origin[0] - 4, Origin[1] + 1, Origin[2] - 1),
//
//         Cell3DPosition(Origin[0] - 4, Origin[1] + 2, Origin[2]),
//         Cell3DPosition(Origin[0] - 5, Origin[1] + 1, Origin[2] + 1),
//         Cell3DPosition(Origin[0] - 5, Origin[1] + 1, Origin[2] - 1),
//         Cell3DPosition(Origin[0] - 5, Origin[1] + 2, Origin[2]),
//         Cell3DPosition(Origin[0] - 6, Origin[1] + 1, Origin[2] - 1),
//         Cell3DPosition(Origin[0] - 6, Origin[1] + 1, Origin[2] + 1),
//
//         Cell3DPosition(Origin[0] - 6, Origin[1] + 1, Origin[2] - 2),
//         Cell3DPosition(Origin[0] - 6, Origin[1] + 1, Origin[2] + 2),
//
//
//         Cell3DPosition(Origin[0] - 7, Origin[1] + 1, Origin[2] - 2),
//         Cell3DPosition(Origin[0] - 7, Origin[1] + 1, Origin[2] + 2),
//
//
//         Cell3DPosition(Origin[0] - 8, Origin[1], Origin[2] - 1),
//         Cell3DPosition(Origin[0] - 8, Origin[1], Origin[2] + 1),
//         //   Cell3DPosition(12, 6, 7)
//
//
//         Cell3DPosition(Origin[0] - 8, Origin[1], Origin[2]),
//
//     }
// );
//
// //state 1
// std::queue<Cell3DPosition> CatomsTest3BlockCode::targetQueue(
//     std::deque{
//         Cell3DPosition(Origin[0] - 9, Origin[1], Origin[2] + 1), // (8,5,7)
//         Cell3DPosition(Origin[0] - 9, Origin[1], Origin[2] - 1), // (8,5,5)
//
//         Cell3DPosition(Origin[0] - 9, Origin[1], Origin[2]), // (8,5,6)
//         // Cell3DPosition(7, 5, 6),
//         Cell3DPosition(Origin[0] - 10, Origin[1], Origin[2] + 1), // (7,5,7)
//
//         // Cell3DPosition(6, 5, 7),
//         Cell3DPosition(Origin[0] - 10, Origin[1], Origin[2] - 1), // (7,5,5)
//         // Cell3DPosition(6, 5, 5),
//         Cell3DPosition(Origin[0] - 10, Origin[1] + 1, Origin[2] - 2), // (7,6,4)
//
//         Cell3DPosition(Origin[0] - 10, Origin[1] + 1, Origin[2] + 2), // (7,6,8)
//         Cell3DPosition(Origin[0] - 11, Origin[1] + 1, Origin[2] + 2), // (6,6,8)
//
//         Cell3DPosition(Origin[0] - 11, Origin[1] + 1, Origin[2] - 2), // (6,6,4)
//
//         Cell3DPosition(Origin[0] - 12, Origin[1] + 1, Origin[2] + 1), // (5,6,7)
//
//         Cell3DPosition(Origin[0] - 12, Origin[1] + 1, Origin[2] - 1), // (5,6,5)
//         Cell3DPosition(Origin[0] - 12, Origin[1] + 2, Origin[2]), // (5,7,6)
//
//         Cell3DPosition(Origin[0] - 13, Origin[1] + 2, Origin[2]), // (4,7,6)
//         Cell3DPosition(Origin[0] - 13, Origin[1] + 1, Origin[2] - 1), // (4,6,5)
//
//         Cell3DPosition(Origin[0] - 13, Origin[1] + 1, Origin[2] + 1), // (4,6,7)
//         Cell3DPosition(Origin[0] - 14, Origin[1] + 1, Origin[2] + 1), // (3,6,7)
//
//         Cell3DPosition(Origin[0] - 14, Origin[1] + 1, Origin[2] - 1), // (3,6,5)
//         Cell3DPosition(Origin[0] - 14, Origin[1] + 1, Origin[2] - 2), // (3,6,4)
//
//         Cell3DPosition(Origin[0] - 14, Origin[1] + 1, Origin[2] + 2), // (3,6,8)
//         Cell3DPosition(Origin[0] - 15, Origin[1] + 1, Origin[2] + 2), // (2,6,8)
//
//         Cell3DPosition(Origin[0] - 15, Origin[1] + 1, Origin[2] - 2), // (2,6,4)
//
//         Cell3DPosition(Origin[0] - 16, Origin[1], Origin[2] - 1), // (1,5,5)
//         Cell3DPosition(Origin[0] - 16, Origin[1], Origin[2]), // (1,5,6)
//
//         Cell3DPosition(Origin[0] - 16, Origin[1], Origin[2] + 1), // (1,5,7)
//         Cell3DPosition(Origin[0] - 17, Origin[1], Origin[2] + 1),
//         Cell3DPosition(Origin[0] - 17, Origin[1], Origin[2] - 1),
//         Cell3DPosition(Origin[0] - 17, Origin[1], Origin[2]),
//         Cell3DPosition(Origin[0] - 18, Origin[1], Origin[2] + 1),
//         Cell3DPosition(Origin[0] - 18, Origin[1], Origin[2] - 1),
//
//
//         Cell3DPosition(Origin[0] - 18, Origin[1] + 1, Origin[2] + 2),
//         Cell3DPosition(Origin[0] - 18, Origin[1] + 1, Origin[2] - 2),
//         // Cell3DPosition(1, 6, 7)
//
//         Cell3DPosition(Origin[0] - 19, Origin[1] + 1, Origin[2] + 2),
//         Cell3DPosition(Origin[0] - 19, Origin[1] + 1, Origin[2] - 2),
//
//
//         Cell3DPosition(Origin[0] - 20, Origin[1] + 1, Origin[2] + 1),
//         Cell3DPosition(Origin[0] - 20, Origin[1] + 1, Origin[2] - 1),
//
//         Cell3DPosition(Origin[0] - 20, Origin[1] + 2, Origin[2]),
//
//     }
// );


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

    if (!module) {
        console << "Error: module pointer is null in startup().\n";
        return;
    }

    if (!startD.empty() && module->position == startD.front()) {
        // Only the module at front of startD proceeds
        console << "[startup] This module (" << module->position << ") is starting D-phase.\n";

        // Compute path now
        dPhasePath = findOptimalPath(module->position, targetD.front());
        if (dPhasePath.empty()) {
            console << "[startup] No path found. Abort.\n";
            return;
        }

        // Now we can safely pop
        startD.pop();
        dPhaseIndex = 1;
        distance = 0;
        module->setColor(RED);

        saveOptimalPath(dPhasePath, "D-phase");

        if (dPhasePath.size() > 1) {
            Cell3DPosition firstHop = dPhasePath[dPhaseIndex];
            console << "[startup] Scheduling first hop to " << firstHop << "\n";
            getScheduler()->schedule(
                new TeleportationStartEvent(getScheduler()->now() + 1000, module, firstHop));
        } else {
            console << "[startup] D-phase path trivial. Skipping.\n";
            finishedD = true;
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
    //Start transfer
    BuildingBlock *nextBlock = BaseSimulator::getWorld()->getBlockByPosition(startT.front());
    CatomsTest3BlockCode *ModuleN = static_cast<CatomsTest3BlockCode *>(nextBlock->blockCode);

// Activate A-PHASE
    if (finishedD && finishedT && !startedA) {
        startedA = true;
        console << "[#34] Beginning A-phase now that D & T are both done.\n";
        scheduleOneTeleportT();  // go to A-PHASE starter
    }

    switch (pev->eventType) {
        //change this to rotaton event in new class
        //bare in mind every module in real life cant have static maybe configure that later
        // case EVENT_ADD_NEIGHBOR: {
        //
        //     // Only act when this module is the one at the front of startT
        //     if (module->position == startT.front()) {
        //
        //         // grab the next target
        //         Cell3DPosition nextPos = startT.front();
        //         startT.pop();
        //         console << "Neighbor detected – teleporting to " << nextPos << "\n";
        //
        //         // schedule a teleport in 1 second
        //         getScheduler()->schedule(
        //             new TeleportationStartEvent(
        //                 getScheduler()->now() + 1000,
        //                 module,
        //                 nextPos
        //             )
        //         );
        //     }
        //     break;
        // }


        case EVENT_TELEPORTATION_END:{
            //case EVENT_ROTATION3D_END:
            //here should !
            if (!finishedD) {
                if (dPhasePath.empty()) {

                    dPhasePath = findOptimalPath(startD.front(), targetD.front());
                    if (dPhasePath.empty()) {
                        console << "[#34] No D-phase path found, abort.\n";
                        return;
                    }

                    console << "[#34] D-phase path (" << dPhasePath.size() << " steps):\n";
                    for (const auto& pos : dPhasePath) {
                        console << pos << "\n";
                    }

                    saveOptimalPath(dPhasePath, "D-phase");

                    dPhaseIndex = 1;
                    if (dPhasePath.size() <= 1) {
                        console << "[#34] D-phase trivial or empty path, skipping.\n";
                        finishedD = true;
                        return;
                    }

                    Cell3DPosition firstHop = dPhasePath[dPhaseIndex];
                    console << "[#34] Scheduling D-phase first hop to " << firstHop << "\n";
                    getScheduler()->schedule(
                        new TeleportationStartEvent(getScheduler()->now() + 1000, module, firstHop)
                    );
                    return;
                }

                if (dPhaseIndex + 1 < dPhasePath.size()) {
                    ++dPhaseIndex;
                    Cell3DPosition nextHop = dPhasePath[dPhaseIndex];
                    console << "[#34] Scheduling next D-phase hop to " << nextHop << "\n";
                    getScheduler()->schedule(
                        new TeleportationStartEvent(getScheduler()->now() + 1000, module, nextHop)
                    );
                } else {
                    console << "[#34] D-phase complete at " << currentPosition << "\n";
                    finishedD = true;

                    // Continue to T-phase immediately
                    scheduleOneTeleportD(); // or call next logic directly
                }
            }

// i think we need to do y+1 (app crashed)
            if (finishedD) {
                // 1) If tPhasePath is empty, compute it and schedule exactly one hop:
                if (tPhasePath.empty()) {
                    tPhasePath = findOptimalPath(module->position, targetT.front());
                    if (tPhasePath.empty()) {
                        console << "[#34] No T-phase path found, abort.\n";
                        return;
                    }

                    // Print & Save…
                    console << "[#34] T-phase path (" << tPhasePath.size() << " steps):\n";
                    // (print each element) …

                    // Schedule the very first hop, then immediately return:
                    tPhaseIndex = 1;
                    Cell3DPosition firstHop = tPhasePath[tPhaseIndex];
                    console << "[#34] Scheduling T-phase first hop to "
                            << firstHop << "\n";
                    getScheduler()->schedule(
                        new TeleportationStartEvent(
    getScheduler()->now() + 1000, module, firstHop)
                    );
                    return;  // ← crucial: stop here so you don’t fall through
                }

                // 2) If we already have a path, schedule exactly one more hop:
                if (tPhaseIndex + 1 < tPhasePath.size()) {
                    ++tPhaseIndex;
                    Cell3DPosition nextHop = tPhasePath[tPhaseIndex];
                    console << "[#34] Scheduling next T-phase hop to "
                            << nextHop << "\n";
                    getScheduler()->schedule(
                        new TeleportationStartEvent(
    getScheduler()->now() + 1000, module, nextHop)
                    );
                }
                else {
                    // We just finished the final hop:
                    console << "[#34] T-phase complete at " << currentPosition << "\n";
                  //  finishedD = false;

                    if(!tPhasePathSaved) {
                        saveOptimalPath(tPhasePath, "T-phase");
                        tPhasePathSaved = true;

                    }

                    finishedT = true;
                 //   scheduleOneTeleportT();
                }
            }


            if (finishedT && !finishedA) {
                if (aPhasePath.empty()) {
                    aPhasePath = findOptimalPath(module->position, targetA.front());
                    if (aPhasePath.empty()) {
                        console << "[#34] No A-phase path found, abort.\n";
                        return;
                    }

                    console << "[#34] A-phase path (" << aPhasePath.size() << " steps):\n";
                    for (const auto& pos : aPhasePath) {
                        console << pos << "\n";
                    }

                    saveOptimalPath(aPhasePath, "A-phase");

                    aPhaseIndex = 1;
                    if (aPhasePath.size() <= 1) {
                        console << "[#34] A-phase trivial or empty path, skipping.\n";
                        finishedA = true;
                        return;
                    }
                    Cell3DPosition firstHop = aPhasePath[aPhaseIndex];
                    console << "[#34] Scheduling A-phase first hop to " << firstHop << "\n";
                    getScheduler()->schedule(
                        new TeleportationStartEvent(
                            getScheduler()->now() + 1000, module, firstHop
                        )
                    );
                    return;
                }

                if (aPhaseIndex + 1 < aPhasePath.size()) {
                    ++aPhaseIndex;
                    Cell3DPosition nextHop = aPhasePath[aPhaseIndex];
                    console << "[#34] Scheduling next A-phase hop to " << nextHop << "\n";
                    getScheduler()->schedule(
                        new TeleportationStartEvent(
                            getScheduler()->now() + 1000, module, nextHop
                        )
                    );
                } else {
                    console << "[#34] A-phase complete at " << currentPosition << "\n";
                    finishedA = true;
                }
            }




else if (finishedA) {
    console << "Process for this module ended\n";
}
            break;}
        default:
            break;
    }
}

struct Node {
    Cell3DPosition pos;
    double f;
    // priority_queue is a max‐heap by default, so we invert the comparison:
    bool operator<(const Node& other) const {
        return this->f > other.f;
    }
};
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

void CatomsTest3BlockCode::saveOptimalPath(
    const std::vector<Cell3DPosition> &path,
    const std::string &phaseLabel
) {
    std::ofstream outFile("optimal_paths.txt", std::ios::app);
    if (!outFile.is_open()) {
        console << "Error: Unable to open file for writing optimal path.\n";
        return;
    }

    // Print a header that includes the phase label (e.g. “D-phase” or “T-phase”)
    outFile << phaseLabel << " path from start to goal:\n";

    // If this is T-phase, skip the first entry (because it duplicates the D-phase endpoint)
    size_t startIndex = 0;
    if ((phaseLabel == "T-phase" || phaseLabel == "A-phase") && path.size() > 1) {
        startIndex = 1;
    }


    for (size_t i = startIndex; i < path.size(); ++i) {
        outFile << path[i] << "\n";
    }
    outFile << "\n";  // blank line to separate entries
    outFile.close();

    console << phaseLabel << " path saved to optimal_paths.txt\n";
}


void CatomsTest3BlockCode::initiateNextModulePathfinding() {
    if (startD.empty()) {
        console << "No more modules in startQueue.\n";
        return;
    }

    // Retrieve the next starting position
    Cell3DPosition nextStart = startD.front();
    startD.pop();

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

        if (targetD.empty()) {
            console << "Error: targetQueue is empty in initiateNextModulePathfinding().\n";
            return;
        }
        fScore[currentPosition] = heuristic(currentPosition, targetD.front());

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

bool CatomsTest3BlockCode::getAllPossibleMotionsFromPosition(
    Cell3DPosition position, vector<pair<short, short>> &links,
    vector<Cell3DPosition> &reachablePositions) {

    bool found = false;
    for(auto &neighPos: lattice->getActiveNeighborCells(position)) {
        Catoms3DBlock* neigh = static_cast<Catoms3DBlock*>(lattice->getBlock(neighPos));
        //cerr << "Neighbor: " << neigh->blockId << endl;
        vector<Catoms3DMotionRulesLink*> vec;
        Catoms3DMotionRules motionRulesInstance; // Create an instance of Catoms3DMotionRules
        Cell3DPosition pos;
        if (!neigh) {
            console << "Warning: null neighbor block at " << neighPos << "\n";
            continue;
        }

        short conFrom = neigh->getConnectorId(position);
        if (conFrom < 0 || conFrom >= 12) {  // Catoms3D has 12 connectors
            console << "Warning: invalid connector ID from " << *neigh << " to " << position << "\n";
            continue;
        }
        //cout << "ConFrom: " << conFrom << endl;

        if (neigh && conFrom >= 0 && conFrom < 12) {
            motionRulesInstance.getValidMotionListFromPivot(neigh, conFrom, vec, static_cast<FCCLattice*>(lattice), NULL);
        }
        for(auto link: vec) {
            cout << link->getConFromID() << " -> " << link->getConToID() << endl;
            Cell3DPosition toPos;
            links.push_back(make_pair(link->getConFromID(), link->getConToID()));
            neigh->getNeighborPos(link->getConToID(), toPos);
            reachablePositions.push_back(toPos);
            found = true;
        }
    }
    return found;
}

void CatomsTest3BlockCode::scheduleOneTeleportD()
{
    // reset A* for this single move
    // gScore.clear();
    // fScore.clear();
    // cameFrom.clear();
    // closedSet.clear();
    // while (!openSet.empty()) openSet.pop();
    // cells.clear();

    // initialize just this one step
    // gScore[start] = 0;
    // fScore[start] = heuristic(start, goal);
    // auto &nbrs = cells[start];
    // nbrs.clear();
    // for (auto &m : module->getAllMotions())
    //     nbrs.push_back(m.first);
    // openSet.push({start, fScore[start]});
    //
    // // schedule the teleport event
    // console << "Scheduling teleport from " << start
    //         << " toward " << goal << "\n";
    getScheduler()->schedule(
  new TeleportationStartEvent(
    getScheduler()->now() + 1000,   // delay
    module,                         // which block
    Cell3DPosition(                // exact destination
      startT.front()[0],
      startT.front()[1] -1 ,
      startT.front()[2]

    )

  )

);

}


void CatomsTest3BlockCode::scheduleOneTeleportT()
{

    getScheduler()->schedule(
  new TeleportationStartEvent(
    getScheduler()->now() + 1000,   // delay
    module,                         // which block
    Cell3DPosition(                // exact destination
      startA.front()[0],
      startA.front()[1]  ,
      startA.front()[2]

    )

  )

);

}


std::vector<Cell3DPosition>
CatomsTest3BlockCode::findOptimalPath(
    const Cell3DPosition &start,
    const Cell3DPosition &goal)
{
    // --- Reset state ---closedSet.clear();
    cameFrom.clear();
    gScore.clear();
    fScore.clear();
    console << "[DEBUG] Entering findOptimalPath from " << start
        << " to " << goal << "\n";

    while (!openSet.empty()) openSet.pop();

    gScore[start] = 0;
    fScore[start] = heuristic(start, goal);
    openSet.push({start, fScore[start]});

    console << "[DEBUG] Pushed start onto openSet with f=" << fScore[start] << "\n";

    while (!openSet.empty()) {
        // 1) Pop
        auto topPair = openSet.top();
        openSet.pop();
        Cell3DPosition current = topPair.first;
        double currentF = topPair.second;

        console << "[DEBUG] Popped node " << current
                << "  f=" << currentF
                << "  g=" << gScore[current]
                << "  h=" << heuristic(current, goal)
                << "\n";

        // 2) Check goal
        if (current == goal) {
            console << "[DEBUG] current == goal! Reconstructing path...\n";
            std::vector<Cell3DPosition> path;
            for (Cell3DPosition p = goal; cameFrom.count(p); p = cameFrom[p]) {
                path.push_back(p);
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        closedSet.insert(current);

        // 3) Generate neighbors
        std::vector<std::pair<short, short> > links;
        std::vector<Cell3DPosition> neighbors;
        bool any = getAllPossibleMotionsFromPosition(current, links, neighbors);

        console << "[DEBUG]   getAllPossibleMotionsFromPosition("
                << current << ") ➔ found=" << any
                << "  neighbors=" << neighbors.size() << "\n";
        for (auto &n: neighbors) {
            console << "        neighbor: " << n << "\n";
        }

        // 4) Try to enqueue each neighbor
        for (auto &neighbor: neighbors) {
            if (closedSet.count(neighbor)) {
                console << "          skipping " << neighbor
                        << " (already in closedSet)\n";
                continue;
            }
            double tentative = gScore[current] + 1;
            if (!gScore.count(neighbor) || tentative < gScore[neighbor]) {
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentative;
                fScore[neighbor] = tentative + heuristic(neighbor, goal);

                console << "          [ENQUEUE] " << neighbor
                        << " ⟵ " << current
                        << "  (g=" << gScore[neighbor]
                        << ", f=" << fScore[neighbor] << ")\n";

                openSet.push({neighbor, fScore[neighbor]});
            }
        }
    }

    // If we exit the loop without ever popping `goal`, we return {}
    console << "[DEBUG] openSet empty, goal never reached → returning empty.\n";
    return {};
}


//since the gettPos works, apply astra