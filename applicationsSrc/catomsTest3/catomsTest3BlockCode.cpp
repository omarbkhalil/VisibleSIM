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


std::map<Cell3DPosition, std::string> CatomsTest3BlockCode::dPhaseAssignment;

std::vector<Cell3DPosition> CatomsTest3BlockCode::dPhasePath;
bool CatomsTest3BlockCode::tPhasePathSaved = false;
bool CatomsTest3BlockCode::aPhasePathSaved = false;

bool CatomsTest3BlockCode::finishedD = false;
bool CatomsTest3BlockCode::finishedT = false;
bool CatomsTest3BlockCode::finishedA = false;

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

//can do dynamic queues if not put in future work
std::queue<Cell3DPosition> CatomsTest3BlockCode::startD(
    std::deque{
      Cell3DPosition(Origin[0] + 3, Origin[1] + 1, Origin[2] - 1),

         Cell3DPosition(Origin[0] + 3, Origin[1] + 1, Origin[2] + 1),
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
        Cell3DPosition(Origin[0]  , Origin[1] , Origin[2] )
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
        Cell3DPosition(Origin[0] -5 , Origin[1]  , Origin[2] )
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

//Maybe,add a flag, when startup finishs it flips, so next module executes A* in Event
void CatomsTest3BlockCode::startup() {
    console << "start\n";
    std::cout << "Working directory: " << std::filesystem::current_path() << std::endl;

    if (!module) {
        console << "Error: module pointer is null in startup().\n";
        return;
    }

    static bool pathAlreadyComputed = false;

    // ─── PATH PLANNER ───
    if (!pathAlreadyComputed && !targetD.empty() && module->position == targetD.front()) {
        console << "[startup] This module (" << module->position << ") is computing both D-phase paths.\n";

        Cell3DPosition goal(
            targetD.front()[0],
            targetD.front()[1] - 1,
            targetD.front()[2]
        );

        // ⬇️ DOWNWARD path (first in startD)
        if (startD.size() >= 1) {
            Cell3DPosition startDown = startD.front();
            std::vector<Cell3DPosition> pathDown = findOptimalPath(startDown, goal);

            if (pathDown.size() >= 2) {
                saveOptimalPath(pathDown, "D-phase-down");
                dPhaseAssignment[pathDown.front()] = "down";
                console << "[startup] Saved D-phase-DOWN path (" << pathDown.size() << " steps):\n";
                for (const auto& pos : pathDown)
                    console << "  " << pos << "\n";
            } else {
                console << "Warning: No valid DOWN path found.\n";
            }
        }

        // ⬆️ UPWARD path (second in startD)
        if (startD.size() >= 2) {
            Cell3DPosition first = startD.front(); startD.pop();
            Cell3DPosition startUp = startD.front(); startD.push(first);

            std::vector<Cell3DPosition> pathUp = findOptimalPath(startUp, goal);

            if (pathUp.size() >= 2) {
                saveOptimalPath(pathUp, "D-phase-up");
                dPhaseAssignment[pathUp.front()] = "up";
                console << "[startup] Saved D-phase-UP path (" << pathUp.size() << " steps):\n";
                for (const auto& pos : pathUp)
                    console << "  " << pos << "\n";
            } else {
                console << "Warning: No valid UP path found.\n";
            }
        } else {
            console << "Warning: Not enough modules in startD to compute both paths.\n";
        }

        pathAlreadyComputed = true;
        return;
    }

    // ─── PATH EXECUTOR (AUTO-DETECT TRACK) ───
    if (pathAlreadyComputed) {
        // ✅ Only allow the module at front of startD to proceed
        if (startD.empty() || module->position != startD.front()) {
            console << "[startup] Module " << module->position << " is waiting. Not first in startD queue.\n";
            hostBlock->setColor(LIGHTGREY);
            distance = -1;
            return;
        }

        // Load both paths
        auto pathUp = loadOptimalPathFromFile("D-phase-up");
        auto pathDown = loadOptimalPathFromFile("D-phase-down");

        // Detect which path the module is on
        auto itUp = std::find(pathUp.begin(), pathUp.end(), module->position);
        auto itDown = std::find(pathDown.begin(), pathDown.end(), module->position);

        if (itUp != pathUp.end()) {
            console << "[startup] Module " << module->position << " is on D-phase-UP path.\n";
            dPhasePath = pathUp;
            dPhaseIndex = std::distance(pathUp.begin(), itUp);
        } else if (itDown != pathDown.end()) {
            console << "[startup] Module " << module->position << " is on D-phase-DOWN path.\n";
            dPhasePath = pathDown;
            dPhaseIndex = std::distance(pathDown.begin(), itDown);
        } else {
            console << "[startup] Module " << module->position << " is passive (not on any D-phase path).\n";
            hostBlock->setColor(LIGHTGREY);
            distance = -1;
            return;
        }

        // Start motion
        finishedD = false;
        distance = 0;
        module->setColor(RED);

        if (dPhaseIndex + 1 >= dPhasePath.size()) {
            console << "[startup] Already at final position of D-phase path.\n";
            finishedD = true;
            return;
        }

        Cell3DPosition nextStep = dPhasePath[dPhaseIndex + 1];
        console << "[startup] Scheduling next D-phase hop to " << nextStep << "\n";
        getScheduler()->schedule(
            new Catoms3DRotationStartEvent(getScheduler()->now() + 1000, module, nextStep)
        );
        return;
    }


    // ─── Passive fallback ───
    console << "[startup] Module " << module->position << " is passive.\n";
    distance = -1;
    hostBlock->setColor(LIGHTGREY);
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
    // if (finishedD && finishedT && !startedA) {
    //     startedA = true;
    //     console << "[#34] Beginning A-phase now that D & T are both done.\n";
    //      // scheduleOneTeleportT();  // go to A-PHASE starter
    //     // finishedT = true;
    //
    // }

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


        case EVENT_ROTATION3D_END: {
            if (!finishedD) {
                if (dPhasePath.empty()) {
                    console << "[#34] Error: dPhasePath is empty at EVENT_ROTATION3D_END\n";
                    return;
                }

                if (dPhaseIndex + 1 < dPhasePath.size()) {
                    ++dPhaseIndex;

                    // Keep skipping if same as current position
                    while (dPhaseIndex < dPhasePath.size() &&
                           dPhasePath[dPhaseIndex] == module->position) {
                        console << "[#34] Skipping redundant hop to " << dPhasePath[dPhaseIndex] << "\n";
                        ++dPhaseIndex;
                           }

                    if (dPhaseIndex >= dPhasePath.size()) {
                        console << "[#34] D-phase complete at " << module->position << " (no more useful steps)\n";
                        finishedD = true;
                     //  scheduleOneTeleportD();
                        return;
                    }

                    Cell3DPosition nextHop = dPhasePath[dPhaseIndex];
                    console << "[#34] Scheduling next D-phase hop to " << nextHop << "\n";
                    getScheduler()->schedule(
                        new Catoms3DRotationStartEvent(getScheduler()->now() + 1000, module, nextHop)
                    );
                } else {
                    console << "[#34] D-phase complete at " << module->position << "\n";
                    finishedD = true;
                    initiateNextModulePathfinding();
                    // scheduleOneTeleportD();


                    if (!startD.empty() && module->position == startD.front()) {
                        startD.pop();
                        console << "[#34] Module " << module->position << " popped from startD queue.\n";
                    }

                }

            }





// i think we need to do y+1 (app crashed)
if (finishedD && !finishedT && !finishedA) {
    // ──────────────── COMPUTE ONCE ────────────────
if (!tPhasePathSaved && finishedD && !finishedT && !finishedA) {

        // Only the leader (at targetD.front()) computes the T-phase path
        tPhasePath = findOptimalPath( module->position, targetT.front());

        if (tPhasePath.empty()) {
            console << "[#" << module->blockId << "] No T-phase path found, abort.\n";
            return;
        }

        console << "[#" << module->blockId << "] Computed T-phase path (" << tPhasePath.size() << " steps):\n";
        for (const auto &pos : tPhasePath)
            console << pos << "\n";

        saveOptimalPath(tPhasePath, "T-phase");
        tPhasePathSaved = true;

        tPhaseIndex = 1; // Start from the second node
        Cell3DPosition firstHop = tPhasePath[tPhaseIndex];
        console << "[#" << module->blockId << "] Scheduling T-phase first hop to " << firstHop << "\n";
        getScheduler()->schedule(
            new Catoms3DRotationStartEvent(getScheduler()->now() + 1000, module, firstHop)
        );
        return;
    }

    // ──────────────── LOAD FROM FILE ────────────────
    if (tPhasePath.empty()) {
        tPhasePath = loadOptimalPathFromFile("T-phase");

        if (tPhasePath.empty()) {
            console << "[#" << module->blockId << "] Failed to load T-phase path.\n";
            return;
        }

        console << "[#" << module->blockId << "] Loaded T-phase path (" << tPhasePath.size() << " steps).\n";

        // Force all modules to follow the full path (start from beginning)
        tPhaseIndex = 0;
        // Determine starting index based on current position
        auto it = std::find(tPhasePath.begin(), tPhasePath.end(), module->position);
        if (it != tPhasePath.end())
            tPhaseIndex = std::distance(tPhasePath.begin(), it);
        else {
            console << "[#" << module->blockId << "] Current position not on T-phase path.\n";
            return;
        }
    }

    // ──────────────── FOLLOW PATH ────────────────
    if (tPhaseIndex + 1 < tPhasePath.size()) {
        ++tPhaseIndex;
        Cell3DPosition nextHop = tPhasePath[tPhaseIndex];
        console << "[#" << module->blockId << "] Scheduling next T-phase hop to " << nextHop << "\n";
        getScheduler()->schedule(
            new Catoms3DRotationStartEvent(getScheduler()->now() + 1000, module, nextHop)
        );
    } else {
        console << "[#" << module->blockId << "] T-phase complete at " << module->position << "\n";
         finishedT = true;
        startedA = true;

         scheduleOneTeleportT();

    }

    return;
}



    if (startedA && !finishedA) {
    // ────── COMPUTE ONCE (first module after T-phase) ──────
    if (!aPhasePathSaved) {
        aPhasePath = findOptimalPath(module->position, targetA.front());

        if (aPhasePath.empty()) {
            console << "[#" << module->blockId << "] No A-phase path found, abort.\n";
            return;
        }

        console << "[#" << module->blockId << "] Computed A-phase path (" << aPhasePath.size() << " steps):\n";
        for (const auto &pos : aPhasePath)
            console << pos << "\n";

        saveOptimalPath(aPhasePath, "A-phase");
        aPhasePathSaved = true;

        aPhaseIndex = 1;
        Cell3DPosition firstHop = aPhasePath[aPhaseIndex];
        console << "[#" << module->blockId << "] Scheduling A-phase first hop to " << firstHop << "\n";
        getScheduler()->schedule(
            new Catoms3DRotationStartEvent(
                getScheduler()->now() + 1000, module, firstHop
            )
        );
        return;
    }

    // ────── LOAD FROM FILE (for all other modules) ──────
    if (aPhasePath.empty()) {
        aPhasePath = loadOptimalPathFromFile("A-phase");

        if (aPhasePath.empty()) {
            console << "[#" << module->blockId << "] Failed to load A-phase path.\n";
            return;
        }

        console << "[#" << module->blockId << "] Loaded A-phase path (" << aPhasePath.size() << " steps).\n";

        auto it = std::find(aPhasePath.begin(), aPhasePath.end(), module->position);
        if (it != aPhasePath.end()) {
            aPhaseIndex = std::distance(aPhasePath.begin(), it);
        } else {
            console << "[#" << module->blockId << "] Not on A-phase path → teleporting to startA.front()\n";
            getScheduler()->schedule(
                new Catoms3DRotationStartEvent(
                    getScheduler()->now() + 1000,
                    module,
                    startA.front()
                )
            );
            return;
        }

    }

    // ────── FOLLOW A-phase path ──────
    if (aPhaseIndex + 1 < aPhasePath.size()) {
        ++aPhaseIndex;
        Cell3DPosition nextHop = aPhasePath[aPhaseIndex];
        console << "[#" << module->blockId << "] Scheduling next A-phase hop to " << nextHop << "\n";
        getScheduler()->schedule(
            new Catoms3DRotationStartEvent(
                getScheduler()->now() + 1000, module, nextHop
            )
        );
    } else {
        console << "[#" << module->blockId << "] A-phase complete at " << module->position << "\n";
        finishedA = true;
        initiateNextModulePathfinding();
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



    for (size_t i = startIndex; i < path.size(); ++i) {
        outFile << path[i] << "\n";
    }
    outFile << "\n";  // blank line to separate entries
    outFile.close();

    console << phaseLabel << " path saved to optimal_paths.txt\n";
}


std::vector<Cell3DPosition> CatomsTest3BlockCode::loadOptimalPathFromFile(const std::string& phaseLabel) {
    std::ifstream inFile("optimal_paths.txt");
    std::vector<Cell3DPosition> loadedPath;

    if (!inFile.is_open()) {
        console << "Error: Unable to open optimal_paths.txt for reading.\n";
        return loadedPath;
    }

    std::string line;
    std::string header = phaseLabel + " path from start to goal:";
    bool readingPath = false;

    while (std::getline(inFile, line)) {
        // Skip empty lines
        if (line.empty()) {
            if (readingPath) break; // End of current path section
            continue;
        }

        if (!readingPath) {
            if (line == header) {
                readingPath = true;
            }
        } else {
            // Parse line like (35,4,6)
            int x, y, z;
            if (sscanf(line.c_str(), "(%d,%d,%d)", &x, &y, &z) == 3) {
                loadedPath.emplace_back(x, y, z);
            } else {
                console << "Warning: Failed to parse line: " << line << "\n";
            }
        }
    }

    if (loadedPath.empty()) {
        console << "Warning: No path found for " << phaseLabel << "\n";
    } else {
        console << phaseLabel << " path loaded from file (" << loadedPath.size() << " steps).\n";
    }

    return loadedPath;
}





void CatomsTest3BlockCode::initiateNextModulePathfinding() {
    if (startD.empty()) {
        console << "No more modules in startQueue.\n";
        return;
    }

    // Get next module
    Cell3DPosition nextStart = startD.front();
    startD.pop();

    BuildingBlock *nextBlock = BaseSimulator::getWorld()->getBlockByPosition(nextStart);
    if (!nextBlock || !nextBlock->blockCode) {
        console << "Error: Block at " << nextStart << " is null or has no blockCode.\n";
        return;
    }

    finishedD = false;
    finishedT = false;
    finishedA = false;

    CatomsTest3BlockCode *nextModule = static_cast<CatomsTest3BlockCode *>(nextBlock->blockCode);



    // Determine which path this module should follow
    auto pathUp = loadOptimalPathFromFile("D-phase-up");
    auto pathDown = loadOptimalPathFromFile("D-phase-down");

    if (!pathUp.empty() && pathUp.front() == nextStart) {
        nextModule->dPhasePath = pathUp;
        nextModule->dPhaseIndex = 0;
        nextModule->finishedD = false;
        console << "[INIT] Assigned D-phase-UP to " << nextStart << "\n";
    } else if (!pathDown.empty() && pathDown.front() == nextStart) {
        nextModule->dPhasePath = pathDown;
        nextModule->dPhaseIndex = 0;
        nextModule->finishedD = false;
        console << "[INIT] Assigned D-phase-DOWN to " << nextStart << "\n";
    } else {
        console << "[INIT] No matching D-phase path for " << nextStart << "\n";
        return;
    }

    nextModule->distance = 0;
    nextModule->module->setColor(RED);

    // Schedule first move
    if (nextModule->dPhasePath.size() > 1) {
        Cell3DPosition firstHop = nextModule->dPhasePath[1];
        console << "[INIT] Scheduling first hop to " << firstHop << "\n";
        getScheduler()->schedule(
            new Catoms3DRotationStartEvent(getScheduler()->now() + 1000, nextModule->module, firstHop)
        );
    } else {
        console << "[INIT] Path too short, skipping.\n";
        nextModule->finishedD = true;
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
  new Catoms3DRotationStartEvent(
    getScheduler()->now() + 1000,   // delay
    module,                         // which block
    Cell3DPosition(                // exact destination
      startT.front()[0],
      startT.front()[1]-1  ,
      startT.front()[2]

    )

  )

);

}


void CatomsTest3BlockCode::scheduleOneTeleportT()
{

    getScheduler()->schedule(
  new Catoms3DRotationStartEvent(
    getScheduler()->now() + 1000,   // delay
    module,                         // which block
    Cell3DPosition(                // exact destination
      startA.front()[0] ,
      startA.front()[1] +1  ,
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
    // Reset
    cameFrom.clear();
    gScore.clear();
    fScore.clear();
    console << "Entering findOptimalPath from " << start
        << " to " << goal << "\n";

    while (!openSet.empty()) openSet.pop();

    gScore[start] = 0;
    fScore[start] = heuristic(start, goal);
    openSet.push({start, fScore[start]});

    console << "Pushed start onto openSet with f=" << fScore[start] << "\n";

    while (!openSet.empty()) {
        // 1) Pop
        auto topPair = openSet.top();
        openSet.pop();
        Cell3DPosition current = topPair.first;
        double currentF = topPair.second;

        console << " Popped node " << current
                << "  f=" << currentF
                << "  g=" << gScore[current]
                << "  h=" << heuristic(current, goal)
                << "\n";

        // 2) Check goal
        if (current == goal) {
            console << " current == goal! Reconstructing path...\n";
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

        console << "getAllPossibleMotionsFromPosition("
                << current << ") ➔ found=" << any
                << "  neighbors=" << neighbors.size() << "\n";
        for (auto &n: neighbors) {
            console << "        neighbor: " << n << "\n";
        }

        // 4) to enqueue each neighbor
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

    // If we exit the loop without ever popping goal
    console << " openSet empty, goal never reached → returning empty.\n";
    return {};
}


//we can get farthest path, and all use it , or in limitation
//only 1 class, no need for 2 since we have getallPossibleMotion
//try to let metaModules itself compute
//
//Always choosing downpath for first StartD