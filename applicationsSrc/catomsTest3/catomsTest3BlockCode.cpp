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

bool CatomsTest3BlockCode::dPathsComputed2 = false;
// bool CatomsTest3BlockCode::dPathsComputed = false;

std::map<int, int> CatomsTest3BlockCode::moduleToAPathIndex;

bool CatomsTest3BlockCode::startedA = false;
bool CatomsTest3BlockCode::finishedD = false;
bool CatomsTest3BlockCode::finishedT = false;
bool CatomsTest3BlockCode::finishedA = false;
bool CatomsTest3BlockCode::onlyone = false;

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
std::map<int, Cell3DPosition> CatomsTest3BlockCode::startDOrigin;

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
         Cell3DPosition(Origin[0] + 3, Origin[1] + 2, Origin[2]),

                Cell3DPosition(Origin[0] + 2, Origin[1] + 1, Origin[2] - 1),
               Cell3DPosition(Origin[0] + 2, Origin[1] + 1, Origin[2] - 2),
               Cell3DPosition(Origin[0] + 2, Origin[1] + 1, Origin[2] + 1),
               Cell3DPosition(Origin[0] + 2, Origin[1] + 1, Origin[2] + 2),
               Cell3DPosition(Origin[0] + 1, Origin[1] + 1, Origin[2] + 2),
               Cell3DPosition(Origin[0] + 1, Origin[1] + 1, Origin[2] - 2),
               Cell3DPosition(Origin[0], Origin[1], Origin[2] + 1),
               Cell3DPosition(Origin[0], Origin[1], Origin[2] - 1),

       // Cell3DPosition(Origin[0], Origin[1], Origin[2]),

    }
  );

std::queue<Cell3DPosition> CatomsTest3BlockCode::targetD(
    std::deque{
// x should be -1
        Cell3DPosition(Origin[0]  , Origin[1] , Origin[2] )
    }
  );


std::queue<Cell3DPosition> CatomsTest3BlockCode::startD1(
    std::deque{
        Cell3DPosition(Origin[0] -1  , Origin[1] , Origin[2] - 1),
        Cell3DPosition(Origin[0] -1  , Origin[1] , Origin[2] + 1),

        Cell3DPosition(Origin[0] -1  , Origin[1] , Origin[2] ),
        Cell3DPosition(Origin[0] -2  , Origin[1] , Origin[2]+1 ),
        Cell3DPosition(Origin[0] -2  , Origin[1] , Origin[2]-1 ),

        Cell3DPosition(Origin[0] -2  , Origin[1] + 1, Origin[2] + 2 ),
        Cell3DPosition(Origin[0] -2  , Origin[1] + 1, Origin[2] - 2 ),

        Cell3DPosition(Origin[0] -3  , Origin[1] + 1, Origin[2] + 2 ),
       Cell3DPosition(Origin[0] -3  , Origin[1] + 1, Origin[2] - 2 ),

        Cell3DPosition(Origin[0] -4  , Origin[1] + 1, Origin[2] + 1 ),
        Cell3DPosition(Origin[0] -4  , Origin[1] + 1, Origin[2] - 1 ),
        // Cell3DPosition(Origin[0] -4  , Origin[1] + 2, Origin[2]  ),



       // Cell3DPosition(Origin[0], Origin[1], Origin[2]),

    }
  );

std::queue<Cell3DPosition> CatomsTest3BlockCode::targetD1(
    std::deque{
// x should be -1
        Cell3DPosition(Origin[0] -4  , Origin[1] + 2, Origin[2]  )
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

// std::queue<Cell3DPosition> CatomsTest3BlockCode::targetA;

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
        Cell3DPosition(Origin[0] -9 , Origin[1] , Origin[2]-1 ),
        Cell3DPosition(Origin[0] -9 , Origin[1] , Origin[2]+1 ),
         Cell3DPosition(Origin[0] -9 , Origin[1] , Origin[2] ),
        Cell3DPosition(Origin[0] -10 , Origin[1] , Origin[2]-1 ),
        Cell3DPosition(Origin[0] -10 , Origin[1] , Origin[2]+1 ),
        Cell3DPosition(Origin[0] -10 , Origin[1]+1 , Origin[2]+2 ),
        Cell3DPosition(Origin[0] -10 , Origin[1]+1 , Origin[2]-2 ),
        Cell3DPosition(Origin[0] -11 , Origin[1]+1 , Origin[2]+2 ),
              Cell3DPosition(Origin[0] -11 , Origin[1]+1 , Origin[2]-2 ),
        Cell3DPosition(Origin[0] -12 , Origin[1]+1 , Origin[2]+1 ),
                    Cell3DPosition(Origin[0] -12 , Origin[1]+1 , Origin[2]-1 ),

        Cell3DPosition(Origin[0] -12 , Origin[1]+2 , Origin[2] ),

        //
        //
        //
    }
  );



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

    static bool dPathsComputed = false;
    static bool tPathComputed = false;

    // ─── D-PHASE PATH PLANNER ───
    if ((!dPathsComputed || !dPathsComputed2) && !targetD.empty() && module->position == targetD.front()) {
        console << "[startup] Computing D-phase paths...\n";
        Cell3DPosition goal = targetD.front();
        std::deque<Cell3DPosition> tempStartD;

        int i = 0;
        while (i < 12 && !startD.empty()) {
            Cell3DPosition start = startD.front(); startD.pop();
            tempStartD.push_back(start);

            std::vector<Cell3DPosition> path = findOptimalPath(start, Cell3DPosition(goal[0], goal[1] - 1, goal[2]));
            if (path.size() >= 2) {
                std::stringstream filename;
                filename << "D-phase-" << i;
                saveOptimalPath(path, filename.str());
                console << "[startup] Saved " << filename.str() << " (" << path.size() << " steps).\n";
            } else {
                console << "[startup]  No valid path from " << start << " to " << goal << "\n";
            }
            ++i;
        }

        for (const auto& pos : tempStartD) startD.push(pos);

        if (!dPathsComputed)
            dPathsComputed = true;


        return;
    }

    // ─── T-PHASE PATH PLANNER ───
    if (!tPathComputed && !startT.empty() && module->position == startT.front()) {
        console << "[startup] Computing T-phase path from " << module->position << " to " << targetT.front() << "\n";
        module->setColor(DARKGREY);  // Visual marker for T-phase planner

        Cell3DPosition startPos = Cell3DPosition(targetD.front()[0], targetD.front()[1]-1, targetD.front()[2]);

        tPhasePath = findOptimalPath(startPos, targetT.front());

        if (tPhasePath.empty()) {
            console << "[startup]  Failed to compute T-phase path.\n";
        } else {
            saveOptimalPath(tPhasePath, "T-phase");
            tPhasePathSaved = true;
            console << "[startup] Saved T-phase path (" << tPhasePath.size() << " steps).\n";
        }

        tPathComputed = true;
        return;
    }

    // ─── PATH EXECUTOR (AUTO-DETECT TRACK) ───
    if (dPathsComputed) {
        if (startD.empty() || module->position != startD.front()) {
            console << "[startup] Module " << module->position << " is waiting. Not first in startD queue.\n";
            hostBlock->setColor(LIGHTGREY);
            distance = -1;
            return;
        }

        bool matched = false;
        for (int i = 0; i < 11; ++i) {
            std::stringstream filename;
            filename << "D-phase-" << i;
            std::vector<Cell3DPosition> path = loadOptimalPathFromFile(filename.str());

            auto it = std::find(path.begin(), path.end(), module->position);
            if (it != path.end()) {
                console << "[startup] Module " << module->position << " is on " << filename.str()
                        << " at index " << std::distance(path.begin(), it) << "\n";
                dPhasePath = path;
                dPhaseIndex = std::distance(path.begin(), it);
                matched = true;
                break;
            }
        }

        if (!matched) {
            console << "[startup] Module " << module->position << " is passive (not on any D-phase path).\n";
            hostBlock->setColor(LIGHTGREY);
            distance = -1;
            return;
        }

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


        case EVENT_ROTATION3D_END: {
            if (!finishedD) {
                startDOrigin[module->blockId] = module->position; // when D starts

                setColor(BLUE);
                if (dPhasePath.empty()) {
                    console << "[#" << module->blockId << "] dPhasePath is empty. Attempting to load from file...\n";

                    bool matched = false;
                    for (int i = 0; i < 12; ++i) {
                        std::stringstream filename;
                        filename << ("D1-phase-") << i;
                        std::vector<Cell3DPosition> path = loadOptimalPathFromFile(filename.str());

                        auto it = std::find(path.begin(), path.end(), module->position);
                        if (it != path.end()) {
                            dPhasePath = path;
                            dPhaseIndex = std::distance(path.begin(), it);
                            matched = true;
                            console << "[#" << module->blockId << "] Loaded " << filename.str()
                                    << " and set dPhaseIndex to " << dPhaseIndex << "\n";
                            break;
                        }
                    }

                    if (!matched) {
                        console << "[#" << module->blockId << "] Module position not found in any path.\n";
                        return;
                    }
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
                      scheduleOneTeleportD();
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
                    // initiateNextModulePathfinding();
                    // scheduleOneTeleportD();


                    if (startD.size()==1 ) {
                        startD.pop();
                    }


                }

            }





// i think we need to do y+1 (app crashed)
            if (finishedD && !finishedT && !finishedA) {
                setColor(GREEN);
                if(dPathsComputed2) {
                    // dPathsComputed2 = false;


                }
                else {
                // ──────────────── LOAD FROM FILE ────────────────
                if (tPhasePath.empty()) {
                    tPhasePath = loadOptimalPathFromFile("T-phase");

                    if (tPhasePath.empty()) {
                        console << "[#" << module->blockId << "] Failed to load T-phase path.\n";
                        return;
                    }

                    console << "[#" << module->blockId << "] Loaded T-phase path (" << tPhasePath.size() << " steps).\n";

                    // Determine starting index based on current position
                    auto it = std::find(tPhasePath.begin(), tPhasePath.end(), module->position);
                    if (it != tPhasePath.end()) {
                        tPhaseIndex = std::distance(tPhasePath.begin(), it);
                    } else {
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
                    // ++tPhaseIndex;

                } else {
                    console << "[#" << module->blockId << "] T-phase complete at " << module->position << "\n";
                    finishedT = true;
                    startedA = true;

                    if (!onlyone) {
                        // Add logic here if needed
                    }

                    scheduleOneTeleportT();
                }
                }
                return;
            }








if (startedA && !finishedA) {
    setColor(YELLOW);
    // ────── COMPUTE ONCE ──────
    if (aPhasePath.empty()) {
        if (!targetA.empty()) {
            Cell3DPosition goal = targetA.front(); targetA.pop();
            Cell3DPosition start = startA.front();

            console << "[#" << module->blockId << "] Computing A-phase path from "
                    << start << " to " << goal << "\n";

            std::vector<Cell3DPosition> path = findOptimalPath(start, goal);
            if (path.empty()) {
                console << "[#" << module->blockId << "]  Could not compute A-phase path to "
                        << goal << "\n";
                finishedA = true;
                return;
            }

            if (path[0] == module->position) {
                path.erase(path.begin());
            }

            std::stringstream filename;
            filename << "A-phase-" << module->blockId;
            saveOptimalPath(path, filename.str());

            console << "[#" << module->blockId << "] 💾 Saved A-phase-" << module->blockId
                    << " with " << path.size() << " steps.\n";

            aPhasePathSaved = true;
        } else {
            console << "[#" << module->blockId << "] ️ No A-phase targets available.\n";
            finishedA = true;
            return;
        }
    }

    // ────── LOAD ──────
    if (aPhasePathSaved && aPhasePath.empty()) {
        std::stringstream filename;
        filename << "A-phase-" << module->blockId;
        aPhasePath = loadOptimalPathFromFile(filename.str());

        if (aPhasePath.empty()) {
            console << "[#" << module->blockId << "] Failed to load A-phase path from file\n";
            finishedA = true;
            return;
        }

        console << "[#" << module->blockId << "]  Loaded A-phase path with "
                << aPhasePath.size() << " steps.\n";
    }
    ++aPhaseIndex;

    // ────── FOLLOW ──────
    if (aPhaseIndex < aPhasePath.size()) {
        Cell3DPosition nextHop = aPhasePath[aPhaseIndex];
        console << "[#" << module->blockId << "] A-phase step to " << nextHop << "\n";

        getScheduler()->schedule(
            new Catoms3DRotationStartEvent(getScheduler()->now() + 1000, module, nextHop)
        );
    } else {
        console << "[#" << module->blockId << "] A-phase complete at " << module->position << "\n";
        finishedA = true;

        // ────── When All Modules Done ──────
        if (startD.empty() && targetD.empty()) {

            // ─── D1-PHASE PATH PLANNER ───
                console << "[startup] Computing D1-phase paths...\n";


            Cell3DPosition nextTarget = targetD1.front();
            BuildingBlock *targetBlock = BaseSimulator::getWorld()->getBlockByPosition(nextTarget);
            CatomsTest3BlockCode *targetModule = static_cast<CatomsTest3BlockCode *>(targetBlock->blockCode);
            targetModule->computeD1PhasePaths();


                    dPathsComputed2 = true;

            console << "[DEBUG] ✅ All modules have finished A-phase.\n";

            // Print queues BEFORE refill
            std::queue<Cell3DPosition> tempStartD = startD;
            console << "[DEBUG] Final startD queue:\n";
            while (!tempStartD.empty()) {
                console << "  " << tempStartD.front() << "\n";
                tempStartD.pop();
            }

            std::queue<Cell3DPosition> tempTargetD = targetD;
            console << "[DEBUG] Final targetD queue:\n";
            while (!tempTargetD.empty()) {
                console << "  " << tempTargetD.front() << "\n";
                tempTargetD.pop();
            }

            // ────── Start Phase 2 with startD1/targetD1 ──────
            if (!startD1.empty() && !targetD1.empty()) {
                console << "[DEBUG] 🌀 Starting second round using startD1 and targetD1\n";

                while (!startD1.empty()) {
                    startD.push(startD1.front());
                    startD1.pop();
                }


                while (!targetD1.empty()) {
                    targetD.push(targetD1.front());
                    targetD1.pop();
                }

                // Print queues AFTER refill
                std::queue<Cell3DPosition> newStartD = startD;
                console << "[DEBUG] ➕ Refilled startD queue:\n";
                while (!newStartD.empty()) {
                    console << "  " << newStartD.front() << "\n";
                    newStartD.pop();
                }

                std::queue<Cell3DPosition> newTargetD = targetD;
                console << "[DEBUG] ➕ Refilled targetD queue:\n";
                while (!newTargetD.empty()) {
                    console << "  " << newTargetD.front() << "\n";
                    newTargetD.pop();
                }


                // Trigger the next module
                setColor(RED);

                initiateNextModulePathfinding();
                return;
            }
        }



        setColor(RED);
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

void CatomsTest3BlockCode::computeD1PhasePaths() {
    setColor(DARKGREY);
    console << "[D1-PLANNER] Computing D1-phase paths...\n";
    Cell3DPosition goal = targetD1.front();
    std::deque<Cell3DPosition> tempStartD1;

    int i = 0;
    while (i < 12 && !startD1.empty()) {
        Cell3DPosition start = startD1.front(); startD1.pop();
        tempStartD1.push_back(start);

        std::vector<Cell3DPosition> path = findOptimalPath(start, Cell3DPosition(goal[0], goal[1] - 1, goal[2]));
        if (path.size() >= 2) {
            std::stringstream filename;
            filename << "D1-phase-" << i;
            saveOptimalPath(path, filename.str());
            console << "[D1-PLANNER] Saved " << filename.str() << " (" << path.size() << " steps).\n";
        } else {
            console << "[D1-PLANNER] No valid path from " << start << " to " << goal << "\n";
        }
        ++i;
    }

    for (const auto &pos : tempStartD1)
        startD1.push(pos);
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

        if (line == header) {
            readingPath = true;
            loadedPath.clear();  // Reset in case of prior header matches
            continue;
        }

        if (readingPath) {
            if (line.empty()) break;

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
        if (targetD.empty()) {
            console << "[INIT] Both startD and targetD are empty. No modules left to process.\n";
            return;
        }

        // Handle the single module in targetD
        Cell3DPosition nextTarget = targetD.front();
        BuildingBlock *targetBlock = BaseSimulator::getWorld()->getBlockByPosition(nextTarget);

        CatomsTest3BlockCode *targetModule = static_cast<CatomsTest3BlockCode *>(targetBlock->blockCode);

        targetModule->finishedD = true;  // Assume D-phase done
        targetModule->finishedT = false;
        targetModule->finishedA = false;
        targetModule->startedA = false;

        console << "[INIT] TargetD module activated at position: " << nextTarget << ". Starting T-phase.\n";

        // Load and initiate T-phase directly here
        targetModule->tPhasePath = loadOptimalPathFromFile("T-phase");
        targetModule->tPhaseIndex = 1;

        if (targetModule->tPhasePath.size() > 1) {
            Cell3DPosition nextHop = targetModule->tPhasePath[1]; // Start from second index
            console << "[INIT] Scheduling first T-phase hop to " << nextHop << "\n";
            getScheduler()->schedule(
                new Catoms3DRotationStartEvent(getScheduler()->now() + 1000, targetModule->module, nextHop)
            );
        }

        targetD.pop();
        return;
    }
    startD.pop();

    // Normal D-phase handling
    Cell3DPosition nextStart = startD.front();
    BuildingBlock *nextBlock = BaseSimulator::getWorld()->getBlockByPosition(nextStart);

    if (!nextBlock || !nextBlock->blockCode) {
        console << "[INIT] Error: Block at " << nextStart << " is null or has no blockCode.\n";
        startD.pop();
        return;
    }

    CatomsTest3BlockCode *nextModule = static_cast<CatomsTest3BlockCode *>(nextBlock->blockCode);

    nextModule->finishedD = false;
    nextModule->finishedT = false;
    nextModule->finishedA = false;
    nextModule->startedA = false;

    bool matched = false;
    for (int i = 0; i < 12; ++i) {
        std::stringstream filename;
        if (dPathsComputed2)
            filename << "D1-phase-" << i;
        else
            filename << "D-phase-" << i;

        std::vector<Cell3DPosition> path = loadOptimalPathFromFile(filename.str());

        auto it = std::find(path.begin(), path.end(), nextStart);
        if (it != path.end()) {
            nextModule->dPhasePath = path;
            nextModule->dPhaseIndex = std::distance(path.begin(), it);
            console << "[INIT] Assigned " << filename.str()
                    << " to " << nextStart
                    << " at index " << nextModule->dPhaseIndex << "\n";
            matched = true;
            break;
        }
    }

    if (!matched) {
        console << "[INIT] No matching D-phase path for " << nextStart << "\n";
        startD.pop();
        return;
    }

    nextModule->distance = 0;
    nextModule->module->setColor(RED);

    if (nextModule->dPhaseIndex + 1 < nextModule->dPhasePath.size()) {
        Cell3DPosition nextHop = nextModule->dPhasePath[nextModule->dPhaseIndex + 1];
        console << "[INIT] Scheduling first hop to " << nextHop << "\n";
        getScheduler()->schedule(
            new Catoms3DRotationStartEvent(getScheduler()->now() + 1000, nextModule->module, nextHop)
        );
    } else {
        console << "[INIT] Already at end of D-phase path. Marking D-phase complete.\n";
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
      startA.front()[0]  ,
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
    closedSet.clear();
    while (!openSet.empty()) openSet.pop();

    console << "Entering findOptimalPath from " << start << " to " << goal << "\n";

    gScore[start] = 0;
    fScore[start] = heuristic(start, goal);
    openSet.push({start, fScore[start]});

    console << "Pushed start onto openSet with f=" << fScore[start] << "\n";

    while (!openSet.empty()) {
        // 1) Pop
        auto topPair = openSet.top();
        openSet.pop();
        Cell3DPosition current = topPair.first;

        console << "Popped node " << current
                << "  g=" << gScore[current]
                << "  h=" << heuristic(current, goal)
                << "\n";

        // 2) Check goal
        if (current == goal) {
            console << "Reached goal! Reconstructing path...\n";
            std::vector<Cell3DPosition> path;
            for (Cell3DPosition p = goal; cameFrom.count(p); p = cameFrom[p]) {
                path.push_back(p);
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());

            //  Validate the entire path for rotation feasibility
            for (size_t i = 0; i + 1 < path.size(); ++i) {
                std::vector<std::pair<short, short>> dummyLinks;
                std::vector<Cell3DPosition> reachable;
                getAllPossibleMotionsFromPosition(path[i], dummyLinks, reachable);
                if (std::find(reachable.begin(), reachable.end(), path[i + 1]) == reachable.end()) {
                    console << " Invalid rotation from " << path[i] << " to " << path[i + 1]
                            << " — aborting path\n";
                    return {};
                }
            }

            return path;
        }

        closedSet.insert(current);

        // 3) Generate neighbors
        std::vector<std::pair<short, short>> links;
        std::vector<Cell3DPosition> neighbors;
        bool any = getAllPossibleMotionsFromPosition(current, links, neighbors);

        console << "getAllPossibleMotionsFromPosition(" << current << ") ➔ "
                << (any ? "valid" : "none") << ", neighbors=" << neighbors.size() << "\n";

        // 4) Enqueue neighbors
        for (auto &neighbor : neighbors) {
            if (closedSet.count(neighbor)) {
                console << "  Skipping " << neighbor << " (already in closedSet)\n";
                continue;
            }

            double tentativeG = gScore[current] + 1;
            if (!gScore.count(neighbor) || tentativeG < gScore[neighbor]) {
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentativeG;
                fScore[neighbor] = tentativeG + heuristic(neighbor, goal);

                console << "  [ENQUEUE] " << neighbor
                        << " ← " << current
                        << " (g=" << gScore[neighbor]
                        << ", f=" << fScore[neighbor] << ")\n";

                openSet.push({neighbor, fScore[neighbor]});
            }
        }
    }

    // If we exit the loop without ever reaching the goal
    console << "openSet exhausted — goal not reachable. Returning empty path.\n";
    return {};
}

bool Cell3DPosition::isAdjacentTo(const Cell3DPosition &other) const {
    int dx = std::abs((*this)[0] - other[0]);
    int dy = std::abs((*this)[1] - other[1]);
    int dz = std::abs((*this)[2] - other[2]);

    return (dx + dy + dz == 1); // 6-connected neighborhood
}

bool CatomsTest3BlockCode::isRotationPossible(const Cell3DPosition& from, const Cell3DPosition& to) {
    std::vector<std::pair<short, short>> links;
    std::vector<Cell3DPosition> neighbors;
    getAllPossibleMotionsFromPosition(from, links, neighbors);

    return std::find(neighbors.begin(), neighbors.end(), to) != neighbors.end();
}



void CatomsTest3BlockCode::initializeTargetA(const std::deque<Cell3DPosition>& tempStartD) {
    console << "[InitTargetA] Called with tempStartD\n";
    for (const auto& pos : tempStartD) {
        Cell3DPosition targetPos(pos[0] - 12, pos[1] -1, pos[2]);
        targetA.push(targetPos);
        console << "[InitTargetA] Added targetA position: " << targetPos << "\n";
    }
}



//we can get farthest path, and all use it , or in limitation
//only 1 class, no need for 2 since we have getallPossibleMotion
//try to let metaModules itself compute
//
//Always choosing downpath for first StartD


//try pop 3 modules
//point to planners in handlers

//can do Tphase planner in startup as i think
// In the Tphase, take the same module structure, apply normal Astar, shift the X and save as a star

//insteaad of adding  TD1 and SD1 manually, maybe can to Atarget+8X