#include "catomsTest3BlockCode.hpp"

#include <queue>

#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>

#include "motion/teleportationEvents.h"
#include "robots/catoms3D/catoms3DMotionEngine.h"
#include "robots/catoms3D/catoms3DRotationEvents.h"


using namespace Catoms3D;



map<Cell3DPosition, vector<Cell3DPosition>> CatomsTest3BlockCode::cells;
vector<Cell3DPosition> CatomsTest3BlockCode::visited;
vector<Cell3DPosition> CatomsTest3BlockCode::teleportedPositions;


std::priority_queue<std::pair<Cell3DPosition, double>, std::vector<std::pair<Cell3DPosition, double>>, std::function<bool(std::pair<Cell3DPosition, double>, std::pair<Cell3DPosition, double>)>> CatomsTest3BlockCode::openSet{
    [](std::pair<Cell3DPosition, double> a, std::pair<Cell3DPosition, double> b) {             return a.second > b.second;
 } // Assuming this comparator logic
};


std::map<Cell3DPosition, double> CatomsTest3BlockCode::gScore;
std::map<Cell3DPosition, double> CatomsTest3BlockCode::fScore;
map<Cell3DPosition, Cell3DPosition> CatomsTest3BlockCode::cameFrom;
set<Cell3DPosition> CatomsTest3BlockCode::closedSet;

std::queue<Cell3DPosition> CatomsTest3BlockCode::startQueue(
    std::deque{
        Cell3DPosition(20, 6, 7),
        Cell3DPosition(20, 6, 5)

    }
);

std::queue<Cell3DPosition> CatomsTest3BlockCode::targetQueue(
    std::deque{
        Cell3DPosition(9, 4, 6),
        Cell3DPosition(9, 4, 7)

    }
);



//const Cell3DPosition CatomsTest3BlockCode::goalPosition = Cell3DPosition(9, 4, 6);


CatomsTest3BlockCode::CatomsTest3BlockCode(Catoms3DBlock *host) : Catoms3DBlockCode(host) {
    if (!host) return;


    module = static_cast<Catoms3DBlock*>(hostBlock);
}


void CatomsTest3BlockCode::startup() {
    console << "start\n";
    static bool hasStarted = false;
    if (hasStarted) {

        distance = -1;
        hostBlock->setColor(LIGHTGREY);
        return;
    }

    if (!startQueue.empty() && (module->position == startQueue.front())) {
        // Mark that the starting module has been activated.
        hasStarted = true;

        Cell3DPosition startTarget = startQueue.front();
        startQueue.pop();

        module->setColor(RED);
        distance = 0;

        Cell3DPosition currentPosition = startTarget;
        cells[currentPosition].clear();  // Clear any previous data
        teleportedPositions.push_back(currentPosition);

        // Initialize for A* pathfinding with targetQueue.front() as goal.
        gScore[currentPosition] = 0;
        fScore[currentPosition] = heuristic(currentPosition, targetQueue.front());

        // Record initial visited nodes.
        for (auto &pos: module->getAllMotions()) {
            cells[currentPosition].push_back(pos.first);
            visited.push_back(pos.first);
        }

        // Push current position to open set with its f score.
        openSet.push({currentPosition, fScore[currentPosition]});

        // Schedule the next move.
        if (!openSet.empty()) {
            auto nextStep = openSet.top();
            openSet.pop();
            getScheduler()->schedule(new TeleportationStartEvent(getScheduler()->now() + 1000, module, nextStep.first));
        }
    } else {
        // Either this module is not on the startQueue front or another module has already started.
        distance = -1;
        hostBlock->setColor(LIGHTGREY);
    }
}



void CatomsTest3BlockCode::onMotionEnd() {
    console << " has reached its destination\n";

}

void CatomsTest3BlockCode::processLocalEvent(EventPtr pev) {
    Catoms3DBlockCode::processLocalEvent(pev);

    // Update the adjacency list for the new position
    Cell3DPosition currentPosition = module->position;
    if (cells.find(currentPosition) == cells.end()) {
        vector<Cell3DPosition> neighbors;
        for (auto &motion : module->getAllMotions()) {
            neighbors.push_back(motion.first);
        }
        cells[currentPosition] = neighbors; // Update the cells map
    }

    switch (pev->eventType) {
        case EVENT_TELEPORTATION_END:
            if (isReturning) {
                if (!discoveredPath.empty()) {
                    Cell3DPosition nextPosition = discoveredPath.back();
                    discoveredPath.pop_back();
                    console << "Returning: Teleporting to " << nextPosition << "\n";
                    getScheduler()->schedule(new TeleportationStartEvent(getScheduler()->now() + 1000, module, nextPosition));
                } else {
                    console << "Return journey complete. Back at the initial position.\n";
                    isReturning = false;
                    // Reset the active module flag so that the next waiting module can start.
                }
            } else {
                // Check if the current position is the goal.
                if ( !targetQueue.empty() && currentPosition == targetQueue.front() ) {
                    targetQueue.pop();
                    console << "Goal reached. Reconstructing optimal path...\n";
                    isReturning = true;
                    discoveredPath.clear();

                    // Backtrack from goal to start using the cameFrom map.
                    while (cameFrom.find(currentPosition) != cameFrom.end()) {
                        discoveredPath.push_back(currentPosition);
                        currentPosition = cameFrom[currentPosition];
                    }
                    discoveredPath.push_back(currentPosition);  // Add the start position.
                    std::reverse(discoveredPath.begin(), discoveredPath.end());

                    console << "Optimal path from start to goal:\n";
                    for (auto &pos : discoveredPath) {
                        console << pos << "\n";
                    }

                    // Begin the return journey.
                    if (!discoveredPath.empty()) {
                        Cell3DPosition nextPosition = discoveredPath.back();
                        discoveredPath.pop_back();
                        console << "Starting return journey: Teleporting to " << nextPosition << "\n";
                        getScheduler()->schedule(new TeleportationStartEvent(getScheduler()->now() + 1000, module, nextPosition));
                    }
                } else {
                    // Continue the A* algorithm:
                    closedSet.insert(currentPosition);

                    // Process each neighbor provided by the module's available motions.
                    for (auto &motion : module->getAllMotions()) {
                        Cell3DPosition neighbor = motion.first;

                        // Skip already evaluated nodes.
                        if (closedSet.find(neighbor) != closedSet.end())
                            continue;

                        // Compute tentative gScore (assuming a uniform cost of 1 per move).
                        double tentative_gScore = gScore[currentPosition] + 1;

                        // If this path to neighbor is better, record it.
                        if (gScore.find(neighbor) == gScore.end() || tentative_gScore < gScore[neighbor]) {
                            cameFrom[neighbor] = currentPosition;
                            gScore[neighbor] = tentative_gScore;
                            fScore[neighbor] = tentative_gScore + heuristic(neighbor, targetQueue.front());

                            // Add the neighbor to the open set for further exploration.
                            openSet.push({neighbor, fScore[neighbor]});
                        }
                    }

                    // Schedule the next move if there are nodes to explore.
                    if (!openSet.empty()) {
                        auto nextStep = openSet.top();
                        openSet.pop();
                        console << "A*: Teleporting to " << nextStep.first << "\n";
                        getScheduler()->schedule(new TeleportationStartEvent(getScheduler()->now() + 1000, module, nextStep.first));
                    } else {
                        console << "A*: No more nodes to explore. Path not found.\n";
                    }
                }
            }
            break;

        default:
            break;
    }
}




void CatomsTest3BlockCode::onBlockSelected() {
    cerr << endl << "--- PRINT MODULE " << *module << "---" << endl;
}

void CatomsTest3BlockCode::onAssertTriggered() {
    console << " has triggered an assert\n";
}

bool CatomsTest3BlockCode::parseUserCommandLineArgument(int &argc, char **argv[]) {
    if ((argc > 0) && ((*argv)[0][0] == '-')) {
        switch((*argv)[0][1]) {
            case 'b': {
                cout << "-b option provided" << endl;
                return true;
            } break;

            case '-': {
                std::string varg = std::string((*argv)[0] + 2);
                if (varg == std::string("foo")) {
                    int fooArg;
                    try {
                        fooArg = stoi((*argv)[1]);
                        argc--;
                        (*argv)++;
                    } catch(std::logic_error&) {
                        std::stringstream err;
                        err << "foo must be an integer." << endl;
                        throw CLIParsingError(err.str());
                    }
                    cout << "--foo option provided with value: " << fooArg << endl;
                } else return false;
                return true;
            }

            default: cerr << "Unrecognized command line argument: " << (*argv)[0] << endl;
        }
    }
    return false;
}

std::string CatomsTest3BlockCode::onInterfaceDraw() {
    std::stringstream trace;
    trace << "Distance: " << distance;
    return trace.str();
}

double CatomsTest3BlockCode::heuristic(const Cell3DPosition& current, const Cell3DPosition& goal) {
    // calculation (Manhattan distance)
    return std::abs(current[0] - goal[0]) + std::abs(current[1] - goal[1]) + std::abs(current[2] - goal[2]);
}