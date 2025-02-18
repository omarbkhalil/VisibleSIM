#include "catomsTest3BlockCode.hpp"

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <fstream>  // for serialization functions
#include <iostream> // for cout
#include <filesystem>

#include "motion/teleportationEvents.h"
#include "robots/catoms3D/catoms3DMotionEngine.h"
#include "robots/catoms3D/catoms3DRotationEvents.h"

using namespace Catoms3D;

// Forward declarations for serialization helper functions in the global namespace.
void saveOptimalPath(const std::vector<Cell3DPosition>& path, const std::string& filename);
std::vector<Cell3DPosition> loadOptimalPath(const std::string& filename);

map<Cell3DPosition, vector<Cell3DPosition>> CatomsTest3BlockCode::cells;
vector<Cell3DPosition> CatomsTest3BlockCode::visited;
vector<Cell3DPosition> CatomsTest3BlockCode::teleportedPositions;

std::priority_queue<
    std::pair<Cell3DPosition, double>,
    std::vector<std::pair<Cell3DPosition, double>>,
    std::function<bool(std::pair<Cell3DPosition, double>, std::pair<Cell3DPosition, double>)>
> CatomsTest3BlockCode::openSet{
    [](std::pair<Cell3DPosition, double> a, std::pair<Cell3DPosition, double> b) {
        return a.second > b.second;
    }
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

// Constructor.
CatomsTest3BlockCode::CatomsTest3BlockCode(Catoms3DBlock *host) : Catoms3DBlockCode(host) {
    if (!host) return;
    module = static_cast<Catoms3DBlock*>(hostBlock);
}

void CatomsTest3BlockCode::startup() {
    console << "start\n";
    std::cout << "Working directory: " << std::filesystem::current_path() << std::endl;

    // Only the designated module (at the start cell) should use restored mode.
    if(module->position != startQueue.front()){
        // For modules not at the designated starting cell, use normal initialization.
        // (You may decide to do nothing or run a different branch.)
        return;
    }

    // Check if an optimal path file exists.
    std::ifstream optimalFile("optimal_path.bin", std::ios::binary);
    if (optimalFile.good()) {
        console << "Optimal path file found. Loading and teleporting along the saved path...\n";
        auto optimalPath = ::loadOptimalPath("optimal_path.bin");
        if (!optimalPath.empty()) {
            // If the file was saved from start->target, use it as is.
            discoveredPath = optimalPath;
            restoredMode = true;

            // Remove duplicate start cells if they exist.
            if(discoveredPath.size() > 1 && discoveredPath.front() == discoveredPath[1]){
                discoveredPath.erase(discoveredPath.begin()+1);
            }

            // If the module is not at the starting position, schedule a teleport to it.
            if (module->position != discoveredPath.front()) {
                console << "Restored mode: Teleporting module to start position " << discoveredPath.front() << "\n";
                getScheduler()->schedule(new TeleportationStartEvent(getScheduler()->now() + 1000, module, discoveredPath.front()));
            } else {
                // Already at start; remove it from the path.
                discoveredPath.erase(discoveredPath.begin());
                // And schedule next teleport if available and if the next position is not the same.
                if (!discoveredPath.empty() && module->position != discoveredPath.front()) {
                    Cell3DPosition nextPosition = discoveredPath.front();
                    discoveredPath.erase(discoveredPath.begin());
                    getScheduler()->schedule(new TeleportationStartEvent(getScheduler()->now() + 1000, module, nextPosition));
                }
            }
        }
        return; // Skip normal A* initialization if restored mode.
    }

    // Normal A* initialization.
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
    // Call the base implementation.
    Catoms3DBlockCode::processLocalEvent(pev);

    // Restored mode: simply follow the saved path.
    if (pev->eventType == EVENT_TELEPORTATION_END && restoredMode) {
        if (!discoveredPath.empty()) {
            Cell3DPosition nextPosition = discoveredPath.front();
            // Check: if the module is already at the next position, skip it.
            if (module->position == nextPosition) {
                console << "Restored mode: Module already at " << nextPosition << ", skipping teleportation.\n";
                discoveredPath.erase(discoveredPath.begin());
                if (!discoveredPath.empty() && module->position != discoveredPath.front()) {
                    nextPosition = discoveredPath.front();
                    discoveredPath.erase(discoveredPath.begin());
                    getScheduler()->schedule(new TeleportationStartEvent(getScheduler()->now() + 1000, module, nextPosition));
                }
            } else {
                discoveredPath.erase(discoveredPath.begin());
                console << "Restored mode: Teleporting to " << nextPosition << "\n";
                getScheduler()->schedule(new TeleportationStartEvent(getScheduler()->now() + 1000, module, nextPosition));
            }
        } else {
            console << "Restored mode: Teleportation complete. End of path reached.\n";
        }
        return;
    }

    // Normal A* processing.
    Cell3DPosition currentPosition = module->position;
    if (cells.find(currentPosition) == cells.end()) {
        vector<Cell3DPosition> neighbors;
        for (auto &motion : module->getAllMotions()) {
            neighbors.push_back(motion.first);
        }
        cells[currentPosition] = neighbors;
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
                }
            } else {
                if (!targetQueue.empty() && currentPosition == targetQueue.front()) {
                    targetQueue.pop();
                    console << "Goal reached. Reconstructing optimal path...\n";
                    isReturning = true;
                    discoveredPath.clear();
                    while (cameFrom.find(currentPosition) != cameFrom.end()) {
                        discoveredPath.push_back(currentPosition);
                        currentPosition = cameFrom[currentPosition];
                    }
                    discoveredPath.push_back(currentPosition);
                    std::reverse(discoveredPath.begin(), discoveredPath.end());
                    console << "Optimal path from start to goal:\n";
                    for (auto &pos : discoveredPath) {
                        console << pos << "\n";
                    }
                    saveOptimalPath(discoveredPath, "optimal_path.bin");
                    if (!discoveredPath.empty()) {
                        Cell3DPosition nextPosition = discoveredPath.back();
                        discoveredPath.pop_back();
                        console << "Starting return journey: Teleporting to " << nextPosition << "\n";
                        getScheduler()->schedule(new TeleportationStartEvent(getScheduler()->now() + 1000, module, nextPosition));
                    }
                } else {
                    closedSet.insert(currentPosition);
                    for (auto &motion : module->getAllMotions()) {
                        Cell3DPosition neighbor = motion.first;
                        if (closedSet.find(neighbor) != closedSet.end())
                            continue;
                        double tentative_gScore = gScore[currentPosition] + 1;
                        if (gScore.find(neighbor) == gScore.end() || tentative_gScore < gScore[neighbor]) {
                            cameFrom[neighbor] = currentPosition;
                            gScore[neighbor] = tentative_gScore;
                            fScore[neighbor] = tentative_gScore + heuristic(neighbor, targetQueue.front());
                            openSet.push({neighbor, fScore[neighbor]});
                        }
                    }
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
    return std::abs(current[0] - goal[0]) +
           std::abs(current[1] - goal[1]) +
           std::abs(current[2] - goal[2]);
}

void saveOptimalPath(const std::vector<Cell3DPosition>& path, const std::string& filename) {
    std::ofstream ofs(filename, std::ios::binary);
    if (!ofs) {
         cout << "Error: Unable to open file " << filename << " for writing.\n";
        return;
    }
    size_t size = path.size();
    ofs.write(reinterpret_cast<const char*>(&size), sizeof(size));
    for (const auto& pos : path) {
        pos.serialize(ofs);
    }
    ofs.close();
    cout << "Optimal path saved to " << filename << " with " << size << " positions.\n";
}

std::vector<Cell3DPosition> loadOptimalPath(const std::string& filename) {
    std::vector<Cell3DPosition> path;
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs) {
        cout << "Error: Unable to open file " << filename << " for reading.\n";
        return path;
    }
    size_t size = 0;
    ifs.read(reinterpret_cast<char*>(&size), sizeof(size));
    cout << "Loading optimal path from " << filename << " with " << size << " positions.\n";
    for (size_t i = 0; i < size; ++i) {
        Cell3DPosition pos;
        pos.deserialize(ifs);
        path.push_back(pos);
    }
    ifs.close();
    return path;
}
