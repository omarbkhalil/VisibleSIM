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
std::map<Cell3DPosition, std::vector<Cell3DPosition>> CatomsTest3BlockCode::cells;
std::vector<Cell3DPosition> CatomsTest3BlockCode::visited;
std::vector<Cell3DPosition> CatomsTest3BlockCode::teleportedPositions;

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
std::map<Cell3DPosition, Cell3DPosition> CatomsTest3BlockCode::cameFrom;
std::set<Cell3DPosition> CatomsTest3BlockCode::closedSet;

std::vector<std::vector<Cell3DPosition>> CatomsTest3BlockCode::globalOptimalPaths;

// Queues for start and target positions.
std::queue<Cell3DPosition> CatomsTest3BlockCode::startQueue(
    std::deque{
        Cell3DPosition(20, 6, 7),
        Cell3DPosition(20, 6, 5),

        Cell3DPosition(20, 7, 6),
      //  Cell3DPosition(19, 6, 7),
        Cell3DPosition(19, 6, 5),

      //  Cell3DPosition(19, 6, 8),
        Cell3DPosition(19, 6, 4),
        Cell3DPosition(19, 6, 7),

        Cell3DPosition(19, 6, 8),
        Cell3DPosition(18, 6, 8),

        Cell3DPosition(18, 6, 4),
        Cell3DPosition(17, 5, 7),

        Cell3DPosition(17, 5, 5),
        Cell3DPosition(17, 5, 6),

        Cell3DPosition(16, 5, 7),
        Cell3DPosition(16, 5, 5),

        Cell3DPosition(16, 5, 6),
        Cell3DPosition(15, 5, 7),

        Cell3DPosition(15, 5, 5),
        Cell3DPosition(15, 6, 8),
        Cell3DPosition(15, 6, 4),

        Cell3DPosition(14, 6, 8),

        Cell3DPosition(14, 6, 4),

        Cell3DPosition(13, 6, 7),
        Cell3DPosition(13, 6, 5),

        Cell3DPosition(13, 7, 6),

     //   Cell3DPosition(12, 6, 7)



    }
);

//state 1
std::queue<Cell3DPosition> CatomsTest3BlockCode::targetQueue(
    std::deque{
        Cell3DPosition(8, 5, 7),
        Cell3DPosition(8, 5, 5),

        Cell3DPosition(8, 5, 6),
      //  Cell3DPosition(7, 5, 6),
        Cell3DPosition(7, 5, 7),

       // Cell3DPosition(6, 5, 7),
        Cell3DPosition(7, 5, 5),
      //  Cell3DPosition(6, 5, 5),
        Cell3DPosition(7, 6, 4),

        Cell3DPosition(7, 6, 8),
        Cell3DPosition(6, 6, 8),

        Cell3DPosition(6, 6, 4),

        Cell3DPosition(5, 6, 7),

        Cell3DPosition(5, 6, 5),
        Cell3DPosition(5, 7, 6),

        Cell3DPosition(4, 7, 6),
        Cell3DPosition(4, 6, 5),

        Cell3DPosition(4, 6, 7),
        Cell3DPosition(3, 6, 7),

        Cell3DPosition(3, 6, 5),
        Cell3DPosition(3, 6, 4),

        Cell3DPosition(3, 6, 8),
        Cell3DPosition(2, 6, 8),

        Cell3DPosition(2, 6, 4),

        Cell3DPosition(1, 5, 5),
        Cell3DPosition(1, 5, 6),

        Cell3DPosition(1, 5, 7),
        // Cell3DPosition(1, 6, 7)



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

// Flag to enable saving. If the file exists, this flag will be disabled.
bool CatomsTest3BlockCode::savingEnabled = true;

CatomsTest3BlockCode::CatomsTest3BlockCode(Catoms3DBlock *host) : Catoms3DBlockCode(host) {
    if (!host) return;
    module = static_cast<Catoms3DBlock*>(hostBlock);
}


std::vector<std::vector<Cell3DPosition>> CatomsTest3BlockCode::loadAllOptimalPaths() {
    std::vector<std::vector<Cell3DPosition>> allPaths;
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

void CatomsTest3BlockCode::startup() {
    console << "start\n";
    std::cout << "Working directory: " << std::filesystem::current_path() << std::endl;

    // Check that the module pointer is valid.
    if (!module) {
        console << "Error: module pointer is null in startup().\n";
        return;
    }

    // Load all saved optimal paths and print them.
    std::vector<std::vector<Cell3DPosition>> loadedPaths = loadAllOptimalPaths();
    if (loadedPaths.empty()) {
        console << "No optimal paths loaded.\n";
    } else {
        console << "Loaded optimal paths:\n";
        for (size_t i = 0; i < loadedPaths.size(); ++i) {
            console << "Path " << (i + 1) << ":\n";
            for (const auto &pos : loadedPaths[i]) {
                console << pos << "\n";
            }
            console << "\n";
        }
        // If there is a saved file, search for a matching path.
        for (const auto &path : loadedPaths) {
            // Check that the path is not empty and its first position matches the module's current position.
            if (!path.empty() && path.front() == module->position) {
                matchingPath = path;  // assuming matchingPath is declared as a member variable
                console << "Matching path found for module's current position.\n";
                break;
            }
        }
    }

    // Normal A* initialization when no saved optimal paths exist.
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

        for (auto &pos : module->getAllMotions()) {
            cells[currentPosition].push_back(pos.first);
            visited.push_back(pos.first);
        }
        openSet.push({currentPosition, fScore[currentPosition]});
        if (!openSet.empty()) {
            auto nextStep = openSet.top();
            openSet.pop();
          getScheduler()->schedule(new TeleportationStartEvent(getScheduler()->now() + 1000, module, nextStep.first));
            //module->moveTo(nextStep.first);
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
        for (auto &motion : module->getAllMotions()) {
            neighbors.push_back(motion.first);
        }
        cells[currentPosition] = neighbors;
    }

    switch (pev->eventType) {
        case EVENT_TELEPORTATION_END:
            if (isReturning) {
                console << "Return journey skipped. Optimal path already computed.\n";
                isReturning = false;
                discoveredPath.clear();
            } else {
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
                    for (auto &pos : discoveredPath) {
                        console << pos << "\n";
                    }


//save

                    auto savedPaths = loadAllOptimalPaths();

                    bool alreadySaved = false;
                    for (const auto &savedPath : savedPaths) {
                        if (savedPath == discoveredPath) { // Assumes operator== is defined for Cell3DPosition
                            alreadySaved = true;
                            break;
                        }
                    }

                    if (!alreadySaved) {
                        saveOptimalPath(discoveredPath);
                    }



                    // Initiate next module's pathfinding so that subsequent modules move.
                    initiateNextModulePathfinding();
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
                            // Ensure targetQueue is not empty before using front()
                            if (targetQueue.empty()) {
                                console << "Error: targetQueue is empty during A* processing.\n";
                                return;
                            }
                            fScore[neighbor] = tentative_gScore + heuristic(neighbor, targetQueue.front());
                            openSet.push({neighbor, fScore[neighbor]});
                        }
                    }

                    //Move on Loaded
                    if (!matchingPath.empty() && matchingPath.size() > 1) {

                        Cell3DPosition nextStep = matchingPath[1];
                        console << "Matching Path: Teleporting to " << nextStep << "\n";
                        getScheduler()->schedule(new TeleportationStartEvent(getScheduler()->now() + 1000, module, nextStep));
                      //  module->moveTo(nextStep);

                        matchingPath.erase(matchingPath.begin());
                    } else if (!openSet.empty()) {
                        auto nextStep = openSet.top();
                        openSet.pop();
                        console << "A*: Teleporting to " << nextStep.first << "\n";
                       getScheduler()->schedule(new TeleportationStartEvent(getScheduler()->now() + 1000, module, nextStep.first));
                       // module->moveTo(nextStep.first);

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
    std::cerr << std::endl << "--- PRINT MODULE " << *module << " ---" << std::endl;
}

void CatomsTest3BlockCode::onAssertTriggered() {
    console << " has triggered an assert\n";
}

bool CatomsTest3BlockCode::parseUserCommandLineArgument(int &argc, char **argv[]) {
    if ((argc > 0) && ((*argv)[0][0] == '-')) {
        switch((*argv)[0][1]) {
            case 'b': {
                std::cout << "-b option provided" << std::endl;
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

double CatomsTest3BlockCode::heuristic(const Cell3DPosition& current, const Cell3DPosition& goal) {
    return std::abs(current[0] - goal[0]) +
           std::abs(current[1] - goal[1]) +
           std::abs(current[2] - goal[2]);
}

void CatomsTest3BlockCode::saveOptimalPath(const std::vector<Cell3DPosition>& path) {
    // If saving is disabled, do nothing.
    if (!savingEnabled) {
        console << "Optimal path saving disabled. Not saving new path.\n";
        return;
    }

    std::ofstream outFile("optimal_paths.txt", std::ios::app);
    if (outFile.is_open()) {
        outFile << "Optimal path from start to goal:\n";
        for (const auto &pos : path) {
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
    Catoms3DBlockCode* nextModule = static_cast<Catoms3DBlockCode*>(nextBlock->blockCode);


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

        for (auto &motion : nextModule->hostBlock->getAllMotions()) {
            cells[currentPosition].push_back(motion.first);
            visited.push_back(motion.first);
        }

        openSet.push({currentPosition, fScore[currentPosition]});

        if (!openSet.empty()) {
            auto nextStep = openSet.top();
            openSet.pop();
            getScheduler()->schedule(new TeleportationStartEvent(
                getScheduler()->now() + 1000, nextModule->hostBlock, nextStep.first));

           // nextModule->hostBlock->moveTo(nextStep.first);

        }
    }
}