#include "catomsTest1BlockCode.hpp"

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>

#include "motion/teleportationEvents.h"
#include "robots/catoms3D/catoms3DMotionEngine.h"
#include "robots/catoms3D/catoms3DRotationEvents.h"


using namespace Catoms3D;

map<Cell3DPosition, vector<Cell3DPosition>> CatomsTest1BlockCode::cells;
vector<Cell3DPosition> CatomsTest1BlockCode::visited;
vector<Cell3DPosition> CatomsTest1BlockCode::teleportedPositions;

CatomsTest1BlockCode::CatomsTest1BlockCode(Catoms3DBlock *host) : Catoms3DBlockCode(host) {
    if (!host) return;

    /*addMessageEventFunc2(SAMPLE_MSG_ID,
        std::bind(&CatomsTest1BlockCode::handleSampleMessage, this,
                  std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(BACK_MSG_ID,
        std::bind(&CatomsTest1BlockCode::handleBackMessage, this,
                  std::placeholders::_1, std::placeholders::_2));
*/
    module = static_cast<Catoms3DBlock*>(hostBlock);
}

void CatomsTest1BlockCode::startup() {
    console << "start\n";
    if (module->blockId == 1) {
        module->setColor(RED);
        distance = 0;
        //floodDistance();
        cells[module->position] = vector<Cell3DPosition>();
        Cell3DPosition currentPosition = module->position;
        teleportedPositions.push_back(currentPosition);
        for(auto &pos: module->getAllMotions()) {
           cells[module->position].push_back(pos.first);
            visited.push_back(pos.first);
       }
        getScheduler()->schedule(new TeleportationStartEvent(getScheduler()->now() + 1000, module, *cells[currentPosition].begin()));
    } else {
        distance = -1;
        hostBlock->setColor(LIGHTGREY);
    }
}

/*
 void CatomsTest1BlockCode::handleSampleMessage(std::shared_ptr<Message> _msg,
                                               P2PNetworkInterface* sender) {
    MessageOf<int>* msg = static_cast<MessageOf<int>*>(_msg.get());
    int d = *msg->getData() + 1;

    console << "Block " << module->blockId << " received distance = " << d
            << " from " << sender->getConnectedBlockId() << "\n";

    if (distance == -1 || d < distance) {
        distance = d;
        parent = sender;
        console << "Block " << module->blockId << " adopts distance = " << distance << "\n";
        floodDistance();
    } else {
        // No improvement, send back
        sendMessage("Back", new MessageOf<int>(BACK_MSG_ID, distance), sender, 100, 200);
    }
}

void CatomsTest1BlockCode::handleBackMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender) {
    MessageOf<int> *msg = static_cast<MessageOf<int> *>(_msg.get());
    nbWaitedAnswers--;

    if (nbWaitedAnswers == 0) {
        if (parent) {
            // Not leader, propagate back
            sendMessage("Back", new MessageOf<int>(BACK_MSG_ID, distance), parent, 100, 200);
        } else {
            // Leader: gradient is stable
            console << "Gradient construction complete. All distances are stable.\n";

            // Example: run A* from leader (1) to some goal node (e.g., 5)
            int goalId = 33;
            std::vector<int> path = runAStar(1, goalId);
            if (!path.empty()) {
                console << "A* Path from 1 to " << goalId << ": ";
                for (int id : path) console << id << " ";
                console << "\n";
            } else {
                console << "No path found from 1 to " << goalId << "\n";
            }
        }
    }
}

void CatomsTest1BlockCode::floodDistance() {
    nbWaitedAnswers = 0;
    for (int i = 0; i < module->getNbInterfaces(); i++) {
        P2PNetworkInterface* neighborInterface = module->getInterface(i);
        if (neighborInterface && neighborInterface->connectedInterface && neighborInterface != parent) {
            sendMessage("Flood", new MessageOf<int>(SAMPLE_MSG_ID, distance), neighborInterface, 100, 100);
            nbWaitedAnswers++;
        }
    }
    if (nbWaitedAnswers == 0 && parent != nullptr) {
        sendMessage("Back", new MessageOf<int>(BACK_MSG_ID, distance), parent, 100, 200);
    }
}

*/
void CatomsTest1BlockCode::onMotionEnd() {
    console << " has reached its destination\n";
}

void CatomsTest1BlockCode::processLocalEvent(EventPtr pev) {
    Catoms3DBlockCode::processLocalEvent(pev);

    switch (pev->eventType) {
        case EVENT_TELEPORTATION_END:
            // Return handling
            if (isReturning) {
                if (!discoveredPath.empty()) {
                    // Get the next position
                    Cell3DPosition nextPosition = discoveredPath.back();
                    discoveredPath.pop_back(); // Remove it from the path

                    console << "Returning: Teleporting to " << nextPosition << "\n";

                    getScheduler()->schedule(
                        new TeleportationStartEvent(getScheduler()->now() + 1000, module, nextPosition)
                    );
                } else {
                    // Return complete
                    console << "Return journey complete. Back at the initial position.\n";
                    isReturning = false; // Reset the flag
                }
            }
            // Discovery handling
            else if (!visited.empty() && module->position != Cell3DPosition(15, 0, 1)) {
                // Get the next position to explore
                Cell3DPosition nextPosition = visited.back();
                visited.pop_back();
                teleportedPositions.push_back(nextPosition);

                if (parentMap.find(nextPosition) == parentMap.end()) {
                    parentMap[nextPosition] = module->position;
                }

                // Discover adjacent positions
                for (auto &pos : module->getAllMotions()) {
                    cells[module->position].push_back(pos.first);
                    if (std::find(visited.begin(), visited.end(), pos.first) == visited.end() &&
                        std::find(teleportedPositions.begin(), teleportedPositions.end(), pos.first) == teleportedPositions.end()) {
                        visited.push_back(pos.first);
                        parentMap[pos.first] = module->position; // Track parent
                    }
                }

                getScheduler()->schedule(
                    new TeleportationStartEvent(getScheduler()->now() + 1000, module, nextPosition)
                );
            }
            // Goal reached handling
            else if (module->position == Cell3DPosition(15, 0, 1)) {
                console << "Goal reached. Reconstructing optimal path...\n";
                isReturning = true; // Set return flag

                // Backtrack from goal to start
                discoveredPath.clear();
                Cell3DPosition current = module->position;

                //Connect the path
                while (parentMap.find(current) != parentMap.end()) {
                    discoveredPath.push_back(current);
                    current = parentMap[current];
                }
                discoveredPath.push_back(current); // Add the start position

                std::reverse(discoveredPath.begin(), discoveredPath.end()); // Reverse to start-to-goal order

                console << "Optimal path from start to goal:\n";
                for (auto &pos : discoveredPath) {
                    console << pos << "\n";
                }

                // Start returning
                if (!discoveredPath.empty()) {
                    Cell3DPosition nextPosition = discoveredPath.back();
                    discoveredPath.pop_back();

                    console << "Starting return journey: Teleporting to " << nextPosition << "\n";
                    getScheduler()->schedule(
                        new TeleportationStartEvent(getScheduler()->now() + 1000, module, nextPosition)
                    );
                }
            }
            break;

        default:
            break;
    }
}



void CatomsTest1BlockCode::onBlockSelected() {
    cerr << endl << "--- PRINT MODULE " << *module << "---" << endl;
}

void CatomsTest1BlockCode::onAssertTriggered() {
    console << " has triggered an assert\n";
}

bool CatomsTest1BlockCode::parseUserCommandLineArgument(int &argc, char **argv[]) {
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

std::string CatomsTest1BlockCode::onInterfaceDraw() {
    std::stringstream trace;
    trace << "Distance: " << distance;
    return trace.str();
}

/*
 std::vector<int> CatomsTest1BlockCode::runAStar(int startId, int goalId) {
    auto *world = (Catoms3DWorld*)BaseSimulator::getWorld();
    if (!world->getBlockById(goalId)) {
        console << "Goal block not found\n";
        return {};
    }

    std::unordered_map<int, NodeRecord> records;
    auto cmp = [&records](int lhs, int rhs) {
        return records[lhs].fScore > records[rhs].fScore;
    };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> openSet(cmp);

    records[startId] = {0.0, heuristic(startId, goalId), -1};
    openSet.push(startId);

    std::unordered_set<int> closedSet;

    while (!openSet.empty()) {
        int current = openSet.top();
        openSet.pop();

        if (current == goalId) {
            return reconstructPath(records, current);
        }

        closedSet.insert(current);

        Catoms3DBlock *currentModule = world->getBlockById(current);
        if (!currentModule) continue;

        for (int i = 0; i < currentModule->getNbInterfaces(); i++) {
            P2PNetworkInterface* neighborInterface = currentModule->getInterface(i);
            if (!neighborInterface || !neighborInterface->connectedInterface) continue;
            int neighborId = neighborInterface->connectedInterface->hostBlock->blockId;

            if (closedSet.find(neighborId) != closedSet.end()) continue;

            double tentative_gScore = records[current].gScore + 1.0;

            if (records.find(neighborId) == records.end() || tentative_gScore < records[neighborId].gScore) {
                records[neighborId].cameFrom = current;
                records[neighborId].gScore = tentative_gScore;
                records[neighborId].fScore = tentative_gScore + heuristic(neighborId, goalId);
                openSet.push(neighborId);
            }
        }
    }

    return {};
}

std::vector<int> CatomsTest1BlockCode::reconstructPath(std::unordered_map<int, NodeRecord> &records, int current) {
    std::vector<int> path;
    while (current != -1) {
        path.push_back(current);
        current = records[current].cameFrom;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

double CatomsTest1BlockCode::heuristic(int currentId, int goalId) {
    auto *world = (Catoms3DWorld*)BaseSimulator::getWorld();
    Catoms3DBlock *cBlock = world->getBlockById(currentId);
    Catoms3DBlock *gBlock = world->getBlockById(goalId);

    if (!cBlock || !gBlock) return 0.0;

    CatomsTest1BlockCode *cCode = dynamic_cast<CatomsTest1BlockCode*>(cBlock->blockCode);
    CatomsTest1BlockCode *gCode = dynamic_cast<CatomsTest1BlockCode*>(gBlock->blockCode);

    if (!cCode || !gCode) return 0.0;

    int dC = cCode->distance;
    int dG = gCode->distance;
    if (dC == -1 || dG == -1) return 0.0;

    // Simple heuristic: absolute difference in their distances from the leader
    return std::fabs((double)dC - (double)dG);
}
*/