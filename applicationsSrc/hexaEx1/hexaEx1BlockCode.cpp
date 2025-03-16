#include "hexaEx1BlockCode.hpp"
#include <set>
#include <windows.h>

#include "robots/hexanodes/hexanodesWorld.h"
#include "events/scheduler.h"
#include "events/events.h"
#include "utils/trace.h"
#include "robots/hexanodes/hexanodesMotionEvents.h"
#include "robots/hexanodes/hexanodesMotionEngine.h"

HexanodesBlock *moduleA = nullptr;
HexanodesBlock *moduleB = nullptr;
HexanodesBlock *moduleC = nullptr;
HexanodesBlock *moduleCenter = nullptr;
int distAB, distBC, distBcenter, distCcenter;
P2PNetworkInterface *pathInterface = nullptr;
int maxDistance = 0;

static bool firstModuleStarted = false;
static bool Check = false;

using namespace Hexanodes;

HexaEx1BlockCode::HexaEx1BlockCode(HexanodesBlock *host) : HexanodesBlockCode(host) {
    // @warning Do not remove block below, as a blockcode with a NULL host might be created
    //  for command line parsing
    if (not host) return;

    // Registers a callback (handleSampleMessage) to the message of type SAMPLE_MSG_ID
    addMessageEventFunc2(SAMPLE_MSG_ID,
                         std::bind(&HexaEx1BlockCode::handleSampleMessage, this,
                                   std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(BACK_MSG_ID,
                         std::bind(&HexaEx1BlockCode::handleBackMessage, this,
                                   std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(REPORT_MSG_ID,
                         std::bind(&HexaEx1BlockCode::handleReportMessage, this,
                                   std::placeholders::_1, std::placeholders::_2));

    // Set the module pointer
    module = static_cast<HexanodesBlock *>(hostBlock);
}


std::vector<Cell3DPosition> HexaEx1BlockCode::targetPositions;


void HexaEx1BlockCode::startup() {
    HexanodesWorld *wrl = Hexanodes::getWorld();

    if (!target->isInTarget(module->position) && !firstModuleStarted) {
        firstModuleStarted = true;

        if (target) {
            target->highlight(); // Highlight the target
        }

        vector<HexanodesMotion *> tab = wrl->getAllMotionsForModule(module);
        console << "#motion=" << tab.size() << "\n";

        if (canMove(motionDirection::CW)) {
            moveTo(motionDirection::CW, 500000);
        }
    }
}



void HexaEx1BlockCode::onMotionEnd() {
    HexanodesWorld *wrl = Hexanodes::getWorld();

    if (!target->isInTarget(module->position)) {
        auto tab = wrl->getAllMotionsForModule(module);
        auto ci = tab.begin();
        while (ci != tab.end() && ((*ci)->direction != motionDirection::CW)) {
            ci++;
        }
        if (ci != tab.end() && nMotions <= 500) {
            moveTo(CW);

        }
    } else {
        Check = true;
        //Sleep(2000);
        floodDistance();
        // auto tab = wrl->getAllMotionsForModule(moduleB);
        // auto ci = tab.begin();
        // while (ci != tab.end() && ((*ci)->direction != motionDirection::CW)) {
        //     ci++;
        // }
        // if (ci != tab.end() && nMotions <= 500) {
        //     moveTo(CW);
        // }
    }
}


void HexaEx1BlockCode::handleSampleMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender) {
    MessageOf<int> *msg = static_cast<MessageOf<int> *>(_msg.get());
    int receivedDistance = *msg->getData() + 1; // Increment the distance
    console << "Module #" << module->blockId << " received distance = " << receivedDistance
            << " from module #" << sender->getConnectedBlockId() << "\n";

    if (parent == nullptr || distance > receivedDistance) {
        distance = receivedDistance;
        console << "distance: " << distance << "\n";
        parent = sender; // Record the interface to trace back
        for (auto *it: module->getP2PNetworkInterfaces()) {
            if (it->isConnected() && it != sender) {
                nbWaitedAnswers++;
                sendMessage("Flood", new MessageOf<int>(SAMPLE_MSG_ID, distance), it, 100, 200);
            }
        }
        if (nbWaitedAnswers == 0) {
            int returnDistance;
            if (canMove(CW) && !target->isInTarget(module->position)) returnDistance = distance;
            else returnDistance = -1;
            sendMessage("Back", new MessageOf<int>(BACK_MSG_ID, returnDistance), sender, 100, 200);
            distance = 0;
            parent = nullptr;
            maxDistance = -1;
        }
    } else {
        sendMessage("Back", new MessageOf<int>(BACK_MSG_ID, -1), sender, 100, 200);
    }
}

void HexaEx1BlockCode::handleBackMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender) {
    if (!_msg || !sender) {
        console << "Error: Invalid message or sender in handleBackMessage.\n";
        return;
    }

    MessageOf<int> *msg = static_cast<MessageOf<int> *>(_msg.get());
    int receivedDistance = *msg->getData();
    nbWaitedAnswers--;

    console << "Module #" << module->blockId << " received Back message from module #"
            << sender->getConnectedBlockId() << " with distance = " << receivedDistance << "\n";

    if (receivedDistance > maxDistance) {
        maxDistance = receivedDistance;
        pathInterface = sender;
    }

    if (nbWaitedAnswers == 0) {
        console << "Module #" << module->blockId << " has received all responses. Max distance = "
                << maxDistance << "\n";

        if (parent) {
            sendMessage("Back", new MessageOf<int>(BACK_MSG_ID, maxDistance), parent, 100, 200);
        } else if (pathInterface && maxDistance > 0) {
            sendMessage("report", new Message(REPORT_MSG_ID), pathInterface, 100, 200);
        } else {
            console << "Module #" << module->blockId << " is the farthest.\n";
        }
    }
}

void HexaEx1BlockCode::handleReportMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender) {
    if(pathInterface) {
        sendMessage("report", new Message(REPORT_MSG_ID), pathInterface, 100, 200);
        pathInterface = nullptr;
    } else {
        console << "can start moving \n";
        HexanodesWorld *wrl = Hexanodes::getWorld();
        auto tab = wrl->getAllMotionsForModule(module);
        auto ci = tab.begin();
        while (ci != tab.end() && ((*ci)->direction != motionDirection::CW)) {
            ci++;
        }
        if (ci != tab.end() && nMotions <= 500) {
            moveTo(CW);
        }
    }
}

/*
void HexaEx1BlockCode::convergecastReportBack() {
    if (pathInterface) {
        sendMessage("Convergecast", new MessageOf<int>(SAMPLE_MSG_ID, distAB), pathInterface, 100, 100);
        console << "Module #" << module->blockId << " sent convergecast with distance = " << distAB << "\n";
    }
}
*/

void HexaEx1BlockCode::startFarthestModuleMovement() {
    if (moduleB == nullptr) {
        console << "No farthest module found.\n";
        return;
    }

    console << "Farthest module identified. Initiating movement for module #" << moduleB->blockId
            << " with distance = " << distAB << "\n";

    HexanodesWorld *wrl = Hexanodes::getWorld();
    auto tab = wrl->getAllMotionsForModule(moduleB);
    auto ci = tab.begin();

    while (ci != tab.end() && ((*ci)->direction != motionDirection::CW)) {
        ci++;
    }
    if (ci != tab.end()) {
        Cell3DPosition targetPosition = (*ci)->getFinalPos(moduleB->position); // Get the target position
        moduleB->moveTo(targetPosition); // Move the farthest module to the calculated position
        console << "Module #" << moduleB->blockId << " started moving to target position: "
                << targetPosition << "\n";
    } else {
        console << "No possible motions for the farthest module #" << moduleB->blockId << "\n";
    }
}


void HexaEx1BlockCode::floodDistance() {
    console << "Initiating flood from module #" << module->blockId << "\n";

    // Send the initial flood message to all neighbors
    for (int i = 0; i < module->getNbInterfaces(); i++) {
        P2PNetworkInterface *neighborInterface = module->getInterface(i);
        if (neighborInterface && neighborInterface->connectedInterface) {
            nbWaitedAnswers++;
            sendMessage("Flood", new MessageOf<int>(SAMPLE_MSG_ID, 0), neighborInterface, 100, 100);
        }
    }
}


void HexaEx1BlockCode::processLocalEvent(EventPtr pev) {
    std::shared_ptr<Message> message;
    stringstream info;

    // Do not remove line below
    HexanodesBlockCode::processLocalEvent(pev);

    switch (pev->eventType) {
        case EVENT_ADD_NEIGHBOR: {
            // Do something when a neighbor is added to an interface of the module
            break;
        }

        case EVENT_REMOVE_NEIGHBOR: {
            // Do something when a neighbor is removed from an interface of the module
            break;
        }
    }
}

/// ADVANCED BLOCKCODE FUNCTIONS BELOW

void HexaEx1BlockCode::onBlockSelected() {
    // Debug stuff:
    cerr << endl << "--- PRINT MODULE " << *module << "---" << endl;
}

void HexaEx1BlockCode::onAssertTriggered() {
    console << " has triggered an assert" << "\n";

    // Print debugging some info if needed below
    // ...
}

bool HexaEx1BlockCode::parseUserCommandLineArgument(int &argc, char **argv[]) {
    /* Reading the command line */
    if ((argc > 0) && ((*argv)[0][0] == '-')) {
        switch ((*argv)[0][1]) {
            // Single character example: -b
            case 'b': {
                cout << "-b option provided" << endl;
                return true;
            }
            break;

            // Composite argument example: --foo 13
            case '-': {
                string varg = string((*argv)[0] + 2); // argv[0] without "--"
                if (varg == string("foo")) {
                    //
                    int fooArg;
                    try {
                        fooArg = stoi((*argv)[1]);
                        argc--;
                        (*argv)++;
                    } catch (std::logic_error &) {
                        stringstream err;
                        err << "foo must be an integer. Found foo = " << argv[1] << endl;
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

string HexaEx1BlockCode::onInterfaceDraw() {
    return "Number of motions: " + to_string(nMotions);
}
