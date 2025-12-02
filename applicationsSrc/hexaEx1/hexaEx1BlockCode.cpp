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

 void  HexaEx1BlockCode::init() {
     myDistance = 2000000;
     bestDist   = -1;
     bestId = 0;
     parent = nullptr;
     winnerChild = nullptr;
     nbWaitedAns = 0;



 }
void HexaEx1BlockCode::startup() {


   // parent = nullptr; winnerChild = nullptr;
    //myDistance = 0;
   // bestDist = -1; bestId = 0;
     if (isA) {
        bestId     = getId();
        setColor(RED);
        stage = 1;
        nbWaitedAns = sendMessageToAllNeighbors("initial go from leader",
                                  new MessageOf<pair<int,int>>(SAMPLE_MSG_ID, {1,stage}), 1000, 0, 0);

    }
    else {
        setColor(LIGHTGREY);
    }

    HexanodesWorld *wrl = Hexanodes::getWorld();

    if (!target->isInTarget(module->position) && !firstModuleStarted) {
        firstModuleStarted = true;

        if (target) {
            target->highlight(); // Highlight the target
        }

        // vector<HexanodesMotion *> tab = wrl->getAllMotionsForModule(module);
        // console << "#motion=" << tab.size() << "\n";
        //
        // if (canMove(motionDirection::CW)) {
        //     moveTo(motionDirection::CW, 500000);
        // }
    }
}



void HexaEx1BlockCode::onMotionEnd() {
     HexanodesWorld *wrl = Hexanodes::getWorld();
     auto motions = wrl->getAllMotionsForModule(module);

     int counter = 0;

     for (int i = 0; i < module->getNbInterfaces(); i++) {
         P2PNetworkInterface *neighborInterface = module->getInterface(i);
         if (neighborInterface && neighborInterface->connectedInterface) {
             counter++;
             nMotions++;
         }
     }


     // Loop until we either:
     // 1. Find a CW motion that enters the target (Phase 1)
     // 2. Continue moving inside the target (Phase 2)
     for (auto m : motions) {
         if (m->direction != motionDirection::CW) continue;

         Cell3DPosition nextPos = m->getFinalPos(module->position);

         if (!FTarget) {
             // Phase 1: move until first position inside target
             moveTo(CW);
             if (target->isInTarget(nextPos)) {
                 FTarget = true; // first target reached
                 console << "Entered target at position: " << nextPos << "\n";
             }
             return; // wait for next motion end
         }
else if (counter>2) {


    Check = true; // optional flag
    stage++;
    init();
    nbWaitedAns = sendMessageToAllNeighbors("initial go from leader",
                   new MessageOf<pair<int,int>>(SAMPLE_MSG_ID, {stage,0}), 1000, 0, 0);

}
         else  {
             // Phase 2: already in target, move only if next position stays in target
             if (target->isInTarget(nextPos)) {
                 moveTo(CW);
                 return; // wait for next motion end
             } else {
                 // Stop: next position leaves target
                 console << "Module reached target boundary, stopping at position: " << module->position << "\n";
                 Check = true; // optional flag
                 stage++;
                 init();
                 nbWaitedAns = sendMessageToAllNeighbors("initial go from leader",
                                new MessageOf<pair<int,int>>(SAMPLE_MSG_ID, {stage,0}), 1000, 0, 0);
                 return;
             }
         }
     }
 }


void HexaEx1BlockCode::handleSampleMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender) {
    MessageOf<pair<int, int>>* msg = static_cast<MessageOf<pair<int,int>>*>(_msg.get());

    pair<int, int> msgData =*msg->getData();

    int msgDistance = msgData.second;
    int msgStage = msgData.first;

    if (msgStage>stage) {
        stage = msgStage;

        init();

        if (target->isInTarget(module->position)) {
            msgDistance= -1;
        }

        parent = nullptr;
        myDistance = msgDistance;

    }


    if (parent == nullptr || distance > msgDistance) {
        // First time we get GO: adopt this sender as parent in the BFS tree
        parent      = sender;
        myDistance  = msgDistance;

        bestDist    = myDistance;
        bestId      = getId();
        winnerChild = nullptr;

        // Forward GO to all neighbors except parent; count children
        nbWaitedAns = sendMessageToAllNeighbors("go",
                           new MessageOf<pair<int,int>>(SAMPLE_MSG_ID, {stage,myDistance + 1}),
                           1000, 100, 1, parent);

        // If no children, leaf send BACK immediately to parent
        if (nbWaitedAns == 0) {
            BackPayload b{0, (target->isInTarget(module->position)?-1:myDistance), getId()};
            sendMessage("leafBack",
                        new MessageOf<BackPayload>(BACK_MSG_ID, b),
                        parent, 1000, 100);
        }
    } else {
        // Already have a parent
        BackPayload b{0, (target->isInTarget(module->position)?-1:myDistance), getId()};

       // BackPayload b{0, -1, getId()}; // dist=-1 so it never wins aggregation
        sendMessage("reject",
                    new MessageOf<BackPayload>(BACK_MSG_ID, b),
                    sender, 1000, 100);
    }
}

void HexaEx1BlockCode::handleBackMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender) {
    auto* m = static_cast<MessageOf<BackPayload>*>(_msg.get());
    BackPayload bp = *m->getData();

    if (bp.kind == 1) {
        if (getId() == bp.id) {
            // farthest from the B
            setColor(YELLOW);
            distAB   = bp.dist;
            // bp.dist    = 0;
            console << "DIstanceTEST=" <<  bp.dist << "\n";

            HexanodesWorld *wrl = Hexanodes::getWorld();


            vector<HexanodesMotion *> tab = wrl->getAllMotionsForModule(module);
            console << "#motion=" << tab.size() << "\n";

            if (canMove(motionDirection::CW)) {
                moveTo(motionDirection::CW, 500000);
            }
            // // ---- Start PHASE 2 from B ----
            // parent2      = nullptr;
            // winnerChild2 = nullptr;
            // myDistance2  = 0;
            // bestDist2    = 0;
            // bestId2      = getId();

init();
        } else if (winnerChild) {
            // Forward only along the stored branch that produced the winner
            sendMessage("selectDown",
                        new MessageOf<BackPayload>(BACK_MSG_ID,bp),
                        winnerChild, 100, 1000);
        }
        return; // do not touch nbWaitedAns on select-down
    }

    // ACK_UP from a child
    if (bp.dist > bestDist || (bp.dist == bestDist && bp.id < bestId)) {
        bestDist    = bp.dist;
        bestId      = bp.id;
        winnerChild = sender;
    }

    // Count child answers
    nbWaitedAns--;
    if (nbWaitedAns == 0) {
        if (parent) {

            if (myDistance > bestDist || (myDistance == bestDist && getId() < bestId)) {
                bestDist    = myDistance;
                bestId      = getId();
                winnerChild = nullptr;
            }

            BackPayload up{0, bestDist, bestId};
            sendMessage("backUp",
                        new MessageOf<BackPayload>(BACK_MSG_ID, up),
                        parent, 100, 1000);
        } else {
            // Root decides winner
            console << "Phase1: farthest from leader = " << bestId
                    << " at dist = " << bestDist << "\n";

            distAB = bestDist;

            if (winnerChild) {
                // Send select-down
                BackPayload sel{1, bestDist, bestId}; // SELECT_DOWN
                sendMessage("selectDown",
                            new MessageOf<BackPayload>(BACK_MSG_ID, sel),
                            winnerChild, 100, 1000);
            }
        }
    }
}

void HexaEx1BlockCode::handleReportMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender) {
    // if(pathInterface) {
    //     sendMessage("report", new Message(REPORT_MSG_ID), pathInterface, 100, 200);
    //     pathInterface = nullptr;
    // } else {
    //     console << "can start moving \n";
    //     HexanodesWorld *wrl = Hexanodes::getWorld();
    //     auto tab = wrl->getAllMotionsForModule(module);
    //     auto ci = tab.begin();
    //     while (ci != tab.end() && ((*ci)->direction != motionDirection::CW)) {
    //         ci++;
    //     }
    //     if (ci != tab.end() && nMotions <= 500) {
    //         moveTo(CW);
    //     }
    // }
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

void HexaEx1BlockCode::parseUserBlockElements(TiXmlElement *config) {
    const char *attr = config->Attribute("isA");
    if (attr != nullptr) {
        std::cout << getId() << " is isA!" << std::endl;
        isA = true;
    }
}