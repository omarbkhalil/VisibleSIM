#include "MyApp2plusBlockCode.hpp"
#include <set>
#include <windows.h>

#include "events/scheduler.h"
#include "events/events.h"
#include "utils/trace.h"
#include "robots/blinkyBlocks/blinkyBlocksWorld.h"
#include "robots/blinkyBlocks/blinkyBlocksBlockCode.h"

BlinkyBlocksBlock *moduleA = nullptr;
BlinkyBlocksBlock *moduleB = nullptr;
BlinkyBlocksBlock *moduleC = nullptr;
BlinkyBlocksBlock *moduleCenter = nullptr;
int distAB, distBC, distBcenter, distCcenter;
P2PNetworkInterface *pathInterface = nullptr;
int maxDistance = 0;

static bool firstModuleStarted = false;
static bool Check = false;

using namespace BlinkyBlocks;

MyApp2plusBlockCode::MyApp2plusBlockCode(BlinkyBlocksBlock *host) : BlinkyBlocksBlockCode(host) {
    // @warning Do not remove block below, as a blockcode with a NULL host might be created
    //  for command line parsing
    if (not host) return;

    // Registers a callback (handleSampleMessage) to the message of type SAMPLE_MSG_ID
    addMessageEventFunc2(SAMPLE_MSG_ID,
                         std::bind(&MyApp2plusBlockCode::handleSampleMessage, this,
                                   std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(BACK_MSG_ID,
                         std::bind(&MyApp2plusBlockCode::handleBackMessage, this,
                                   std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(REPORT_MSG_ID,
                         std::bind(&MyApp2plusBlockCode::handleReportMessage, this,
                                   std::placeholders::_1, std::placeholders::_2));

    // Set the module pointer
    module = static_cast<BlinkyBlocksBlock *>(hostBlock);
}


std::vector<Cell3DPosition> MyApp2plusBlockCode::targetPositions;


void MyApp2plusBlockCode::startup() {


    console << "start";

    // Sample distance coloring algorithm below
    if (isA) { // Master ID is 1
        module->setColor(RED);
        distance = 0;
      floodDistance();
    } else {
        distance = -1; // Unknown distance
        hostBlock->setColor(LIGHTGREY);
    }

}



void MyApp2plusBlockCode::onMotionEnd() {





    }



void MyApp2plusBlockCode::handleSampleMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender) {
    MessageOf<int> *msg = static_cast<MessageOf<int> *>(_msg.get());
    int receivedDistance = *msg->getData();
    console << "Module #" << module->blockId << " received distance = " << receivedDistance
            << " from module #" << sender->getConnectedBlockId() << "\n";

    if (distance == -1 || receivedDistance + 1 < distance) {
        distance = receivedDistance + 1;
        parent = sender; // Record the interface to trace back
        console << "distance: " << distance << "\n";
        floodDistance(); // Propagate the new distance to neighbors
    } else {
        // No update required, send back negative acknowledgment
        sendMessage("Back", new MessageOf<int>(BACK_MSG_ID, -1), sender, 100, 200);
    }
}

void MyApp2plusBlockCode::handleBackMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender) {
    MessageOf<int> *msg = static_cast<MessageOf<int> *>(_msg.get());
    int receivedDistance = *msg->getData();
    nbWaitedAnswers--;
    if(receivedDistance > maxDistance) {
        maxDistance = receivedDistance;
        pathInterface = sender;
    }
    if (nbWaitedAnswers == 0) {
        if(distance > maxDistance) {
            maxDistance = distance;
            pathInterface = nullptr;
        }
        if(parent) {
            sendMessage("Back", new MessageOf<int>(BACK_MSG_ID, maxDistance), parent, 100, 200);
            maxDistance = -1;
            parent = nullptr;
        } else {
            console << "Distance: " << maxDistance << "\n";
            if(pathInterface and maxDistance > 0) {
                sendMessage("report", new Message(REPORT_MSG_ID), pathInterface, 100, 200);
                maxDistance = -1;
                parent = nullptr;
                pathInterface = nullptr;
            }
        }
    }
}
void MyApp2plusBlockCode::handleReportMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender) {
    if(pathInterface) {
        sendMessage("report", new Message(REPORT_MSG_ID), pathInterface, 100, 200);
        pathInterface = nullptr;
    } else if(!pathInterface) {
        moduleA=module;
        moduleA->setColor(YELLOW);



    }
    /* else if(moduleB) {

         sendMessage("reportbck", new Message(REPORTbck_MSG_ID), pathInterface, 100, 200);


     }


 */
}
/*
 void MyApp2plusBlockCode::handleReportbckMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender) {
    if(pathInterface) {
        sendMessage("report", new Message(REPORTbck_MSG_ID), pathInterface, 100, 200);
        pathInterface = nullptr;
    } else {
        module->setColor(GREEN);

    }

}
*/

void MyApp2plusBlockCode::floodDistance() {
    nbWaitedAnswers = 0; // Reset before starting
    for (int i = 0; i < module->getNbInterfaces(); i++) {
        P2PNetworkInterface* neighborInterface = module->getInterface(i);
        if (neighborInterface && neighborInterface->connectedInterface && neighborInterface != parent) {
            sendMessage("Flood", new MessageOf<int>(SAMPLE_MSG_ID, distance), neighborInterface, 100, 100);
            nbWaitedAnswers++;
        }
    }
    // If no neighbors to wait for, send back immediately
    if (nbWaitedAnswers == 0 && parent != nullptr) {
        sendMessage("Back", new MessageOf<int>(BACK_MSG_ID, distance), parent, 100, 200);
    }
}










void MyApp2plusBlockCode::processLocalEvent(EventPtr pev) {
    std::shared_ptr<Message> message;
    stringstream info;

    // Do not remove line below
    BlinkyBlocksBlockCode::processLocalEvent(pev);

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

void MyApp2plusBlockCode::onBlockSelected() {
    // Debug stuff:
    cerr << endl << "--- PRINT MODULE " << *module << "---" << endl;
}

void MyApp2plusBlockCode::onAssertTriggered() {
    console << " has triggered an assert" << "\n";

    // Print debugging some info if needed below
    // ...
}

bool MyApp2plusBlockCode::parseUserCommandLineArgument(int &argc, char **argv[]) {
    /* Reading the command line */
    if ((argc > 0) && ((*argv)[0][0] == '-')) {
        switch((*argv)[0][1]) {

            // Single character example: -b
            case 'b':   {
                cout << "-b option provided" << endl;
                return true;
            } break;

            // Composite argument example: --foo 13
            case '-': {
                string varg = string((*argv)[0] + 2); // argv[0] without "--"
                if (varg == string("foo")) { //
                    int fooArg;
                    try {
                        fooArg = stoi((*argv)[1]);
                        argc--;
                        (*argv)++;
                    } catch(std::logic_error&) {
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

string MyApp2plusBlockCode::onInterfaceDraw() {
    string str = "Press 'r' to run the simulation.";
    if (moduleA != nullptr) {
    }
    if (moduleB != nullptr) {
        str += "\nB= (" + moduleB->position.to_string() + "), distAB=" + to_string(distAB);
    }
    if (moduleC != nullptr) {
        str += "\nC= (" + moduleC->position.to_string() + "), distBC=" + to_string(distBC);
        console << str << "\n";
    }
    if (moduleCenter != nullptr) {
        str += "\nCenter= (" + moduleCenter->position.to_string() + "), dist to B=" +
               to_string(distBcenter) + ", dist to C=" + to_string(distCcenter);
        console << str << "\n";
    }
    return str;
}


void MyApp2plusBlockCode::parseUserBlockElements(TiXmlElement *config) {
    const char *attr = config->Attribute("isA");
    if (attr != nullptr) {
        std::cout << getId() << " is A!" << std::endl;

        isA = true;
    }
}
