#include "myApp2BlockCode.hpp"

BlinkyBlocksBlock *moduleA=nullptr;
BlinkyBlocksBlock *moduleB=nullptr;
BlinkyBlocksBlock *moduleC=nullptr;
BlinkyBlocksBlock *moduleCenter=nullptr;
int distAB,distBC,distBcenter,distCcenter;

using namespace BlinkyBlocks;

MyApp2BlockCode::MyApp2BlockCode(BlinkyBlocksBlock *host) : BlinkyBlocksBlockCode(host) {
    // @warning Do not remove block below, as a blockcode with a NULL host might be created
    //  for command line parsing
    if (not host) return;



    // Registers a callback (handleSampleMessage) to the message of type SAMPLE_MSG_ID
    addMessageEventFunc2(SAMPLE_MSG_ID,
                         std::bind(&MyApp2BlockCode::handleSampleMessage, this,
                                   std::placeholders::_1, std::placeholders::_2));

    // Set the module pointer
    module = static_cast<BlinkyBlocksBlock*>(hostBlock);
  }

void MyApp2BlockCode::startup() {
    console << "start";

    // Sample distance coloring algorithm below
    if (isA) { // Master ID is 1
        module->setColor(RED);
        distance = 0;
        sendMessageToAllNeighbors("Sample Broadcast",
                                  new MessageOf<int>(SAMPLE_MSG_ID,distance),100,200,0);
    } else {
        distance = -1; // Unknown distance
        hostBlock->setColor(LIGHTGREY);
    }

    // Additional initialization and algorithm start below
    // ...
}

void MyApp2BlockCode::handleSampleMessage(std::shared_ptr<Message> _msg,
                                               P2PNetworkInterface* sender) {
    MessageOf<int>* msg = static_cast<MessageOf<int>*>(_msg.get());

    int dis=0;
    int d = *msg->getData() + 1;
    console << " received d =" << d << " from " << sender->getConnectedBlockId() << "\n";

    if (distance == -1 || distance > d) {
        console << " updated distance = " << d << "\n";
        distance = d;
        floodDistance();

        if (distance > distAB) {

            if (moduleB != nullptr) {
                moduleB->setColor(LIGHTGREY);
            }
            distAB = distance;
            moduleB = module;
            moduleB->setColor(YELLOW);

            distest=distance;
        }
        else if (distance > distBC) {

            if (moduleC != nullptr) {
                moduleC->setColor(LIGHTGREY);
            }
            distBC = distance;
            moduleC = module;
            moduleC->setColor(BLUE);





        }
        else if (distance == (distance-(distest / 2))) {
            if (moduleCenter == nullptr) {
            moduleCenter = module;
            moduleCenter->setColor(BROWN);
        }

        }


    }

    }





void MyApp2BlockCode::onMotionEnd() {
    console << " has reached its destination" << "\n";

    // ...
}

void MyApp2BlockCode::processLocalEvent(EventPtr pev) {
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

void MyApp2BlockCode::onBlockSelected() {
    // Debug stuff:
    cerr << endl << "--- PRINT MODULE " << *module << "---" << endl;
}

void MyApp2BlockCode::onAssertTriggered() {
    console << " has triggered an assert" << "\n";

    // Print debugging some info if needed below
    // ...
}

bool MyApp2BlockCode::parseUserCommandLineArgument(int &argc, char **argv[]) {
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

string MyApp2BlockCode::onInterfaceDraw() {
    string str = "Press 'r' to run the simulation.";
    if (moduleA != nullptr) {
        str = "A= (" + moduleA->position.to_string() + ")";
        console << str << "\n";
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
void MyApp2BlockCode::floodDistance() {
    for (int i = 0; i < module->getNbInterfaces(); i++) {
        P2PNetworkInterface* neighborInterface = module->getInterface(i);
        if (neighborInterface && neighborInterface->connectedInterface) {
            sendMessage("Flood", new MessageOf<int>(SAMPLE_MSG_ID, distance + 1), neighborInterface, 100, 100);
        }
    }

}
void MyApp2BlockCode::parseUserBlockElements(TiXmlElement *config) {
    const char *attr = config->Attribute("isA");
    if (attr != nullptr) {
        std::cout << getId() << " is A!" << std::endl;

        isA = true;
    }
}
