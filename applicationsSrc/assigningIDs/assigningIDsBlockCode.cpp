#include "assigningIDsBlockCode.hpp"

using namespace BlinkyBlocks;

#include <fstream>
static std::ofstream id_output("IDs_output.txt");


AssigningIDsBlockCode::AssigningIDsBlockCode(BlinkyBlocksBlock *host) : BlinkyBlocksBlockCode(host) {
    // @warning Do not remove block below, as a blockcode with a NULL host might be created
    //  for command line parsing
    if (not host) return;

    // Registers a callback (handleSampleMessage) to the message of type SAMPLE_MSG_ID
    addMessageEventFunc2(TYPE_1_EXPLORE, std::bind(&AssigningIDsBlockCode::exploreNeighbors, this, std::placeholders::_1, std::placeholders::_2));
     addMessageEventFunc2(TYPE_2_CONFIRM_CHILD, std::bind(&AssigningIDsBlockCode::confirmChild, this, std::placeholders::_1, std::placeholders::_2));
     addMessageEventFunc2(TYPE_3_REJECT_CHILD, std::bind(&AssigningIDsBlockCode::rejectChild, this, std::placeholders::_1, std::placeholders::_2));
     addMessageEventFunc2(TYPE_4_REPORT_SIZE, std::bind(&AssigningIDsBlockCode::reportSize, this, std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(TYPE_5_ASSIGN_ID, std::bind(&AssigningIDsBlockCode::assignId, this, std::placeholders::_1, std::placeholders::_2));

    // Set the module pointer
    module = static_cast<BlinkyBlocksBlock*>(hostBlock);
  }

void AssigningIDsBlockCode::startup() {
    console << "start";

    for (int i = 0; i < module->getNbInterfaces(); i++) {
        P2PNetworkInterface* ni = module->getInterface(i);
        if (ni->isConnected()) {
            neighbours.push_back(ni);
        }
    }

    console << "Module " << getId() << " initial neighbours: ";
    if (neighbours.empty()) {
        console << "NONE\n";
    } else {
        for (auto* n : neighbours) {
            console << n->getConnectedBlockId() << " ";
        }
        console << "\n";
    }




    // Sample distance coloring algorithm below
    if (isLeader== true && isDiscovered == false) { // Master ID is 1
        isDiscovered = true;

        sendMessageToAllNeighbors("Explore", new Message(TYPE_1_EXPLORE), 1000, 0, 0);



        module->setColor(RED);
        distance = 0;
         } else {
        distance = -1; // Unknown distance
        hostBlock->setColor(LIGHTGREY);
    }

    // Additional initialization and algorithm start below
    // ...
}

void AssigningIDsBlockCode::handleSampleMessage(std::shared_ptr<Message> _msg,
                                               P2PNetworkInterface* sender) {
    MessageOf<int>* msg = static_cast<MessageOf<int>*>(_msg.get());

    int d = *msg->getData() + 1;
    console << " received d =" << d << " from " << sender->getConnectedBlockId() << "\n";

    if (distance == -1 || distance > d) {
        console << " updated distance = " << d << "\n";
        distance = d;
        module->setColor(Colors[distance % NB_COLORS]);

        // Broadcast to all neighbors but ignore sender
        // sendMessageToAllNeighbors("Sample Broadcast",
        //                           new MessageOf<int>(SAMPLE_MSG_ID,distance),100,200,1,sender);
    }
}

void AssigningIDsBlockCode::onMotionEnd() {
    console << " has reached its destination" << "\n";

    // do stuff
    // ...
}

void AssigningIDsBlockCode::processLocalEvent(EventPtr pev) {
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

void AssigningIDsBlockCode::onBlockSelected() {
    // Debug stuff:
    cerr << endl << "--- PRINT MODULE " << *module << "---" << endl;
}

void AssigningIDsBlockCode::onAssertTriggered() {
    console << " has triggered an assert" << "\n";

    // Print debugging some info if needed below
    // ...
}

bool AssigningIDsBlockCode::parseUserCommandLineArgument(int &argc, char **argv[]) {
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

string AssigningIDsBlockCode::onInterfaceDraw() {
    stringstream trace;
    trace << "Some value " << 123;
    return trace.str();
}

void AssigningIDsBlockCode::parseUserBlockElements(TiXmlElement *config) {
    const char *attr = config->Attribute("leader");
    isLeader = (attr?Simulator::extractBoolFromString(attr):false);
    if (isLeader) {
        std::cout << getId() << " is leader!" << std::endl; // complete with your code
    }
}


void AssigningIDsBlockCode::exploreNeighbors(std::shared_ptr<Message> msg,
                                             P2PNetworkInterface *sender) {
    console << "Module " << getId()
            << " rcv explore message: " << sender->getConnectedBlockId() << "\n";
    module->setColor(GREY);

    if (isDiscovered) {
        // Already discovered → type 3
        sendMessage("RejectChild", new Message(TYPE_3_REJECT_CHILD), sender, 1000, 0);
    } else {
        // First time discovery
        isDiscovered = true;
        parent = sender;

        // neighbours ← neighbours − {sender}
        neighbours.erase(
            std::remove(neighbours.begin(), neighbours.end(), sender),
            neighbours.end()
        );

        sendMessage("ConfirmChild", new Message(TYPE_2_CONFIRM_CHILD), sender, 1000, 0);

        // PSEUDOCODE:
        // if neighbours.size = 0 AND !subtree_size_sent then CHECK()
        // else for each neighbour send type 1
        if (neighbours.empty() && !subtreeSizeSent) {
            Check();    // Phase 2: I am a leaf → report size
        } else {
            for (P2PNetworkInterface *n : neighbours) {
                sendMessage("Explore",
                            new Message(TYPE_1_EXPLORE),
                            n,
                            1000,
                            0);
            }
        }
    }

    // do NOT call Check() again here
}

void AssigningIDsBlockCode::confirmChild(std::shared_ptr<Message> msg,
                                         P2PNetworkInterface *sender) {
    console << "Module " << getId()
            << " Confirmed Child: " << sender->getConnectedBlockId() << "\n";
    module->setColor(GREEN);

    neighbours.erase(
        std::remove(neighbours.begin(), neighbours.end(), sender),
        neighbours.end()
    );

    children.push_back(sender);
    childrenSizes.push_back(0); // will be filled in reportSize

    if (neighbours.empty() && !subtreeSizeSent) {
        Check();
    }
}

void AssigningIDsBlockCode::rejectChild(std::shared_ptr<Message> msg, P2PNetworkInterface *sender) {
    console << "Module " << getId() << " Rejected Child: " << sender->getConnectedBlockId() << "\n";
    module->setColor(RED);

    // MessageOf<string>* m = static_cast<MessageOf<string>*>(msg.get());
    // string messageType = *(m->getData());

    // neighbours.push_back(sender);
    // childrenSizes.push_back(0);
    //
    if ( !subtreeSizeSent) {
        Check();
    }
}


void AssigningIDsBlockCode::reportSize(std::shared_ptr<Message> msg,
                                       P2PNetworkInterface *sender) {
    console << "At reporting message " << getId() << " Rejected Child: " << sender->getConnectedBlockId() << "\n";
    module->setColor(GREY);



    MessageOf<int>* m = static_cast<MessageOf<int>*>(msg.get());
    int childSize = *(m->getData());

    console << "Module " << getId()
            << " received subtree size " << childSize
            << " from child " << sender->getConnectedBlockId() << "\n";

    // 1. Find which child this is
    for (int i = 0; i < children.size(); i++) {
        if (children[i] == sender) {

            // 2. Store child's subtree size
            childrenSizes[i] = childSize;

            // 3. Add to my subtree size
            subtree_size += childSize;

            break;
        }
    }

    // 4. Call CHECK() to verify if all children reported
    if ( !subtreeSizeSent) {
        Check();
    }}

void AssigningIDsBlockCode::assignId(std::shared_ptr<Message> msg, P2PNetworkInterface *sender) {
    console << "Module " << getId() << " Assigning id " << sender->getConnectedBlockId() << "\n";
    module->setColor(BLUE);


    MessageOf<int>* m = static_cast<MessageOf<int>*>(msg.get());
    int final_id = *(m->getData());
    id_output << "Module " << getId() << " = " << final_id << "\n";
    id_output.flush();



    int next_id = *(m->getData()) + 1;



    for (int i = 0; i < children.size(); i++) {
        // int childStartId = next_id;

        // send type 5 message to child with its starting ID
        sendMessage("AssignId",
            new MessageOf<int>(TYPE_5_ASSIGN_ID, next_id),
                   children[i]
, 1000, 0);

        // next id += child.subtree size
        next_id += childrenSizes[i];   // childrenSizes[i] = child.subtree size
    }

    // MessageOf<string>* m = static_cast<MessageOf<string>*>(msg.get());
    // string messageType = *(m->getData());

    // neighbours.push_back(sender);
    // childrenSizes.push_back(0);
    //
    if ( !subtreeSizeSent) {
        Check();
    }
}


void AssigningIDsBlockCode::Check() {

    // Already sent → never send again
    if (subtreeSizeSent) return;

    // Case 1: Leaf node (children.size == 0)
    // Case 2: All children have sent subtree sizes
    bool allChildrenReported = true;

    for (int i = 0; i < children.size(); i++) {
        if (childrenSizes[i] == 0) {
            allChildrenReported = false;
            break;
        }
    }

    if (children.empty() || allChildrenReported) {

        if (!isLeader) {
            // Not the leader => send subtree size to parent
            sendMessage("ReportSize",
                new MessageOf<int>(TYPE_4_REPORT_SIZE, subtree_size),
                parent, 1000, 0);

            console << "Module " << getId()
                    << " sent subtree size = " << subtree_size
                    << " to parent " << parent->getConnectedBlockId()
                    << "\n";

            subtreeSizeSent = true;
        }
        else {
            id_output << "Module " << getId() << " = 0\n";
            id_output.flush();
           int unique_id = 0;
            int next_id = 1;



 for (int i = 0; i < children.size(); i++) {
        // int childStartId = next_id;

        // send type 5 message to child with its starting ID
        sendMessage("AssignId",
            new MessageOf<int>(TYPE_5_ASSIGN_ID, next_id),
                   children[i]
, 1000, 0);

        // next id += child.subtree size
        next_id += childrenSizes[i];   // childrenSizes[i] = child.subtree size
    }







            // Leader received all subtree sizes (or is leaf)
            console << "Leader " << getId()
                    << " finished Phase 2. Total system size = "
                    << subtree_size << "\n";

            //  calculate number of bits and start ID assignment
            // (Phase 3)
        }
    }


}