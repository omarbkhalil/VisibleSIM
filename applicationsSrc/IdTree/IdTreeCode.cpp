#include "IdTreeCode.hpp"

IdTreeCode::IdTreeCode(BlinkyBlocksBlock *host) : BlinkyBlocksBlockCode(host), module(host) {
    if (!host) return;

    // Register message types with their handlers
    addMessageEventFunc2(TYPE_1_EXPLORE, std::bind(&IdTreeCode::exploreNeighbors, this, std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(TYPE_2_CONFIRM_CHILD, std::bind(&IdTreeCode::confirmChild, this, std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(TYPE_3_REJECT_CHILD, std::bind(&IdTreeCode::rejectChild, this, std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(TYPE_4_REPORT_SIZE, std::bind(&IdTreeCode::reportSize, this, std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(TYPE_5_ASSIGN_ID, std::bind(&IdTreeCode::assignId, this, std::placeholders::_1, std::placeholders::_2));
}

void IdTreeCode::startup() {
    uniqueID = -1;
    subtreeSize = 1;
    isDiscovered = false;
    subtreeSizeSent = false;
    receivedSizeReports = 0;
    parentModule = nullptr;
    childrenModules.clear();
    childrenSizes.clear();

    if (isLeader) {
        isDiscovered = true;
        console << "Leader " << getId() << "\n";
        sendMessageToAllNeighbors("Explore", new Message(TYPE_1_EXPLORE), 1000, 0, 0);
    }
}

// Phase 1: Explore and build logical tree
void IdTreeCode::exploreNeighbors(std::shared_ptr<Message> msg, P2PNetworkInterface *sender) {
    console << "Module " << getId() << " rcv explore message: " << sender->getConnectedBlockId() << "\n";
    if (isDiscovered) { 
        sendMessage("RejectChild", new Message(TYPE_3_REJECT_CHILD), sender, 1000, 0);
    } else {
        isDiscovered = true;
        parentModule = sender;
        sendMessage("ConfirmChild", new Message(TYPE_2_CONFIRM_CHILD), sender, 1000, 0);
        sendMessageToAllNeighbors("Explore", new Message(TYPE_1_EXPLORE), 1000, 0, 1, parentModule);
    }
}

void IdTreeCode::confirmChild(std::shared_ptr<Message> msg, P2PNetworkInterface *sender) {
    console << "Module " << getId() << ",confirmed child!" << sender->getConnectedBlockId() << "\n";
    childrenModules.push_back(sender);
    childrenSizes.push_back(0);
}

void IdTreeCode::rejectChild(std::shared_ptr<Message> msg, P2PNetworkInterface *sender) {
    console << "Module " << getId() << ",rejected child!" << sender->getConnectedBlockId() << "\n";
}

// Phase 2
void IdTreeCode::reportSize(std::shared_ptr<Message> msg, P2PNetworkInterface *sender) {
    if (msg) {
        int childSubtreeSize = *static_cast<MessageOf<int>*>(msg.get())->getData();
        subtreeSize += childSubtreeSize;
        receivedSizeReports++;
        console << "Module " << getId() << " received ReportSize from " << sender->getConnectedBlockId()
                << " with subtree size " << childSubtreeSize << "\n";
    }

    CHECK_SubtreeSize();
}

void IdTreeCode::CHECK_SubtreeSize() {
    if (childrenModules.empty() || receivedSizeReports == static_cast<int>(childrenModules.size())) {
        if (!subtreeSizeSent) {
            if (parentModule == nullptr) {  // Leader
                cout << "Leader " << getId() << "Total size: " << subtreeSize << "\n";
                CHECK(1);  // Start ID assignment with nextID = 1
            } else {
                sendMessage("ReportSize", new MessageOf<int>(TYPE_4_REPORT_SIZE, subtreeSize), parentModule, 1000, 0);
                subtreeSizeSent = true;
            }
        }
    }
}

// Phase 3: Unique ID Assignment
void IdTreeCode::assignId(std::shared_ptr<Message> msg, P2PNetworkInterface *sender) {
    int baseID = *static_cast<MessageOf<int>*>(msg.get())->getData();
    uniqueID = baseID;
    int nextID = baseID + 1;

    console << "Module " << getId() << ",now has unique Id: " << uniqueID << "\n";
    CHECK(nextID);
}

void IdTreeCode::CHECK(int nextID) {
    if (isLeader) {
        uniqueID = 0;
        console << "Leader " << getId() << " begins assigning unique IDs.\n";
    }

    for (size_t i = 0; i < childrenModules.size(); ++i) {
        sendMessage("AssignID", new MessageOf<int>(TYPE_5_ASSIGN_ID, nextID), childrenModules[i], 1000, 0);
        console << "Module " << getId() << " sending ID " << nextID << " to child module " << childrenModules[i]->getConnectedBlockId() << "\n";
        nextID += childrenSizes[i];
    }
}

string IdTreeCode::onInterfaceDraw() {
    string display = "Unique Id: " + std::to_string(uniqueID);
    display += "\nSubtree Size: " + std::to_string(subtreeSize);
    return display;
}

void IdTreeCode::parseUserBlockElements(TiXmlElement *config) {
    const char *attr = config->Attribute("leader");
    if (attr != nullptr && strcmp(attr, "true") == 0) {
        isLeader = true;
        console << "Module " << getId() << "Is the leader (from XML).\n";
    }
}

