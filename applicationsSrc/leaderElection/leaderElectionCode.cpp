#include "leaderElectionCode.hpp"

#include <thread>
#include <chrono>
int leaderElectionCode::globalLeaderId = -1;

leaderElectionCode::leaderElectionCode(BlinkyBlocksBlock *host)
    : BlinkyBlocksBlockCode(host), module(host) {
    if (!host) return;

    // Register message types with their handlers
    addMessageEventFunc2(TYPE_1_EXPLORE,
        [this](auto && PH1, auto && PH2) {
            exploreNeighbors(std::forward<decltype(PH1)>(PH1),
                             std::forward<decltype(PH2)>(PH2));
        });
    addMessageEventFunc2(TYPE_2_CONFIRM_CHILD,
        [this](auto && PH1, auto && PH2) {
            confirmChild(std::forward<decltype(PH1)>(PH1),
                         std::forward<decltype(PH2)>(PH2));
        });
    addMessageEventFunc2(TYPE_3_REJECT_CHILD,
        [this](auto && PH1, auto && PH2) {
            rejectChild(std::forward<decltype(PH1)>(PH1),
                        std::forward<decltype(PH2)>(PH2));
        });

    addMessageEventFunc2(TYPE_4_UPDATE_PLTREE,
        [this](auto && PH1, auto && PH2) {
            prospectiveLeaderTreeTotalWeightUpdate(
                std::forward<decltype(PH1)>(PH1),
                std::forward<decltype(PH2)>(PH2));
        });
    addMessageEventFunc2(TYPE_5_RESET,
        [this](auto && PH1, auto && PH2) {
            reset(std::forward<decltype(PH1)>(PH1),
                  std::forward<decltype(PH2)>(PH2));
        });
    addMessageEventFunc2(TYPE_6_DISMANTLE,
        [this](auto && PH1, auto && PH2) {
            dismantle(std::forward<decltype(PH1)>(PH1),
                      std::forward<decltype(PH2)>(PH2));
        });
    addMessageEventFunc2(TYPE_7_ELECT_LEADER,
        [this](auto && PH1, auto && PH2) {
            electLeader(std::forward<decltype(PH1)>(PH1),
                        std::forward<decltype(PH2)>(PH2));
        });
    addMessageEventFunc2(TYPE_8_DISMANTLE_TREE,
        [this](auto && PH1, auto && PH2) {
            dismantleTree(std::forward<decltype(PH1)>(PH1),
                          std::forward<decltype(PH2)>(PH2));
        });
    addMessageEventFunc2(TYPE_9_WIN_TREE,
        [this](auto && PH1, auto && PH2) {
            winTreeUpdate(std::forward<decltype(PH1)>(PH1),
                          std::forward<decltype(PH2)>(PH2));
        });
    addMessageEventFunc2(TYPE_10_NOTCUBE,
        [this](auto && PH1, auto && PH2) {
            notifyNeighborsNotACube(std::forward<decltype(PH1)>(PH1),
                                    std::forward<decltype(PH2)>(PH2));
        });
    addMessageEventFunc2(TYPE_11_CUBE_LEADER_UPDATE,
        [this](auto && PH1, auto && PH2) {
            notifyParentYouAreACubeLeader(std::forward<decltype(PH1)>(PH1),
                                          std::forward<decltype(PH2)>(PH2));
        });
    addMessageEventFunc2(TYPE_12_CUBE_CHECK,
        [this](auto && PH1, auto && PH2) {
            CheckIfCube(std::forward<decltype(PH1)>(PH1),
                        std::forward<decltype(PH2)>(PH2));
        });
    addMessageEventFunc2(TYPE_13_REST_IN_PROCESS,
        [this](auto && PH1, auto && PH2) {
            UpdateMyParent(std::forward<decltype(PH1)>(PH1),
                           std::forward<decltype(PH2)>(PH2));
        });
    sendMessageToAllNeighbors(
    "LeaderBroadcast",
    new MessageOf<int>(TYPE_14_LEADER_BROADCAST, (int)getId()),
    1000, 0, 0);

}

void leaderElectionCode::startup() {

    parentModule = nullptr;
    winTreeModuleParent = nullptr;
    isDiscovered = false;
    isExploring = false;
    isLeaf = false;
    isWinTreeProcessed = false;
    notACube = false;
    isProspectiveLeader = false;
    isRestting = false;
    isDismantling = false;

    childrenModules.clear();
    childrenSizes.clear();
    borderNeighbors.clear();
    readyToCompete = false;
    hasSentChoice  = false;
    binaryStringId = "";
    totalConnectedInt = 0;
    total = 0;
    nbWaitedAnswers = 0;
    myTreeTotalWeight = 0;
    for (int i = 0; i < module->getNbInterfaces(); i++) {
        if (module->getInterface(i)->isConnected()) {
            binaryStringId += "1";
            totalConnectedInt++;
        } else {
            binaryStringId += "0";
        }
    }

    binaryIntId = weight = std::strtol(binaryStringId.c_str(), nullptr, 2);
    console << "Block Weight = " << binaryIntId << "\n";

    if (totalConnectedInt == 1) {
        module->setColor(RED);
        isProspectiveLeader = true;
        isDiscovered = true;
        // notACube = true;
        console << "Leader " << getId() << "\n";
        colorId = NUM++;
        module->setColor(colorId);
        sendMessageToAllNeighbors("Not a cube!",
                                  new MessageOf<int>(TYPE_10_NOTCUBE, colorId),
                                  10, 0, 0);
        nbWaitedAnswers = sendMessageToAllNeighbors(
            "Explore",
            new MessageOf<int>(TYPE_1_EXPLORE, colorId),
            100, 0, 0);
    }
    else if (totalConnectedInt == 3 && weight > 37) {
        // waiting for 500000 ms to be sure 1-degree blocks started first
        colorId = NUM++;
        nbWaitedAnswers = sendMessageToAllNeighbors(
            "Cube Check",
            new MessageOf<int>(TYPE_12_CUBE_CHECK, colorId),
            500000, 0, 0);
    }
}

// Phase 1: Explore and build logical tree for each Prospective Leader;
void leaderElectionCode::exploreNeighbors(const std::shared_ptr<Message>& msg,
                                          P2PNetworkInterface *sender) {
    int colorID = *dynamic_cast<MessageOf<int>*>(msg.get())->getData();

    console << "Module " << getId()
            << " rcv explore message: " << sender->getConnectedBlockId() << "\n";

    if (isDiscovered) {
        // already recruited or root
        // if this explore comes from another tree (different color),
        // remember this interface as a border neighbor
        if (colorId != 0 && colorID != colorId) {    // colorId==0 means "not yet assigned"
            borderNeighbors.push_back(sender);
            console << "  -> border neighbor with other tree, color "
                    << colorID << " vs my color " << colorId << "\n";
        }

        sendMessage("RejectChild",
                    new Message(TYPE_3_REJECT_CHILD),
                    sender, 1000, 0);
    } else {
        childrenModules.clear();
        childrenSizes.clear();
        module->setColor(colorID);
        isDismantling = false;
        isRestting = false;
        isDiscovered = true;
        parentModule = sender;
        colorId = colorID;
        isExploring = false;

        nbWaitedAnswers = sendMessageToAllNeighbors(
            "Explore",
            new MessageOf<int>(TYPE_1_EXPLORE, colorID),
            1000, 0, 1, parentModule);

        if (nbWaitedAnswers == 0) {
            // I am a leaf
            isLeaf = true;
            console<<"I am leaf node";
            total = weight;  // leaf total is just its own weight
            sendMessage("ConfirmChild",
                        new MessageOf<int>(TYPE_2_CONFIRM_CHILD, total),
                        parentModule, 1000, 0);
        }
    }
    console << "nbWaitedAnswers From Explore: " << nbWaitedAnswers << "\n";
}


void leaderElectionCode::confirmChild(const std::shared_ptr<Message>& msg,
                                      P2PNetworkInterface *sender) {
    console << "Module " << getId()
            << ", confirmed child ->(" << sender->getConnectedBlockId() << ")\n";
    int childWeight = *dynamic_cast<MessageOf<int>*>(msg.get())->getData();

    childrenModules.push_back(sender);
    nbWaitedAnswers--;
    total += childWeight;

    if (nbWaitedAnswers == 0) {
        total += weight;
        if (isProspectiveLeader) { // root of THIS tree
            console << "total: " << total << "\n";

            // Prospective Leader total weight (for this tree)
            myTreeTotalWeight = total;

            // just send the tree total downwards
            if (!childrenModules.empty()) {
                for (auto &childrenModule : childrenModules) {
                    sendMessage("TreeTotalWightUpdate",
                                new MessageOf<int>(TYPE_4_UPDATE_PLTREE,
                                                   myTreeTotalWeight),
                                childrenModule, 1000, 0);
                }
            }
        }
        else if (isExploring) {
            console << "new total: " << total << "\n";
            isExploring = false;
            if (parentModule)
                sendMessage("back the new tree total to my parent!",
                            new MessageOf<int>(TYPE_9_WIN_TREE, total - weight),
                            parentModule, 1000, 0);
        }
        else {
            if (parentModule)
                sendMessage("ConfirmChild",
                            new MessageOf<int>(TYPE_2_CONFIRM_CHILD, total),
                            parentModule, 1000, 0);
        }
    }
    childrenSizes.push_back(0);
}

void leaderElectionCode::rejectChild(const std::shared_ptr<Message>& msg,
                                     P2PNetworkInterface *sender) {
    nbWaitedAnswers--;
    if (nbWaitedAnswers == 0) {
        total += weight;

        isLeaf = childrenModules.empty();
        console<<"I am leaf node";


        winTreeModuleParent = nullptr;
        if (parentModule)
            sendMessage("ConfirmChild",
                        new MessageOf<int>(TYPE_2_CONFIRM_CHILD, total),
                        parentModule, 1000, 0);
    }
    console << "Module " << getId()
            << ", rejected child!" << sender->getConnectedBlockId() << "\n";
    console << "nbWaitedAnswers From Reject: " << nbWaitedAnswers << "\n";
}


// Phase 2: Update each node on my Prospective Leader logical tree
void leaderElectionCode::prospectiveLeaderTreeTotalWeightUpdate(
    const std::shared_ptr<Message>& msg,
    P2PNetworkInterface *sender) {

    int totalWeight = *dynamic_cast<MessageOf<int>*>(msg.get())->getData();
    console << "Module " << getId()
            << ", Received Tree Total Weight (" << totalWeight << ")\n";

    myTreeTotalWeight = totalWeight;
    readyToCompete = true;                   // NEW: we now know the tree value

    // If I'm a leaf and I have neighbors from other trees, start the competition
    if (isLeaf && !borderNeighbors.empty() && !hasSentChoice) {
        hasSentChoice = true;
        console<<"I am leaf node";
        console << "Module " << getId()
                << " sending CHOICE to " << borderNeighbors.size()
                << " border neighbors, treeWeight=" << myTreeTotalWeight << "\n";

        for (auto *bn : borderNeighbors) {
            sendMessage("Choice / ElectLeader",
                        new MessageOf<int>(TYPE_7_ELECT_LEADER,
                                           myTreeTotalWeight),
                        bn, 5000, 0);
        }
    }

    // still propagate the tree total INSIDE my tree
    if (!childrenModules.empty()) {
        for (auto &childrenModule : childrenModules) {
            sendMessage("TreeTotalWightUpdate",
                        new MessageOf<int>(TYPE_4_UPDATE_PLTREE, totalWeight),
                        childrenModule, 1000, 0);
        }
    }
}


//Phase 3: Start Leader Election
void leaderElectionCode::electLeader(const std::shared_ptr<Message>& msg,
                                     P2PNetworkInterface *sender) {
    int otherTreeWeight =
        *dynamic_cast<MessageOf<int>*>(msg.get())->getData();

    console << "Module: " << getId()
            << " ElectLeader / CHOICE received from block "
            << sender->getConnectedBlockId()
            << " otherTreeWeight=" << otherTreeWeight
            << " myTreeWeight=" << myTreeTotalWeight << "\n";

    if (!readyToCompete) {
        console << "  -> not ready to compete yet, ignoring\n";
        return;
    }

    if (otherTreeWeight > myTreeTotalWeight) {
        // My tree loses against this neighbor tree
        console << "  -> My tree LOST this comparison\n";
        winTreeModuleParent = sender;
        int id = static_cast<int>(getId());
        if (parentModule) {
            sendMessage("You lost; dismantle your tree!",
                        new MessageOf<int>(TYPE_6_DISMANTLE, id),
                        parentModule, 100, 0);
        }
    }
    else if (otherTreeWeight < myTreeTotalWeight) {
        // My tree wins against this neighbor tree
        console << "  -> My tree WON this comparison\n";

        // send a WIN-TREE token upwards so the root can eventually declare itself leader
        if (parentModule) {
            sendMessage("WinTree token to parent",
                        new MessageOf<int>(TYPE_9_WIN_TREE, myTreeTotalWeight),
                        parentModule, 1000, 0);
        }
    }
    else {
        // equal weights → tie-breaking rule could be added here if needed
        console << "  -> tie (same tree weight), no action\n";
    }
}


//Phase 4: if my TreeTotalWeight win the election; restart explore
void leaderElectionCode::reset(const std::shared_ptr<Message>& msg,
                               P2PNetworkInterface *sender) {
    if (!isExploring) {
        console << "Module " << getId() << ", Won Message Received!\n";
        total = 0;
        isExploring = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        nbWaitedAnswers = sendMessageToAllNeighbors(
            "Explore",
            new MessageOf<int>(TYPE_1_EXPLORE, colorId),
            1000, 0, 1, parentModule);
    }
}

void leaderElectionCode::winTreeUpdate(const std::shared_ptr<Message>& msg,
                                       P2PNetworkInterface *sender)
{
    int winningWeight = *dynamic_cast<MessageOf<int>*>(msg.get())->getData();

    // If I have a parent → forward token upward
    if (parentModule) {
        sendMessage("forward WinTree token",
                    new MessageOf<int>(TYPE_9_WIN_TREE, winningWeight),
                    parentModule, 1000, 0);
        return;
    }

    // I am the ROOT of the winning tree
    if (isProspectiveLeader) {
        myTreeTotalWeight = winningWeight;

        console << "GLOBAL LEADER: " << getId()
                << " with tree weight " << winningWeight << "\n";

        // Mark leader visually
        module->setColor(YELLOW);

        // 🔥 Send LEADER ID to every module
        sendMessageToAllNeighbors(
            "LeaderBroadcast",
            new MessageOf<int>(TYPE_14_LEADER_BROADCAST, (int)getId()),
            1000, 0, 0);
    }
}

void leaderElectionCode::leaderBroadcast(const std::shared_ptr<Message>& msg,
                                         P2PNetworkInterface *sender)
{
    int leaderId = *dynamic_cast<MessageOf<int>*>(msg.get())->getData();

    // Store global leader id
    globalLeaderId = leaderId;

    console << "Module " << getId()
            << " learned global leader = " << leaderId << "\n";

    // Optional: color nodes to show they belong to leader's final shape
    module->setColor(YELLOW);

    // Flood the message to all neighbors except sender
    sendMessageToAllNeighbors(
        "LeaderBroadcast",
        new MessageOf<int>(TYPE_14_LEADER_BROADCAST, leaderId),
        1000, 0, 1, sender);
}

//Phase 5: if my TreeTotalWeight lost the election; dismantle my tree
void leaderElectionCode::dismantle(const std::shared_ptr<Message>& msg,
                                   P2PNetworkInterface *sender) {
    console << "Module " << getId() << ", dismantle!\n";
    int senderId = *dynamic_cast<MessageOf<int>*>(msg.get())->getData();

    if (parentModule == nullptr && isProspectiveLeader) {
        if (!isRestting && !isExploring && !isDismantling) {
            isDiscovered = false;
            isLeaf = false;
            isDismantling = true;
            isProspectiveLeader = false;
            total = 0;
            if (!childrenModules.empty()) {
                for (auto &childrenModule : childrenModules) {
                    sendMessage("DismantleTree",
                                new MessageOf<int>(TYPE_8_DISMANTLE_TREE,
                                                   senderId),
                                childrenModule, 100, 0);
                }
            }
        }
    }
    else {
        if (isLeaf) winTreeModuleParent = sender;
        if (parentModule)
            sendMessage("You lost message forward to parent!",
                        new MessageOf<int>(TYPE_6_DISMANTLE, senderId),
                        parentModule, 100, 0);
    }
}

void leaderElectionCode::dismantleTree(const std::shared_ptr<Message>& msg,
                                       P2PNetworkInterface *sender) {
    int senderId = *dynamic_cast<MessageOf<int>*>(msg.get())->getData();
    isDiscovered = false;
    total = 0;
    console << "senderId:____" << senderId << "\n";
    if (static_cast<int>(getId()) == senderId) {
        sendMessage("You won; start leader election again!",
                    new MessageOf<int>(TYPE_5_RESET, colorId),
                    winTreeModuleParent, 3000, 0);
    }
    else if (!childrenModules.empty()) {
        for (auto &childrenModule : childrenModules) {
            sendMessage("DismantleTree",
                        new MessageOf<int>(TYPE_8_DISMANTLE_TREE, senderId),
                        childrenModule, 100, 0);
        }
    }
}

// Helper Functions: Cube Shape detection
void leaderElectionCode::notifyNeighborsNotACube(
    const std::shared_ptr<Message>& msg,
    P2PNetworkInterface *sender) {
    if (!notACube) {
        notACube = true;
        sendMessageToAllNeighbors("Not a cube!",
                                  new MessageOf<int>(TYPE_10_NOTCUBE, 0),
                                  10, 0, 1, sender);
    }
}

void leaderElectionCode::CheckIfCube(const std::shared_ptr<Message>& msg,
                                     P2PNetworkInterface *sender) {
    int receivedColorId = *dynamic_cast<MessageOf<int>*>(msg.get())->getData();
    if (!notACube) {
        sendMessage("notifyParentYouAreACubeLeader",
                    new MessageOf<int>(TYPE_11_CUBE_LEADER_UPDATE,
                                       receivedColorId),
                    sender, 100000, 0);
    }
}

void leaderElectionCode::notifyParentYouAreACubeLeader(
    const std::shared_ptr<Message>& msg,
    P2PNetworkInterface *sender) {
    int receivedColorId = *dynamic_cast<MessageOf<int>*>(msg.get())->getData();
    if (!isDiscovered) {
        isProspectiveLeader = true;
        console << "Leader " << getId() << "\n";
        colorId = receivedColorId;
        module->setColor(colorId);
        isDiscovered = true;
        nbWaitedAnswers = sendMessageToAllNeighbors(
            "Explore",
            new MessageOf<int>(TYPE_1_EXPLORE, colorId),
            2000, 0, 0);
    }
}

void leaderElectionCode::UpdateMyParent(const std::shared_ptr<Message>& msg,
                                        P2PNetworkInterface *sender) {
    if (parentModule == nullptr && isProspectiveLeader && !isRestting) {
        // don't start any rest; I'm updating you
        isRestting = true;
    }
    else {
        if (parentModule)
            sendMessage("notifyParentIAmRestingYou",
                        new MessageOf<int>(TYPE_13_REST_IN_PROCESS, 0),
                        parentModule, 100, 0);
    }
}

string leaderElectionCode::onInterfaceDraw() {
    string display = "Block Id: " + std::to_string(getId()) + "\n";
    display += "my Weight (From Binary Id): " + std::to_string(weight) + "\n";
    display += "my Tree Total Weight: " + std::to_string(total) + "\n";
    display += "my Prospective Leader Total Weight: " +
               std::to_string(myTreeTotalWeight) + "\n";
    display += "is This a Leaf Block: " + std::to_string(isLeaf) + "\n";
    return display;
}

void leaderElectionCode::parseUserBlockElements(TiXmlElement *config) {}