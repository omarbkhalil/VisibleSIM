/**
 * @file MyApp2plusBlockCode.cpp
 */

#include "MyApp2plusBlockCode.hpp"


BlinkyBlocksBlock *moduleA      = nullptr;
BlinkyBlocksBlock *moduleB      = nullptr;
BlinkyBlocksBlock *moduleC      = nullptr;
BlinkyBlocksBlock *moduleCenter = nullptr;

int distAB      = -1;
int distBC      = -1;
int distBcenter = -1;



MyApp2plusBlockCode::MyApp2plusBlockCode(BlinkyBlocksBlock *host)
: BlinkyBlocksBlockCode(host), module(host) {
    if (!host) return;

    addMessageEventFunc2(GO_MSG_ID,
        std::bind(&MyApp2plusBlockCode::myGoFunc, this,
                  std::placeholders::_1, std::placeholders::_2));

    addMessageEventFunc2(BACK_MSG_ID,
        std::bind(&MyApp2plusBlockCode::myBackFunc, this,
                  std::placeholders::_1, std::placeholders::_2));

    addMessageEventFunc2(GO2_MSG_ID,
        std::bind(&MyApp2plusBlockCode::myGo2Func, this,
                  std::placeholders::_1, std::placeholders::_2));

    addMessageEventFunc2(BACK2_MSG_ID,
        std::bind(&MyApp2plusBlockCode::myBack2Func, this,
                  std::placeholders::_1, std::placeholders::_2));
}

void MyApp2plusBlockCode::startup() {
    console << "start " << getId() << "\n";

    // Reset phase 1
    parent = nullptr; winnerChild = nullptr;
    myDistance = 0; nbWaitedAns = 0;
    bestDist = -1; bestId = 0;

    // Reset phase 2
    parent2 = nullptr; winnerChild2 = nullptr;
    myDistance2 = 0; nbWaitedAns2 = 0;
    bestDist2 = -1; bestId2 = 0;

    if (isA) {
        moduleA   = module;
        myDistance = 0;
        bestDist   = 0;
        bestId     = getId();
        setColor(RED);

        // Start GO flood (phase 1)
        nbWaitedAns = sendMessageToAllNeighbors("initial go from leader",
                           new MessageOf<int>(GO_MSG_ID, 1), 0, 0, 0);

        if (nbWaitedAns == 0) {
            parent2 = nullptr; winnerChild2 = nullptr;
            myDistance2 = 0; bestDist2 = 0; bestId2 = getId();
            nbWaitedAns2 = sendMessageToAllNeighbors("go2",
                               new MessageOf<int>(GO2_MSG_ID, 1), 0, 0, 0);
            if (nbWaitedAns2 == 0) setColor(BLUE);
        }
    } else {
        setColor(LIGHTGREY);
    }
}

// ========================= PHASE 1 (from leader) =========================

void MyApp2plusBlockCode::myGoFunc(std::shared_ptr<Message> _msg,
                                   P2PNetworkInterface* sender) {
    auto* msg = static_cast<MessageOf<int>*>(_msg.get());
    int msgData = *msg->getData(); // distance from leader

    if (parent == nullptr) {
        // First time we get GO: adopt this sender as parent in the BFS tree
        parent      = sender;
        myDistance  = msgData;

        bestDist    = myDistance;
        bestId      = getId();
        winnerChild = nullptr;

        // Forward GO to all neighbors except parent; count children
        nbWaitedAns = sendMessageToAllNeighbors("go",
                           new MessageOf<int>(GO_MSG_ID, msgData + 1),
                           0, 0, 1, sender);

        // If no children, leaf send BACK immediately to parent
        if (nbWaitedAns == 0) {
            BackPayload b{0, myDistance, getId(), 0};
            sendMessage("leafBack",
                        new MessageOf<BackPayload>(BACK_MSG_ID, b),
                        parent, 0, 0);
        }
    } else {
        // Already have a parent
        BackPayload b{0, -1, getId(), 0}; // dist=-1 so it never wins aggregation
        sendMessage("reject",
                    new MessageOf<BackPayload>(BACK_MSG_ID, b),
                    sender, 0, 0);
    }
}


void MyApp2plusBlockCode::myBackFunc(std::shared_ptr<Message> _msg,
                                     P2PNetworkInterface* sender) {
    auto* m = static_cast<MessageOf<BackPayload>*>(_msg.get());
    BackPayload bp = *m->getData();

    // SELECT_DOWN
    if (bp.kind == 1) {
        if (getId() == bp.id) {
            // farthest from the B
            setColor(YELLOW);
            moduleB  = module;
            distAB   = bp.dist;

            // ---- Start PHASE 2 from B ----
            parent2      = nullptr;
            winnerChild2 = nullptr;
            myDistance2  = 0;
            bestDist2    = 0;
            bestId2      = getId();

            nbWaitedAns2 = sendMessageToAllNeighbors("go2",
                               new MessageOf<int>(GO2_MSG_ID, 1), 0, 0, 0);

            if (nbWaitedAns2 == 0) setColor(BLUE); // degenerate case
        } else if (winnerChild) {
            // Forward only along the stored branch that produced the winner
            sendMessage("selectDown",
                        new MessageOf<BackPayload>(BACK_MSG_ID, bp),
                        winnerChild, 0, 0);
        }
        return;
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

            BackPayload up{0, bestDist, bestId, 0};
            sendMessage("backUp",
                        new MessageOf<BackPayload>(BACK_MSG_ID, up),
                        parent, 0, 0);
        } else {
            // Root decides winner
            console << "Phase1: farthest from leader = " << bestId
                    << " at dist = " << bestDist << "\n";

            distAB = bestDist;

            if (winnerChild) {
                // Send select-down
                BackPayload sel{1, bestDist, bestId, 0}; // SELECT_DOWN
                sendMessage("selectDown",
                            new MessageOf<BackPayload>(BACK_MSG_ID, sel),
                            winnerChild, 0, 0);
            }
        }
    }
}

// ========================= PHASE 2  =========================

void MyApp2plusBlockCode::myGo2Func(std::shared_ptr<Message> _msg,
                                    P2PNetworkInterface* sender) {
    auto* msg = static_cast<MessageOf<int>*>(_msg.get());
    int d = *msg->getData(); // distance from yellow

    if (parent2 == nullptr) {
        parent2     = sender;
        myDistance2 = d;

        if (bestDist2 < myDistance2 || (bestDist2 == myDistance2 && getId() < bestId2)) {
            bestDist2    = myDistance2;
            bestId2      = getId();
            winnerChild2 = nullptr;
        }

        // Forward GO2 to all except parent2
        nbWaitedAns2 = sendMessageToAllNeighbors("go2",
                           new MessageOf<int>(GO2_MSG_ID, d + 1),
                           0, 0, 1, sender);

        if (nbWaitedAns2 == 0) {
            BackPayload b{0, myDistance2, getId(), 0};
            sendMessage("back2-leaf",
                        new MessageOf<BackPayload>(BACK2_MSG_ID, b),
                        parent2, 0, 0);
        }
    } else {
        // Already in some other branch of GO2
        BackPayload b{0, -1, getId(), 0};
        sendMessage("back2-reject",
                    new MessageOf<BackPayload>(BACK2_MSG_ID, b),
                    sender, 0, 0);
    }
}

void MyApp2plusBlockCode::myBack2Func(std::shared_ptr<Message> _msg, P2PNetworkInterface* sender) {
    auto* m = static_cast<MessageOf<BackPayload>*>(_msg.get());
    BackPayload bp = *m->getData();

    // SELECT_DOWN for phase 2 (
    if (bp.kind == 1) {
        bp.distFromB++; // increment distance along BC path

        // Midpoint detection
        int centerDist = bestDist2 / 2;
        if (bp.distFromB == centerDist && moduleCenter == nullptr) {
            moduleCenter = module;
            distBcenter  = centerDist;
            setColor(BROWN);
            console << "Center of BC reached at id=" << getId()
                    << " (dist from B = " << distBcenter << ")\n";
        }

        if (getId() == bp.id) {
            // Farthest from BC
            moduleC = module;
            console << "Phase2: I am BLUE (id=" << getId()
                    << "), distance from yellow = " << bp.dist << "\n";
            setColor(BLUE); // farthest from yellow
        } else if (winnerChild2) {
            sendMessage("back2-select-down",
                        new MessageOf<BackPayload>(BACK2_MSG_ID, bp),
                        winnerChild2, 0, 0);
        }
        return;
    }

    // ACK_UP
    if (bp.dist > bestDist2 || (bp.dist == bestDist2 && bp.id < bestId2)) {
        bestDist2 = bp.dist;
        bestId2 = bp.id;
        winnerChild2 = sender;
    }

    nbWaitedAns2--;
    if (nbWaitedAns2 == 0) {
        if (parent2) {
            if (myDistance2 > bestDist2 || (myDistance2 == bestDist2 && getId() < bestId2)) {
                bestDist2 = myDistance2;
                bestId2 = getId();
                winnerChild2 = nullptr;
            }
            BackPayload up{0, bestDist2, bestId2, 0};
            sendMessage("back2-up",
                        new MessageOf<BackPayload>(BACK2_MSG_ID, up),
                        parent2, 0, 0);
        } else {
            // Yellow decides winner
            console << "Phase2: farthest from yellow = " << bestId2
                    << " at dist = " << bestDist2 << "\n";

            // Store BC diameter at the phase 2 root
            distBC = bestDist2;

            if (winnerChild2) {
                BackPayload sel{1, bestDist2, bestId2, 0}; // SELECT_DOWN
                sendMessage("back2-select",
                            new MessageOf<BackPayload>(BACK2_MSG_ID, sel),
                            winnerChild2, 0, 0);
            }
        }
    }
}

// ========================= XML parsing =========================

void MyApp2plusBlockCode::parseUserBlockElements(TiXmlElement *config) {
    const char *attr = config->Attribute("isA");
    if (attr != nullptr) {
        std::cout << getId() << " is isA!" << std::endl;
        isA = true;
    }
}
