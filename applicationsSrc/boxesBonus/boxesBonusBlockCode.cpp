#include "BoxesBonusBlockCode.hpp"
#include <vector>
#include <utility>
using namespace BlinkyBlocks;

BoxesBonusBlockCode::BoxesBonusBlockCode(BlinkyBlocksBlock *host) : BlinkyBlocksBlockCode(host) {
    if (not host) return;

    addMessageEventFunc2(D_MSG_ID,
        std::bind(&BoxesBonusBlockCode::handleDMessage, this,
                  std::placeholders::_1, std::placeholders::_2));

    addMessageEventFunc2(DL_MSG_ID,
        std::bind(&BoxesBonusBlockCode::handleDLMessage, this,
                  std::placeholders::_1, std::placeholders::_2));

    addMessageEventFunc2(DLR_MSG_ID,
        std::bind(&BoxesBonusBlockCode::handleDLRMessage, this,
                  std::placeholders::_1, std::placeholders::_2));

    addMessageEventFunc2(W_MSG_ID,
        std::bind(&BoxesBonusBlockCode::handleWMessage, this,
                  std::placeholders::_1, std::placeholders::_2));

    addMessageEventFunc2(Find_W_MSG_ID,
       std::bind(&BoxesBonusBlockCode::handleFindW, this,
                 std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(Set_W_MSG_ID,
        std::bind(&BoxesBonusBlockCode::handleSetW, this,
                  std::placeholders::_1, std::placeholders::_2));

    addMessageEventFunc2(BOX_COLOR_MSG_ID,
           std::bind(&BoxesBonusBlockCode::handleBoxColorMessage, this,
                     std::placeholders::_1, std::placeholders::_2));

    addMessageEventFunc2(H_MSG_ID,
    std::bind(&BoxesBonusBlockCode::handleHMessage, this,
              std::placeholders::_1, std::placeholders::_2));

    addMessageEventFunc2(H_REPLY_ID,
        std::bind(&BoxesBonusBlockCode::handleHReply, this,
                  std::placeholders::_1, std::placeholders::_2));


    module = static_cast<BlinkyBlocksBlock*>(hostBlock);
}

void BoxesBonusBlockCode::startup() {
    console << "start\n";

    //  H phase initialization
    int initH = 0;
    bool hasVerticalNeighbor = false;

    Cell3DPosition above = module->position.offsetZ(+1);
    Cell3DPosition below = module->position.offsetZ(-1);

    if (lattice->cellHasBlock(above)) {
        sendMessage("H_MSG",
            new MessageOf<int>(H_MSG_ID, initH),
            module->getInterfaceToNeighborPos(above), 100, 200);
        hasVerticalNeighbor = true;
        console << "[H_INIT] Module " << module->blockId
                << " → sent H_MSG(h=" << initH << ") up to "
                << lattice->getBlock(above)->blockId << "\n";
    }

    if (lattice->cellHasBlock(below)) {
        sendMessage("H_MSG",
            new MessageOf<int>(H_MSG_ID, initH),
            module->getInterfaceToNeighborPos(below), 100, 200);
        hasVerticalNeighbor = true;
        console << "[H_INIT] Module " << module->blockId
                << " → sent H_MSG(h=" << initH << ") down to "
                << lattice->getBlock(below)->blockId << "\n";
    }

    if (!hasVerticalNeighbor) {
        hVal = 0;
        console << "[H_INIT] Module " << module->blockId
                << " has no vertical neighbors → h=" << hVal << "\n";
    }

    // === Normal depth initialization (unchanged) ===
    if (!lattice->cellHasBlock(module->position.offsetY(1))) {
        dp = 1;
        if (module->getInterfaceToNeighborPos(module->position.offsetY(-1))->isConnected()) {
            sendMessage("Depth Initialization",
                new MessageOf<int>(D_MSG_ID, dp),
                module->getInterfaceToNeighborPos(module->position.offsetY(-1)),
                100, 200);
        }
        setColor(YELLOW);
    } else {
        dp = 0;
        hostBlock->setColor(LIGHTGREY);
    }
}


void BoxesBonusBlockCode::handleWMessage(std::shared_ptr<Message> _msg,
                                    P2PNetworkInterface* sender) {
    auto m = std::static_pointer_cast<MessageOf<WNotify>>(_msg);
    WNotify wn = *m->getData();
    w = wn.w; // store width if you want
    // forward along same Y direction
    Cell3DPosition n = module->position.offsetY(wn.dy);
    if (lattice->cellHasBlock(n)) {
        sendMessage("W_NOTIFY",
            new MessageOf<WNotify>(W_MSG_ID, wn),
            module->getInterfaceToNeighborPos(n), 100, 200);
    }
}
void BoxesBonusBlockCode::notifyLineW(int wVal, int dy) {
    Cell3DPosition n = module->position.offsetY(dy);
    if (lattice->cellHasBlock(n)) {
        WNotify wn; wn.w = wVal; wn.dy = dy;
        sendMessage("W_NOTIFY",
            new MessageOf<WNotify>(W_MSG_ID, wn),
            module->getInterfaceToNeighborPos(n), 100, 200);
    }
}
void BoxesBonusBlockCode::handleDMessage(std::shared_ptr<Message> _msg,
                                    P2PNetworkInterface* sender) {
    auto msg = std::static_pointer_cast<MessageOf<int>>(_msg);
    int drecv = *msg->getData();

    dp = drecv + 1;
    depthReady = true;  // mark that I now know my depth

    console << "Module " << module->blockId << " received D=" << drecv
            << " → set dp=" << dp << "\n";
    //  Otherwise, ask left neighbor for its depth
    if (waitingForLeft) {
        waitingForLeft = false;
        console << "Module " << module->blockId << " sends d to right...\n";
        sendMessage("DL",
            new MessageOf<int>(DL_MSG_ID, dp),
            module->getInterfaceToNeighborPos(module->position.offsetX(1)), 100, 200);
    }
    // Process any pending DL requests that were waiting for my depth
    // if (!pendingDLRequests.empty()) {
    //     console << "   -> Replying to " << pendingDLRequests.size()
    //             << " pending DL_REQ(s)\n";
    //     for (auto *iface : pendingDLRequests) {
    //         sendMessage("DL_REP",
    //             new MessageOf<int>(DL_MSG_ID, dp),
    //             iface, 100, 200);
    //     }
    //     pendingDLRequests.clear();
    // }

    //  Forward d along -Y if someone is in front
    Cell3DPosition frontPos = module->position.offsetY(-1);
    if (lattice->cellHasBlock(frontPos)) {
        sendMessage("SET_D_MSG",
            new MessageOf<int>(D_MSG_ID, dp),
            module->getInterfaceToNeighborPos(frontPos), 100, 200);
        return;
    }

    //  End of column ,check left neighbor
    Cell3DPosition leftPos = module->position.offsetX(-1);

    if (!lattice->cellHasBlock(leftPos)) {
        // No left neighbor, evaluate Rmin
        if (module->getInterfaceToNeighborPos(module->position.offsetX(1))->isConnected() &&
            !module->getInterfaceToNeighborPos(module->position.offsetX(-1))->isConnected()) {
            console << "Module " << module->blockId << " is Rmin!\n";
         //   setColor(CYAN);

            FindWPayload fw;
            fw.originId = module->blockId;
            fw.d_sent   = dp;

            sendMessage("FIND_W",
                new MessageOf<FindWPayload>(Find_W_MSG_ID, fw),
                module->getInterfaceToNeighborPos(module->position.offsetX(1)), 100, 200);
        }
    } else {
        sendMessage("DL_REQ", new MessageOf<int>(DLR_MSG_ID, dp), module->getInterfaceToNeighborPos(leftPos), 100, 200);
    }


}

// ask left neighbor for d_left
void BoxesBonusBlockCode::handleDLRMessage(std::shared_ptr<Message> _msg,
                                      P2PNetworkInterface* sender) {
    auto msg = std::static_pointer_cast<MessageOf<int>>(_msg);
    int dr = *msg->getData();

    console << "Module " << module->blockId
            << " got DL_REQ from " << sender->getConnectedBlockId()
            << " (dr=" << dr << ")\n";

    //If I already know my depth, reply immediately
    if (depthReady) {
        console << "   -> Depth ready, replying immediately with d=" << dp << "\n";
        sendMessage("DL_REP",
            new MessageOf<int>(DL_MSG_ID, dp),
            sender, 100, 200);
    } else {
        // Otherwise, store this request to reply later
        waitingForLeft = true;
        // console << "   -> Depth not ready, storing request for later reply.\n";
        // pendingDLRequests.push_back(sender);
    }
}

void BoxesBonusBlockCode::handleDLMessage(std::shared_ptr<Message> _msg,
                                     P2PNetworkInterface* sender) {
    auto msg = std::static_pointer_cast<MessageOf<int>>(_msg);
    int d_left = * msg->getData();

    console << "Module " << module->blockId
            << " received d_left=" << d_left
            << " from left neighbor " << sender->getConnectedBlockId() << "\n";

    // Rmin check
    Cell3DPosition frontPos = module->position.offsetY(-1);
    bool isFrontEmpty = !lattice->cellHasBlock(frontPos);

    Cell3DPosition leftPos = module->position.offsetX(-1);
    Cell3DPosition leftFrontPos = leftPos.offsetY(-1);
    bool leftFrontEmpty = !lattice->cellHasBlock(leftFrontPos);

    if (isFrontEmpty && (dp != d_left || !leftFrontEmpty)) {
        console << "Module " << module->blockId << " is Rmin!\n";
     //   setColor(RED);

        Cell3DPosition rightPos = module->position.offsetX(+1);
        if (lattice->cellHasBlock(rightPos)) {
            FindWPayload fw; fw.originId = module->blockId; fw.d_sent = dp;
            sendMessage("FIND_W",
                new MessageOf<FindWPayload>(Find_W_MSG_ID, fw),
                module->getInterfaceToNeighborPos(rightPos), 100, 200);
        } else {
            w = 1;
         //   module->setColor(RED);            //  keep Rmin visible as red

            createBoxForRmin(w, dp);
            notifyLineW(w, -1);               // notify front line (−Y)
            return;

        }
    } else {
       // setColor(BLACK);
    }
}


void BoxesBonusBlockCode::handleFindW(std::shared_ptr<Message> _msg,
                                 P2PNetworkInterface* sender) {
    auto m = std::static_pointer_cast<MessageOf<FindWPayload>>(_msg);
    FindWPayload p = *m->getData();

    console << "[FIND_W] Block " << module->blockId
            << " dp=" << dp << " d_sent=" << p.d_sent << "\n";


    if (dp < p.d_sent) {
        console << "   -> Too short! Sending SET_W(w=0) back left\n";
        Cell3DPosition leftX = module->position.offsetX(-1); // go back toward Rmin
        if (lattice->cellHasBlock(leftX)) {
            SetWPayload sw;
            sw.originId = p.originId;
            sw.d_sent   = p.d_sent;
            sw.w_sent   = 0;

            sendMessage("SET_W",
                new MessageOf<SetWPayload>(Set_W_MSG_ID, sw),
                module->getInterfaceToNeighborPos(leftX), 100, 200);
        }
        return;
    }

    // If there is NO right neighbor → we are the last valid column
    Cell3DPosition rightX = module->position.offsetX(+1);
    if (!lattice->cellHasBlock(rightX)) {
        int wLocal = 1;
        console << "   -> Last column, returning SET_W(w=1) to the left\n";

        // Notify BACK line of width (as in paper lines 21–23)
        notifyLineW(wLocal, +1);

        // Return to origin with SET_W along the row (X−1)
        Cell3DPosition leftX = module->position.offsetX(-1);
        if (lattice->cellHasBlock(leftX)) {
            SetWPayload sw;
            sw.originId = p.originId;
            sw.d_sent   = p.d_sent;
            sw.w_sent   = wLocal;

            sendMessage("SET_W",
                new MessageOf<SetWPayload>(Set_W_MSG_ID, sw),
                module->getInterfaceToNeighborPos(leftX), 100, 200);
        }
        return;
    }

    //  continue probing to the right (X+1)
    console << "   -> Forwarding FIND_W to block on the right\n";
    sendMessage("FIND_W",
        new MessageOf<FindWPayload>(Find_W_MSG_ID, p),
        module->getInterfaceToNeighborPos(rightX), 100, 200);
}

void BoxesBonusBlockCode::handleSetW(std::shared_ptr<Message> _msg,
                                P2PNetworkInterface* sender) {
    auto m = std::static_pointer_cast<MessageOf<SetWPayload>>(_msg);
    SetWPayload s = *m->getData();

    // Only columns with enough height participate
    if (dp >= s.d_sent) {
        w = s.w_sent + 1;

        // Notify BACK line of my width
        notifyLineW(w, +1);

        if (module->blockId == s.originId) {
            console << "[WIDTH DONE] Rmin " << module->blockId
                    << " final w=" << w << "\n";
           // module->setColor(RED);        //keep Rmin visible as red
            createBoxForRmin(w, dp);
            return;
        }

        // Keep sending left (X−1)
        Cell3DPosition leftX = module->position.offsetX(-1);
        if (lattice->cellHasBlock(leftX)) {
            SetWPayload next;
            next.originId = s.originId;
            next.d_sent   = s.d_sent;
            next.w_sent   = w;

            sendMessage("SET_W",
                new MessageOf<SetWPayload>(Set_W_MSG_ID, next),
                module->getInterfaceToNeighborPos(leftX), 100, 200);
        }
    }
}
void BoxesBonusBlockCode::createBoxForRmin(int wVal, int dVal) {
    Cell3DPosition pos = module->position;

    //  Collect column heights inside the candidate box
    std::vector<int> heights;
    heights.reserve(wVal * dVal);
    int minH = INT_MAX, maxH = INT_MIN;

    for (int dx = 0; dx < wVal; ++dx) {
        for (int dy = 0; dy < dVal; ++dy) {
            Cell3DPosition c = pos + Cell3DPosition(dx, dy, 0);
            if (!lattice->cellHasBlock(c)) continue;
            auto* b = (BlinkyBlocksBlock*) lattice->getBlock(c);
            auto* code = (BoxesBonusBlockCode*) b->blockCode;
            heights.push_back(code->hVal);
            minH = std::min(minH, code->hVal);
            maxH = std::max(maxH, code->hVal);
        }
    }

    if (heights.empty()) return;

    //  If all equal create box
    if (minH == maxH) {
        console << "[BOX] Uniform height h=" << minH
                << " → creating single box\n";
        buildBoxAtHeight(pos, wVal, dVal, minH);
        return;
    }

    //multiple vertical groups exist
    console << "[BOX SPLIT] Rmin " << module->blockId
            << " detected height variation z∈[" << minH << "," << maxH
            << "] → recreating per-height boxes\n";

    for (int targetH = minH; targetH <= maxH; ++targetH)
        buildBoxAtHeight(pos, wVal, dVal, targetH);
}


// build one box at given height layer
void BoxesBonusBlockCode::buildBoxAtHeight(const Cell3DPosition& origin,
                                           int wVal, int dVal, int targetH) {
    // Compute lowest Z for this layer
    int minZ = origin[2];
    for (int dx = 0; dx < wVal; ++dx) {
        for (int dy = 0; dy < dVal; ++dy) {
            int localZ = origin[2];
            while (lattice->cellHasBlock(Cell3DPosition(origin[0]+dx,
                                                        origin[1]+dy,
                                                        localZ - 1))) {
                --localZ;
            }
            if (localZ < minZ) minZ = localZ;
        }
    }

    // Generate color unique per height layer
    srand(module->blockId * 97 + targetH * 31);
    Color boxColor(50 + rand() % 205, 50 + rand() % 205, 50 + rand() % 205);



    // Color only blocks of that exact height(using global view exceptionally)
    for (int dx = 0; dx < wVal; ++dx) {
        for (int dy = 0; dy < dVal; ++dy) {
            for (int dz = 0; dz <= targetH; ++dz) {
                Cell3DPosition cell(origin[0]+dx, origin[1]+dy, minZ+dz);
                if (!lattice->cellHasBlock(cell)) continue;
                auto* b = (BlinkyBlocksBlock*) lattice->getBlock(cell);
                auto* code = (BoxesBonusBlockCode*) b->blockCode;
                if (code->hVal == targetH) b->setColor(boxColor);
            }
        }
    }
}





void BoxesBonusBlockCode::onMotionEnd() {
    console << " has reached its destination\n";
}

void BoxesBonusBlockCode::processLocalEvent(EventPtr pev) {
    BlinkyBlocksBlockCode::processLocalEvent(pev);

    switch (pev->eventType) {
        case EVENT_ADD_NEIGHBOR: break;
        case EVENT_REMOVE_NEIGHBOR: break;
    }
}

void BoxesBonusBlockCode::onBlockSelected() {
    cerr << endl << "--- PRINT MODULE " << *module << "---" << endl;
    console << "d=" << d << "\n";
    console<< "dp=" << dp << "\n";
    console << "w=" << w << "\n";
}

void BoxesBonusBlockCode::onAssertTriggered() {
    console << " has triggered an assert\n";
}

bool BoxesBonusBlockCode::parseUserCommandLineArgument(int &argc, char **argv[]) {
    if ((argc > 0) && ((*argv)[0][0] == '-')) {
        switch((*argv)[0][1]) {
            case 'b':
                cout << "-b option provided" << endl;
                return true;
            case '-': {
                string varg = string((*argv)[0] + 2);
                if (varg == string("foo")) {
                    int fooArg = stoi((*argv)[1]);
                    argc--;
                    (*argv)++;
                    cout << "--foo option provided with value: " << fooArg << endl;
                } else return false;
                return true;
            }
            default:
                cerr << "Unrecognized argument: " << (*argv)[0] << endl;
        }
    }
    return false;
}

string BoxesBonusBlockCode::onInterfaceDraw() {
    stringstream trace;
    trace << "Some value " << 123;
    return trace.str();
}

    // propagate further
    void BoxesBonusBlockCode::handleBoxColorMessage(std::shared_ptr<Message> _msg,
                                                P2PNetworkInterface* sender) {
    auto* msg = static_cast<MessageOf<MyBoxMsg>*>(_msg.get());
    MyBoxMsg data = *msg->getData();

    Cell3DPosition here = module->position;
    Cell3DPosition rel = here - data.origin;

    // color if inside region
    if (rel[0] >= 0 && rel[0] < data.wVal &&
        rel[1] >= 0 && rel[1] < data.dVal &&
        rel[2] == 0) {
        module->setColor(data.boxColor);
    }

    // propagate the data forward
    sendMessageToAllNeighbors(
     new MessageOf<MyBoxMsg>(BOX_COLOR_MSG_ID, data),
     0,
     0,
     sender->getConnectedBlockId()
 );

}
void BoxesBonusBlockCode::handleHReply(std::shared_ptr<Message> _msg,
                                       P2PNetworkInterface* sender) {
    auto msg = std::static_pointer_cast<MessageOf<int>>(_msg);
    int totalH = *msg->getData();
    hVal = std::max(hVal, totalH);

    console << "[H_REPLY] Module " << module->blockId
            << " received H_REPLY(h=" << totalH
            << ") from neighbor " << sender->getConnectedBlockId() << "\n";

    // Propagate reply back toward Rmin
    Cell3DPosition above = module->position.offsetZ(+1);
    Cell3DPosition below = module->position.offsetZ(-1);

    bool sent = false;

    if (lattice->cellHasBlock(above) &&
        sender != module->getInterfaceToNeighborPos(above)) {
        console << "   ↳ forwarding H_REPLY(h=" << totalH << ") up to "
                << lattice->getBlock(above)->blockId << "\n";
        sendMessage("H_REPLY",
            new MessageOf<int>(H_REPLY_ID, totalH),
            module->getInterfaceToNeighborPos(above), 100, 200);
        sent = true;
        }

    if (lattice->cellHasBlock(below) &&
        sender != module->getInterfaceToNeighborPos(below)) {
        console << "   ↳ forwarding H_REPLY(h=" << totalH << ") down to "
                << lattice->getBlock(below)->blockId << "\n";
        sendMessage("H_REPLY",
            new MessageOf<int>(H_REPLY_ID, totalH),
            module->getInterfaceToNeighborPos(below), 100, 200);
        sent = true;
        }

    if (!sent) {
        console << " [Rmin] Module " << module->blockId
                << " confirmed total column height = " << totalH << "\n";
    }
}



void BoxesBonusBlockCode::handleHMessage(std::shared_ptr<Message> _msg,
                                         P2PNetworkInterface* sender) {
    auto msg = std::static_pointer_cast<MessageOf<int>>(_msg);
    int hRecv = *msg->getData();
    int myH = hRecv + 1;
    hVal = myH;

    console << "[H_MSG] Module " << module->blockId
            << " received h=" << hRecv
            << " from neighbor " << sender->getConnectedBlockId()
            << " → new h=" << myH << "\n";

    // Determine neighbors (up/down along Z)
    Cell3DPosition below = module->position.offsetZ(-1);
    Cell3DPosition above = module->position.offsetZ(+1);

    bool sent = false;

    // Propagate downward
    if (lattice->cellHasBlock(below) &&
        sender != module->getInterfaceToNeighborPos(below)) {
        console << "   ↳ sending H_MSG(h=" << myH << ") down to "
                << lattice->getBlock(below)->blockId << "\n";
        sendMessage("H_MSG",
            new MessageOf<int>(H_MSG_ID, myH),
            module->getInterfaceToNeighborPos(below), 100, 200);
        sent = true;
        }

    // Propagate upward
    if (lattice->cellHasBlock(above) &&
        sender != module->getInterfaceToNeighborPos(above)) {
        console << "   ↳ sending H_MSG(h=" << myH << ") up to "
                << lattice->getBlock(above)->blockId << "\n";
        sendMessage("H_MSG",
            new MessageOf<int>(H_MSG_ID, myH),
            module->getInterfaceToNeighborPos(above), 100, 200);
        sent = true;
        }

    // If no other direction left, send back the final reply
    if (!sent) {
        console << "   ↩ edge reached at " << module->blockId
                << " → sending H_REPLY(h=" << myH << ") back to "
                << sender->getConnectedBlockId() << "\n";
        sendMessage("H_REPLY",
            new MessageOf<int>(H_REPLY_ID, myH),
            sender, 100, 200);
    }
}



bool BoxesBonusBlockCode::isUniformHeight(const Cell3DPosition &origin, int wVal, int dVal) {
    int baseH = 0;
    if (lattice->cellHasBlock(origin)) {
        BlinkyBlocksBlock* b = (BlinkyBlocksBlock*) lattice->getBlock(origin);
        BoxesBonusBlockCode* code = (BoxesBonusBlockCode*) b->blockCode;
        baseH = code->hVal;
    }

    for (int dx = 0; dx < wVal; ++dx) {
        for (int dy = 0; dy < dVal; ++dy) {
            Cell3DPosition colPos = origin + Cell3DPosition(dx, dy, 0);
            if (!lattice->cellHasBlock(colPos)) continue;

            BlinkyBlocksBlock* b = (BlinkyBlocksBlock*) lattice->getBlock(colPos);
            BoxesBonusBlockCode* code = (BoxesBonusBlockCode*) b->blockCode;

            // Each box only covers identical-height columns
            if (code->hVal != baseH) {
                console << "[CHECK_H] mismatch at " << colPos
                        << " h=" << code->hVal
                        << " vs base=" << baseH << "\n";
                return false;
            }
        }
    }

    console << "[CHECK_H] Uniform height=" << baseH
            << " for region (" << origin[0] << "," << origin[1] << ")\n";
    return true;
}