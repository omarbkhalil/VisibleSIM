#include "BoxesBlockCode.hpp"
#include <vector>
#include <utility>
using namespace BlinkyBlocks;

BoxesBlockCode::BoxesBlockCode(BlinkyBlocksBlock *host) : BlinkyBlocksBlockCode(host) {
    if (!host) return;

//Messages
    addMessageEventFunc2(D_MSG_ID, std::bind(&BoxesBlockCode::handleDMessage, this, std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(DL_MSG_ID, std::bind(&BoxesBlockCode::handleDLMessage, this, std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(DLR_MSG_ID, std::bind(&BoxesBlockCode::handleDLRMessage, this, std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(W_MSG_ID, std::bind(&BoxesBlockCode::handleWMessage, this, std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(Find_W_MSG_ID, std::bind(&BoxesBlockCode::handleFindW, this, std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(Set_W_MSG_ID, std::bind(&BoxesBlockCode::handleSetW, this, std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(BOX_COLOR_MSG_ID, std::bind(&BoxesBlockCode::handleBoxColorMessage, this, std::placeholders::_1, std::placeholders::_2));
    module = static_cast<BlinkyBlocksBlock*>(hostBlock);
}

void BoxesBlockCode::startup() {
    console << "start\n";
    //flood depth protocol
    if (!lattice->cellHasBlock(module->position.offsetY(1))) {
        dp = 1;
        auto* iface = module->getInterfaceToNeighborPos(module->position.offsetY(-1));
        if (iface && iface->isConnected()) {
            sendMessage("Depth Initialization", new MessageOf<int>(D_MSG_ID, dp), iface, 100, 200);
        }
        setColor(YELLOW);
    } else {
        dp = 0;
        hostBlock->setColor(LIGHTGREY);
    }
}
// Handle W
void BoxesBlockCode::handleWMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface* sender) {
    auto m = std::static_pointer_cast<MessageOf<WNotify>>(_msg);
    WNotify wn = *m->getData();
    w = wn.w;
    Cell3DPosition n = module->position.offsetY(wn.dy);
    if (lattice->cellHasBlock(n)) {
        sendMessage("W_NOTIFY", new MessageOf<WNotify>(W_MSG_ID, wn), module->getInterfaceToNeighborPos(n), 100, 200);
    }
}
// report W message manually in one direction
void BoxesBlockCode::notifyLineW(int wVal, int dy) {
    WNotify wn; wn.w = wVal; wn.dy = dy;
    Cell3DPosition n = module->position.offsetY(dy);
    if (lattice->cellHasBlock(n)) {
        sendMessage("W_NOTIFY", new MessageOf<WNotify>(W_MSG_ID, wn), module->getInterfaceToNeighborPos(n), 100, 200);
    }
}
// Handle D message
void BoxesBlockCode::handleDMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface* sender) {
    auto msg = std::static_pointer_cast<MessageOf<int>>(_msg);
    int drecv = *msg->getData();
    dp = drecv + 1;
    depthReady = true;
    console << "Module " << module->blockId << " received D=" << drecv << " → set dp=" << dp << "\n";
    if (waitingForLeft) {
        waitingForLeft = false;
        sendMessage("DL", new MessageOf<int>(DL_MSG_ID, dp), module->getInterfaceToNeighborPos(module->position.offsetX(1)), 100, 200);
    }
    Cell3DPosition frontPos = module->position.offsetY(-1);
    if (lattice->cellHasBlock(frontPos)) {
        sendMessage("SET_D_MSG", new MessageOf<int>(D_MSG_ID, dp), module->getInterfaceToNeighborPos(frontPos), 100, 200);
        return;
    }
    Cell3DPosition leftPos = module->position.offsetX(-1);
    if (!lattice->cellHasBlock(leftPos)) {
        if (module->getInterfaceToNeighborPos(module->position.offsetX(1))->isConnected() &&
            !module->getInterfaceToNeighborPos(module->position.offsetX(-1))->isConnected()) {
            console << "Module " << module->blockId << " is Rmin!\n";
            setColor(CYAN);
            FindWPayload fw; fw.originId = module->blockId; fw.d_sent = dp;
            sendMessage("FIND_W", new MessageOf<FindWPayload>(Find_W_MSG_ID, fw), module->getInterfaceToNeighborPos(module->position.offsetX(1)), 100, 200);
        }
    } else {
        sendMessage("DL_REQ", new MessageOf<int>(DLR_MSG_ID, dp), module->getInterfaceToNeighborPos(leftPos), 100, 200);
    }
}
// Handle depth request
void BoxesBlockCode::handleDLRMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface* sender) {
    auto msg = std::static_pointer_cast<MessageOf<int>>(_msg);
    if (depthReady) {
        sendMessage("DL_REP", new MessageOf<int>(DL_MSG_ID, dp), sender, 100, 200);
    } else {
        waitingForLeft = true;
    }
}
// Handle DL reply
void BoxesBlockCode::handleDLMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface* sender) {
    auto msg = std::static_pointer_cast<MessageOf<int>>(_msg);
    int d_left = *msg->getData();
    Cell3DPosition frontPos = module->position.offsetY(-1);
    bool isFrontEmpty = !lattice->cellHasBlock(frontPos);
    Cell3DPosition leftPos = module->position.offsetX(-1);
    Cell3DPosition leftFrontPos = leftPos.offsetY(-1);
    bool leftFrontEmpty = !lattice->cellHasBlock(leftFrontPos);
    if (isFrontEmpty && (dp != d_left || !leftFrontEmpty)) {
        setColor(RED);
        Cell3DPosition rightPos = module->position.offsetX(+1);
        if (lattice->cellHasBlock(rightPos)) {
            FindWPayload fw; fw.originId = module->blockId; fw.d_sent = dp;
            sendMessage("FIND_W", new MessageOf<FindWPayload>(Find_W_MSG_ID, fw), module->getInterfaceToNeighborPos(rightPos), 100, 200);
        } else {
            w = 1;
            module->setColor(RED);
            createBoxForRmin(w, dp);
            notifyLineW(w, -1);
            return;
        }
    } else setColor(BLACK);
}
// Handle FIND_W message
void BoxesBlockCode::handleFindW(std::shared_ptr<Message> _msg, P2PNetworkInterface* sender) {
    auto m = std::static_pointer_cast<MessageOf<FindWPayload>>(_msg);
    FindWPayload p = *m->getData();
    if (dp < p.d_sent) {
        Cell3DPosition leftX = module->position.offsetX(-1);
        if (lattice->cellHasBlock(leftX)) {
            SetWPayload sw; sw.originId = p.originId; sw.d_sent = p.d_sent; sw.w_sent = 0;
            sendMessage("SET_W", new MessageOf<SetWPayload>(Set_W_MSG_ID, sw), module->getInterfaceToNeighborPos(leftX), 100, 200);
        }
        return;
    }
    Cell3DPosition rightX = module->position.offsetX(+1);
    if (!lattice->cellHasBlock(rightX)) {
        int wLocal = 1;
        notifyLineW(wLocal, +1);
        Cell3DPosition leftX = module->position.offsetX(-1);
        if (lattice->cellHasBlock(leftX)) {
            SetWPayload sw; sw.originId = p.originId; sw.d_sent = p.d_sent; sw.w_sent = wLocal;
            sendMessage("SET_W", new MessageOf<SetWPayload>(Set_W_MSG_ID, sw), module->getInterfaceToNeighborPos(leftX), 100, 200);
        }
        return;
    }
    sendMessage("FIND_W", new MessageOf<FindWPayload>(Find_W_MSG_ID, p), module->getInterfaceToNeighborPos(rightX), 100, 200);
}
// Handle SET_W message
void BoxesBlockCode::handleSetW(std::shared_ptr<Message> _msg, P2PNetworkInterface* sender) {
    auto m = std::static_pointer_cast<MessageOf<SetWPayload>>(_msg);
    SetWPayload s = *m->getData();
    if (dp >= s.d_sent) {
        w = s.w_sent + 1;
        notifyLineW(w, +1);
        if (module->blockId == s.originId) {
            module->setColor(RED);
            createBoxForRmin(w, dp);
            return;
        }
        Cell3DPosition leftX = module->position.offsetX(-1);
        if (lattice->cellHasBlock(leftX)) {
            SetWPayload next; next.originId = s.originId; next.d_sent = s.d_sent; next.w_sent = w;
            sendMessage("SET_W", new MessageOf<SetWPayload>(Set_W_MSG_ID, next), module->getInterfaceToNeighborPos(leftX), 100, 200);
        }
    }
}
// Create and propagate box coloring
void BoxesBlockCode::createBoxForRmin(int wVal, int dVal) {
    Cell3DPosition pos = module->position;
    srand(module->blockId * 97);
    Color boxColor(50 + rand() % 205, 50 + rand() % 205, 50 + rand() % 205);
    module->setColor(boxColor);

    MyBoxMsg msg;
    msg.origin = pos;
    msg.wVal = wVal;
    msg.dVal = dVal;
    msg.boxColor = boxColor;

    sendMessageToAllNeighbors(
        new MessageOf<MyBoxMsg>(BOX_COLOR_MSG_ID, msg),
        0, 0, 0
    );
}

void BoxesBlockCode::onMotionEnd() { console << " has reached its destination\n"; }




//debugs and prints
void BoxesBlockCode::processLocalEvent(EventPtr pev) {
    BlinkyBlocksBlockCode::processLocalEvent(pev);
    switch (pev->eventType) { case EVENT_ADD_NEIGHBOR: break; case EVENT_REMOVE_NEIGHBOR: break; }
}

void BoxesBlockCode::onBlockSelected() {
    cerr << endl << "--- PRINT MODULE " << *module << "---" << endl;
    console << "dp=" << dp << "\n";
    console << "w=" << w << "\n";
}

void BoxesBlockCode::onAssertTriggered() { console << " has triggered an assert\n"; }

bool BoxesBlockCode::parseUserCommandLineArgument(int &argc, char **argv[]) {
    if ((argc > 0) && ((*argv)[0][0] == '-')) {
        switch((*argv)[0][1]) {
            case 'b': cout << "-b option provided" << endl; return true;
            case '-': {
                string varg = string((*argv)[0] + 2);
                if (varg == string("foo")) {
                    int fooArg = stoi((*argv)[1]); argc--; (*argv)++;
                    cout << "--foo option provided with value: " << fooArg << endl;
                } else return false;
                return true;
            }
            default: cerr << "Unrecognized argument: " << (*argv)[0] << endl;
        }
    }
    return false;
}

string BoxesBlockCode::onInterfaceDraw() {
    stringstream trace; trace << "Some value " << 123; return trace.str();
}

void BoxesBlockCode::handleBoxColorMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface* sender) {
    auto* msg = static_cast<MessageOf<MyBoxMsg>*>(_msg.get());
    MyBoxMsg data = *msg->getData();
    Cell3DPosition here = module->position;
    Cell3DPosition rel = here - data.origin;
    if (rel[0] >= 0 && rel[0] < data.wVal && rel[1] >= 0 && rel[1] < data.dVal && rel[2] == 0) {
        module->setColor(data.boxColor);
        Cell3DPosition candidates[2] = { here.offsetX(+1), here.offsetY(+1) };
        for (const auto& nbPos : candidates) {
            if (!lattice->cellHasBlock(nbPos)) continue;
            Cell3DPosition nrel = nbPos - data.origin;
            bool inside = (nrel[0] >= 0 && nrel[0] < data.wVal) && (nrel[1] >= 0 && nrel[1] < data.dVal) && (nrel[2] == 0);
            if (!inside) continue;
            auto* nbIface = module->getInterfaceToNeighborPos(nbPos);
            if (!nbIface || nbIface == sender) continue;
            sendMessage("BOX_COLOR", new MessageOf<MyBoxMsg>(BOX_COLOR_MSG_ID, data), nbIface, 0, 0);
        }
    }
}
