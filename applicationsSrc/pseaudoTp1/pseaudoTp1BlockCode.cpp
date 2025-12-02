#include "pseaudoTp1BlockCode.hpp"
using namespace BlinkyBlocks;

PseaudoTp1BlockCode::PseaudoTp1BlockCode(BlinkyBlocksBlock *host)
    : BlinkyBlocksBlockCode(host) {

    if (not host) return;

    // Register the handler for BORDER_MSG_ID
    addMessageEventFunc2(BORDER_MSG_ID,
        std::bind(&PseaudoTp1BlockCode::handleSampleMessage, this,
                  std::placeholders::_1, std::placeholders::_2));

    module = static_cast<BlinkyBlocksBlock*>(hostBlock);

    // Initialize dirInit with dummy "max" positions
    Cell3DPosition BIG(32767, 32767, 0);
    dirInit[FRONT] = BIG;
    dirInit[RIGHT] = BIG;
    dirInit[BACK]  = BIG;
    dirInit[LEFT]  = BIG;
}

// Utility functions

Cell3DPosition PseaudoTp1BlockCode::nextPos(const Cell3DPosition& P, Direction d) const {
    switch (d) {
        case FRONT: return P.offsetY(+1);
        case RIGHT: return P.offsetX(+1);
        case BACK : return P.offsetY(-1);
        default   : return P.offsetX(-1); // LEFT
    }
}

bool PseaudoTp1BlockCode::isOnBorder(const Cell3DPosition& p) const {
    int occ = 0;
    occ += lattice->cellHasBlock(nextPos(p, FRONT)) ? 1 : 0;
    occ += lattice->cellHasBlock(nextPos(p, RIGHT)) ? 1 : 0;
    occ += lattice->cellHasBlock(nextPos(p, BACK))  ? 1 : 0;
    occ += lattice->cellHasBlock(nextPos(p, LEFT))  ? 1 : 0;
    return occ < 4; // At least one empty neighbor
}

bool PseaudoTp1BlockCode::isLess(const Cell3DPosition& a, const Cell3DPosition& b) {
    if (a[0] < b[0]) return true;
    if (a[0] == b[0] && a[1] < b[1]) return true;
    return false;
}

// only returns direction if there a neighbor
int PseaudoTp1BlockCode::getNextDir(const Cell3DPosition& P, int prevDir) {
    int k = (prevDir + 3) % 4;  // Try left
    for (int i = 0; i < 4; i++) {
        Direction d = static_cast<Direction>(k);
        Cell3DPosition Q = nextPos(P, d);

        if (lattice->cellHasBlock(Q) &&
            module->getInterfaceToNeighborPos(Q) != nullptr) {
            return k;
        }
        k = (k + 1) % 4;  // clockwise
    }
    return prevDir; // fallback
}

// Initiator selection

// bool PseaudoTp1BlockCode::isInitiator() {
//
//     return (!lattice->cellHasBlock(module->position.offsetY(-1))) &&
//            ( !lattice->cellHasBlock(module->position.offsetX(-1)) ||
//               lattice->cellHasBlock(module->position.offsetX(+1).offsetY(-1)) );
//
// }
bool PseaudoTp1BlockCode::isInitiator() {
    const Cell3DPosition p = module->position;

    bool belowEmpty     = !lattice->cellHasBlock(p.offsetY(-1));        // empty(x, y−1)
    bool leftEmpty      = !lattice->cellHasBlock(p.offsetX(-1));        // empty(x−1, y)
    bool bottomRightOcc = lattice->cellHasBlock(p.offsetX(+1).offsetY(-1)); // ¬empty(x+1, y−1)

    return belowEmpty && (leftEmpty || bottomRightOcc);
}

// Startup logic (prioritize (1,0,0)) for consistency

void PseaudoTp1BlockCode::startup() {

    if (isInitiator()) {
        setColor(colorFor(module->position));

        MyMsgData data;
        data.dir = FRONT;
        data.borders.clear();
        data.initiatorP = module->position;

        Direction next = static_cast<Direction>(getNextDir(module->position, data.dir));
        Cell3DPosition Q = nextPos(module->position, next);

        // (1,0,0) launches first if present and on border
        const Cell3DPosition anchor(1,0,0);
        bool isAnchor = (module->position == anchor) && isOnBorder(anchor);
        // Anchor fires immediately; others wait 200000 ms
        const long long delay = isAnchor ? 0 : 200000;

        if (auto *ni = module->getInterfaceToNeighborPos(Q)) {
            sendMessage("BORDER_SEARCH",
                        new MessageOf<MyMsgData>(BORDER_MSG_ID, data),
                        ni, delay, 0);
        } else {
            sendMessageToAllNeighbors("BORDER_SEARCH",
                        new MessageOf<MyMsgData>(BORDER_MSG_ID, data),
                        delay, 0, 0);
        }

        console << "[INITIATOR] Launched border trace from " << module->position
                << " delay=" << delay << "ms\n";
    }
}

// Message handler

void PseaudoTp1BlockCode::handleSampleMessage(std::shared_ptr<Message> _msg,
                                              P2PNetworkInterface* sender) {
    auto *msg = static_cast<MessageOf<MyMsgData>*>(_msg.get());
    MyMsgData data = *msg->getData();

    const Cell3DPosition here = module->position;
    int prevDir = data.dir;
    Cell3DPosition initiatorPos = data.initiatorP;

    // initiator color
    auto colorFor = [](const Cell3DPosition& p) -> Color {
        auto nice = [](int v){ return 60 + (v % 160); }; // avoid too dark colors
        int r = nice(p[0]*97  + p[1]*11);
        int g = nice(p[0]*23  + p[1]*131);
        int b = nice((p[0]+p[1])*53 + 37);
        return Color(r,g,b);
    };

    console << "*** handleSampleMessage CALLED at " << here
            << " prevDir=" << prevDir
            << " initiator=" << initiatorPos
            << " from block " << sender->getConnectedBlockId() << "\n";

    // Termination
    if (here == initiatorPos) {
        setColor(colorFor(initiatorPos));
        console << ">>> Border CLOSED at initiator " << here
                << " corners=" << data.borders.size() << "\n";
        for (auto &p : data.borders) console << "Corner: " << p << "\n";
        return;
    }

    Direction incoming = static_cast<Direction>(prevDir);

    // Forward only if unseen or initiator is smaller
    if (dirInit.find(incoming) == dirInit.end() ||
        isLess(initiatorPos, dirInit[incoming])) {

        dirInit[incoming] = initiatorPos;
        Direction next = static_cast<Direction>(getNextDir(here, incoming));

        // Corner detection
        if (next != incoming && isOnBorder(here)) {
            data.borders.push_back(here);
        }

        data.dir = next;
        Cell3DPosition Q = nextPos(here, next);
        MessageOf<MyMsgData> *ms = new MessageOf<MyMsgData>(BORDER_MSG_ID, data);

        if (auto *ni = module->getInterfaceToNeighborPos(Q)) {

            sendMessage("BORDER_SEARCH", ms, ni, 200000, 0);
            console << "[FORWARD] from " << here << " → " << Q << "\n";
        } else {
            sendMessageToAllNeighbors("BORDER_SEARCH",
                                      new MessageOf<MyMsgData>(BORDER_MSG_ID, data),
                                      200000, 0, 1, sender);
            console << "[BROADCAST] from " << here << " (no direct neighbor)\n";
        }

        // Color this module with the initiator's color
        setColor(colorFor(initiatorPos));
    } else {
        console << "[IGNORE] " << here << " already handled this direction.\n";
    }
}

// debugs

void PseaudoTp1BlockCode::onMotionEnd() {
    console << module->blockId << " reached destination.\n";
}

void PseaudoTp1BlockCode::processLocalEvent(EventPtr pev) {
    BlinkyBlocksBlockCode::processLocalEvent(pev);
}

void PseaudoTp1BlockCode::onBlockSelected() {
    cerr << "\n--- PRINT MODULE ---\n" << *module << endl;
}

void PseaudoTp1BlockCode::onAssertTriggered() {
    console << "ASSERT triggered at " << module->blockId << "\n";
}

bool PseaudoTp1BlockCode::parseUserCommandLineArgument(int &argc, char **argv[]) {
    return false;
}

string PseaudoTp1BlockCode::onInterfaceDraw() {
    stringstream trace;
    trace << "Border Detection Active";
    return trace.str();
}

static inline int clampBright(int v) { return 60 + (v % 160); } // avoid very dark colors
Color PseaudoTp1BlockCode::colorFor(const Cell3DPosition& p) {
    // simple hash from (x,y) so each initiator gets a stable color
    int r = clampBright(p[0]*97  + p[1]*11);
    int g = clampBright(p[0]*23  + p[1]*131);
    int b = clampBright((p[0]+p[1])*53 + 37);
    return Color(r,g,b);
}
