#include "PseaudoTP1BonusBlockCode.hpp"
using namespace BlinkyBlocks;

static inline int clampBright(int v) { return 60 + (v % 160); } // avoid very dark colors

PseaudoTP1BonusBlockCode::PseaudoTP1BonusBlockCode(BlinkyBlocksBlock *host)
    : BlinkyBlocksBlockCode(host) {

    if (!host) return;

    addMessageEventFunc2(BORDER_MSG_ID,
        std::bind(&PseaudoTP1BonusBlockCode::handleSampleMessage, this,
                  std::placeholders::_1, std::placeholders::_2));

    module = static_cast<BlinkyBlocksBlock*>(hostBlock);

    // map with "max"
    Cell3DPosition BIG(32767, 32767, 0);
    dirInit[FRONT] = BIG;
    dirInit[RIGHT] = BIG;
    dirInit[BACK]  = BIG;
    dirInit[LEFT]  = BIG;
}

// -------------------- utils --------------------

Cell3DPosition PseaudoTP1BonusBlockCode::nextPos(const Cell3DPosition& P, Direction d) const {
    switch (d) {
        case FRONT: return P.offsetY(+1);
        case RIGHT: return P.offsetX(+1);
        case BACK : return P.offsetY(-1);
        default   : return P.offsetX(-1); // LEFT
    }
}

bool PseaudoTP1BonusBlockCode::isOnBorder(const Cell3DPosition& p) const {
    int occ = 0;
    occ += lattice->cellHasBlock(nextPos(p, FRONT)) ? 1 : 0;
    occ += lattice->cellHasBlock(nextPos(p, RIGHT)) ? 1 : 0;
    occ += lattice->cellHasBlock(nextPos(p, BACK))  ? 1 : 0;
    occ += lattice->cellHasBlock(nextPos(p, LEFT))  ? 1 : 0;
    return occ < 4;
}

bool PseaudoTP1BonusBlockCode::isLess(const Cell3DPosition& a, const Cell3DPosition& b) {
    if (a[0] < b[0]) return true;
    if (a[0] == b[0] && a[1] < b[1]) return true;
    return false;
}

// only returns a dir that has a connected neighbor
int PseaudoTP1BonusBlockCode::getNextDir(const Cell3DPosition& P, int prevDir) {
    int k = (prevDir + 3) % 4;  // left-hand rule first
    for (int i = 0; i < 4; i++) {
        Direction d = static_cast<Direction>(k);
        Cell3DPosition Q = nextPos(P, d);
        if (lattice->cellHasBlock(Q) &&
            module->getInterfaceToNeighborPos(Q) != nullptr) {
            return k;
        }
        k = (k + 1) % 4;
    }
    return prevDir; // fallback
}

Color PseaudoTP1BonusBlockCode::colorFor(const Cell3DPosition& p) {
    int r = clampBright(p[0]*97  + p[1]*11);
    int g = clampBright(p[0]*23  + p[1]*131);
    int b = clampBright((p[0]+p[1])*53 + 37);
    return Color(r,g,b);
}

// -------------------- initiator test --------------------

bool PseaudoTP1BonusBlockCode::isInitiator() {
    const Cell3DPosition p = module->position;
    bool belowEmpty     = !lattice->cellHasBlock(p.offsetY(-1));             // empty(x,y-1)
    bool leftEmpty      = !lattice->cellHasBlock(p.offsetX(-1));             // empty(x-1,y)
    bool bottomRightOcc =  lattice->cellHasBlock(p.offsetX(+1).offsetY(-1)); // ¬empty(x+1,y-1)
    return belowEmpty && (leftEmpty || bottomRightOcc);
}

// -------------------- broadcast helper --------------------

void PseaudoTP1BonusBlockCode::broadcastBorder(const MyMsgData& payload,
                                               long long delay,
                                               P2PNetworkInterface* except) {
    for (int i = 0; i < module->getNbInterfaces(); ++i) {
        auto* itf = module->getInterface(i);
        if (!itf || !itf->isConnected() || itf == except) continue;
        auto* m = new MessageOf<MyMsgData>(BORDER_MSG_ID, payload);
        sendMessage("BORDER_SEARCH", m, itf, delay, 0);
    }
}

// -------------------- startup --------------------

void PseaudoTP1BonusBlockCode::startup() {
    if (!module) return;

    if (isInitiator()) {
        // color only corners

        MyMsgData data;
        data.dir = FRONT;
        data.borders.clear();
        data.initiatorP = module->position;

        Direction next = static_cast<Direction>(getNextDir(module->position, data.dir));
        Cell3DPosition Q = nextPos(module->position, next);

        // (1,0,0) priority if on border
        // For consistency matter only
        const Cell3DPosition anchor(1,0,0);
        bool isAnchor = (module->position == anchor) && isOnBorder(anchor);
        const long long delay = isAnchor ? 0 : 200000;

        if (auto *ni = module->getInterfaceToNeighborPos(Q)) {
            sendMessage("BORDER_SEARCH",
                        new MessageOf<MyMsgData>(BORDER_MSG_ID, data),
                        ni, delay, 0);
        } else {
            broadcastBorder(data, delay, nullptr);
        }

        console << "[INITIATOR] launch from " << module->position
                << " delay=" << delay << "ms\n";
    }
}

// -------------------- handler --------------------

void PseaudoTP1BonusBlockCode::handleSampleMessage(std::shared_ptr<Message> _msg,
                                                   P2PNetworkInterface* sender) {
    auto *msg = static_cast<MessageOf<MyMsgData>*>(_msg.get());
    MyMsgData data = *msg->getData();

    const Cell3DPosition here = module->position;
    int prevDir = data.dir;
    Cell3DPosition initiatorPos = data.initiatorP;

    console << "*** handle at " << here
            << " prevDir=" << prevDir
            << " init=" << initiatorPos
            << " from " << sender->getConnectedBlockId() << "\n";

    // close loop: color the corner
    if (here == initiatorPos) {
        setColor(colorFor(initiatorPos));
        console << ">>> CLOSED at initiator " << here
                << " corners=" << data.borders.size() << "\n";
        for (auto &p : data.borders) console << "Corner: " << p << "\n";
        return;
    }

    Direction incoming = static_cast<Direction>(prevDir);

    // forward only if unseen or smaller initiator
    if (dirInit.find(incoming) == dirInit.end() ||
        isLess(initiatorPos, dirInit[incoming])) {

        dirInit[incoming] = initiatorPos;
        Direction next = static_cast<Direction>(getNextDir(here, incoming));

        //  CORNER ONLY COLORING
        bool isCorner = (next != incoming) && isOnBorder(here);
        if (isCorner) {
            data.borders.push_back(here);
            setColor(colorFor(initiatorPos));   // color only at turns
        }

        data.dir = next;
        Cell3DPosition Q = nextPos(here, next);

        if (auto *ni = module->getInterfaceToNeighborPos(Q)) {
            sendMessage("BORDER_SEARCH",
                        new MessageOf<MyMsgData>(BORDER_MSG_ID, data),
                        ni, 200000, 0);
            console << "[FORWARD] " << here << " → " << Q << "\n";
        } else {
            broadcastBorder(data, 200000, /*except=*/sender);
            console << "[BROADCAST] from " << here << " (no direct neighbor)\n";
        }


    } else {
        console << "[IGNORE] " << here << " already handled for this dir.\n";
    }
}

// --------------------  hooks --------------------

void PseaudoTP1BonusBlockCode::onMotionEnd() {
    console << module->blockId << " reached destination.\n";
}

void PseaudoTP1BonusBlockCode::processLocalEvent(EventPtr pev) {
    BlinkyBlocksBlockCode::processLocalEvent(pev);
}

void PseaudoTP1BonusBlockCode::onBlockSelected() {
    cerr << "\n--- PRINT MODULE ---\n" << *module << endl;
}

void PseaudoTP1BonusBlockCode::onAssertTriggered() {
    console << "ASSERT triggered at " << module->blockId << "\n";
}

bool PseaudoTP1BonusBlockCode::parseUserCommandLineArgument(int &argc, char **argv[]) {
    return false;
}

std::string PseaudoTP1BonusBlockCode::onInterfaceDraw() {
    std::stringstream s; s << "Border Detection Active";
    return s.str();
}
