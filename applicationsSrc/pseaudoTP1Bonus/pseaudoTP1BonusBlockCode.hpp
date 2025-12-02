#ifndef PseaudoTP1BonusBLOCKCODE_H_
#define PseaudoTP1BonusBLOCKCODE_H_

#include "robots/blinkyBlocks/blinkyBlocksWorld.h"
#include "robots/blinkyBlocks/blinkyBlocksBlockCode.h"
#include <unordered_map>
#include <vector>
#include <array>
#include <sstream>

using namespace BlinkyBlocks;

// Message type constant
static const int BORDER_MSG_ID = 2001;

class PseaudoTP1BonusBlockCode : public BlinkyBlocksBlockCode {
private:
    BlinkyBlocksBlock *module = nullptr;
    int distance = -1;

    // Data structures for border tracing
    std::unordered_map<int, Cell3DPosition> dirInit;   // direction → initiator position
    std::vector<Cell3DPosition> borderCorners;         // detected corners
    std::vector<Cell3DPosition> initiatorPositions;    // list of initiators (debug/trace)

    // Message payload
    struct MyMsgData {
        int dir;                        // previous direction
        std::vector<Cell3DPosition> borders;  // corner positions so far
        Cell3DPosition initiatorP;      // starting initiator position
    };

public:
    //  directions
    enum Direction {
        FRONT = 0,
        RIGHT = 1,
        BACK  = 2,
        LEFT  = 3
    };

    Color colorFor(const Cell3DPosition& p);

    // --- Constructors / destructors ---
    PseaudoTP1BonusBlockCode(BlinkyBlocksBlock *host);
    ~PseaudoTP1BonusBlockCode() override = default;

    // --- Core functions ---
    void startup() override;
    void handleSampleMessage(std::shared_ptr<Message> _msg,
                             P2PNetworkInterface *sender);
    void processLocalEvent(EventPtr pev) override;
    void onMotionEnd() override;

    // --- Algorithmic helpers ---
    bool isInitiator();

    void broadcastBorder(const MyMsgData &payload, long long delay, P2PNetworkInterface *except);

    bool isOnBorder(const Cell3DPosition &p) const;
    static bool isLess(const Cell3DPosition &a, const Cell3DPosition &b);
    int getNextDir(const Cell3DPosition &P, int prevDir);
    Cell3DPosition nextPos(const Cell3DPosition &P, Direction d) const;

    // --- Debug hooks / UI ---
    void onBlockSelected() override;
    void onAssertTriggered() override;
    bool parseUserCommandLineArgument(int &argc, char **argv[]) override;
    std::string onInterfaceDraw() override;

    // --- Parsing / optional handlers ---
    void parseUserElements(TiXmlDocument *config) override {}
    void parseUserBlockElements(TiXmlElement *config) override {}
    void onUserKeyPressed(unsigned char, int, int) override {}
    void onGlDraw() override {}
    void onTap(int) override {}

    /**************************************************************************/
    static BlockCode *buildNewBlockCode(BuildingBlock *host) {
        return new PseaudoTP1BonusBlockCode((BlinkyBlocksBlock *)host);
    }
    /**************************************************************************/
};

#endif /* PseaudoTP1BonusBLOCKCODE_H_ */
