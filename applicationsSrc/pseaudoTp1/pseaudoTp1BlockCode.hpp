#ifndef PSEAUDOTP1BLOCKCODE_H_
#define PSEAUDOTP1BLOCKCODE_H_

#include "robots/blinkyBlocks/blinkyBlocksWorld.h"
#include "robots/blinkyBlocks/blinkyBlocksBlockCode.h"
#include <unordered_map>
#include <vector>
#include <array>
#include <sstream>

using namespace BlinkyBlocks;

// Message type
static const int BORDER_MSG_ID = 2001;

class PseaudoTp1BlockCode : public BlinkyBlocksBlockCode {
private:
    BlinkyBlocksBlock *module = nullptr;
    int distance = -1;

    // Data structures for border tracing
    std::unordered_map<int, Cell3DPosition> dirInit;   // direction → initiator position
    std::vector<Cell3DPosition> borderCorners;         // detected corners
    std::vector<Cell3DPosition> initiatorPositions;    // list of initiators

    // Message payload
    struct MyMsgData {
        int dir;                        // previous direction
        std::vector<Cell3DPosition> borders;  // corner positions so far
        Cell3DPosition initiatorP;      // starting initiator position
    };

public:
    // Cardinal directions
    enum Direction {
        FRONT = 0,
        RIGHT = 1,
        BACK  = 2,
        LEFT  = 3
    };
    static Direction oppositeDir(Direction d) ;

    Color colorFor(const Cell3DPosition& p);

    // --- Constructors / destructors ---
    PseaudoTp1BlockCode(BlinkyBlocksBlock *host);
    ~PseaudoTp1BlockCode() override = default;

    // --- Core functions ---
    void startup() override;
    void handleSampleMessage(std::shared_ptr<Message> _msg,
                             P2PNetworkInterface *sender);
    void processLocalEvent(EventPtr pev) override;
    void onMotionEnd() override;

    // --- Algorithmic helpers ---
    bool isInitiator();
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
        return new PseaudoTp1BlockCode((BlinkyBlocksBlock *)host);
    }
    /**************************************************************************/
};

#endif /* PSEAUDOTP1BLOCKCODE_H_ */
