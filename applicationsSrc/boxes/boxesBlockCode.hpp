#ifndef BoxesBlockCode_H_
#define BoxesBlockCode_H_

#include "robots/blinkyBlocks/blinkyBlocksWorld.h"
#include "robots/blinkyBlocks/blinkyBlocksBlockCode.h"
static const int BOX_COLOR_MSG_ID = 3000;

static const int D_MSG_ID   = 1000;  // depth propagation
static const int DL_MSG_ID  = 1001;  // reply with d_left
static const int DLR_MSG_ID = 1002;  // request d_left
static const int W_MSG_ID   = 1003;
static const int Find_W_MSG_ID   = 1004;  // width propagation
static const int Set_W_MSG_ID   = 1005;
using namespace BlinkyBlocks;

//payloads
struct FindWPayload { int originId; int d_sent; };
struct SetWPayload  { int originId; int d_sent; int w_sent; };
struct WNotify      { int w; int dy; };
struct MyBoxMsg {
    Cell3DPosition origin;
    int wVal;
    int dVal;
    Color boxColor;
};

class BoxesBlockCode : public BlinkyBlocksBlockCode {
private:
    int d = -1;                 // current depth of this module
    int dp = -1;                // previous depth (used in propagation)
    int counter = 0;             // used to avoid duplicate requests
    int w=-1; //width for col

    BlinkyBlocksBlock *module;   // pointer to my module
    Cell3DPosition neighborsP;   // cached neighbor position (optional)
bool depthReady = false;

    bool waitingForLeft = false;


public:
    BoxesBlockCode(BlinkyBlocksBlock *host);
    ~BoxesBlockCode() {};

    void startup() override;

    // === Message Handlers ===
    void handleDMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);
    void handleDLRMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);
    void handleDLMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);
    void handleWMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);

    void handleFindW(std::shared_ptr<Message>, P2PNetworkInterface*);
    void handleSetW (std::shared_ptr<Message>, P2PNetworkInterface*);

    void createBoxForRmin(int wVal, int dVal);

    void notifyLineW(int wVal, int dy);

    void processLocalEvent(EventPtr pev) override;
    void onMotionEnd() override;
    void handleSampleMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);

    // Advanced blockcode handlers
    void parseUserElements(TiXmlDocument *config) override {}
    void parseUserBlockElements(TiXmlElement *config) override {}
    void onBlockSelected() override;
    void onAssertTriggered() override;
    void onUserKeyPressed(unsigned char c, int x, int y) override {}
    void onGlDraw() override {}
    void onTap(int face) override {}
    bool parseUserCommandLineArgument(int& argc, char **argv[]) override;
    string onInterfaceDraw() override;

    void handleBoxColorMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);

    /*****************************************************************************/
    static BlockCode *buildNewBlockCode(BuildingBlock *host) {
        return (new BoxesBlockCode((BlinkyBlocksBlock*)host));
    };
    /*****************************************************************************/
};

#endif /* BoxesBlockCode_H_ */