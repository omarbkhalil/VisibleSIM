#ifndef LeaderElecLab2BlockCode_H_
#define LeaderElecLab2BlockCode_H_

#include "robots/blinkyBlocks/blinkyBlocksWorld.h"
#include "robots/blinkyBlocks/blinkyBlocksBlockCode.h"
#include "robots/blinkyBlocks/blinkyBlocksSimulator.h"

static const int TYPE_1_EXPLORE = 2001;
static const int TYPE_2_CONFIRM_CHILD = 2002;
static const int TYPE_3_REJECT_CHILD = 2003;
static const int TYPE_4_REPORT_SIZE = 2004;
static const int TYPE_5_ASSIGN_ID = 2005;
static const int TYPE_6_BROADCAST_TOTAL = 2006;  // totals from root to all nodes
static const int TYPE_7_TREE_INFO = 2007;
static const int TYPE_8_WINNER_CAND = 2008;
static const int TYPE_10_GRID_COLOR = 2010;

constexpr int TYPE_11_WINNER_UP = 1011;

using namespace BlinkyBlocks;

class LeaderElecLab2BlockCode : public BlinkyBlocksBlockCode {

    static constexpr int NumbOfInterfaces = 6;   // ← change to 4 if you only use 4


    private:
    int distance;
    BlinkyBlocksBlock *module;

public :

    struct WinnerUp {
    int leaderTag;   // tag of the winning tree
    int seq;         // paint sequence to dedupe
    int fromId;      // sender for debug
};
    struct WinnerCand {
    int maxOtherScore;  // max score seen from neighbors in other trees
};
    // Dedup & book-keeping for the upward notify
    int lastWinnerUpSeqSent  = -1; // last seq I forwarded upward
    int winnerTagKnown       = -1; // latest leaderTag I've seen
    int winnerSeqKnown       = -1; // latest seq I've seen for that tag

    struct GridPaint {
        int leaderTag;     // rootTag of winning tree
        int colorIdx;      // color index used for painting
        int seq;           // sequence number
        int leaderRootId;  // NEW: moduleId of the global leader root
    };

    // --- State to elect winner ---
    int  local_max_other_score  = -1;  // from physical neighbors in other trees
    int  child_max_other_score  = -1;  // aggregated from children
    bool isGlobalLeader         = false;

    int  paintSeenLeader = -1, paintSeenSeq = -1, gridPaintSeq = 0;

    int  globalLeaderTag    = -1;  // winning tree's rootTag
    int  globalLeaderRootId = -1;  // moduleId of the global leader root
    bool globalLeaderKnown  = false;

    // Optional helper: am I in the winning tree?
    bool isInWinningTree    = false;



    int global_total_size   = -1;
    int global_total_weight = -1;
    int global_total_score  = -1;

int subtree_weight;
    struct TreeInfo {
        int rootTag;
        int degree;       // parent?1:0 + children count
        int isLeaf;       // 1 if leaf, 0 otherwise
        int totalSize;    // NEW: global total size of the sender's tree
        int totalWeight;  // NEW: global total weight of the sender's tree
        int totalScore;   // NEW: convenience = totalSize + totalWeight
    };

    struct SizeWeight {
    int size;    // subtree size
    int weight;  // subtree weight
};

    bool allChildrenReported() const;
    bool isLeafNow() const;

    void sendTreeInfoToAllNeighbors();

    void onTreeInfo(std::shared_ptr<Message> msg, P2PNetworkInterface *sender);

    void maybeSendWinnerCandUp();

    void onWinnerCand(std::shared_ptr<Message> msg, P2PNetworkInterface *sender);

    void tryCrownIfRoot();

    void startGridPaint(int leaderTag, int colorIdx);

    void onGridPaint(std::shared_ptr<Message> msg, P2PNetworkInterface *sender);

    int myRootTag() const;

    void maybeSendWinnerUp(int leaderTag, int seq);

    void onWinnerUp(std::shared_ptr<Message> msg, P2PNetworkInterface *sender);

    // ---- Forest / tree state ----
bool Lroot = false;                 // I am a root in the forest?
    int  rootColorIdx = 0;              // my subtree palette seed

    bool isDiscovered = false;
    bool subtreeSizeSent = false;
    int  receivedSizeReports = 0;
    int  uniqueID = -1;
    int  subtreeSize = 1;

    P2PNetworkInterface* parent = nullptr;
    std::vector<P2PNetworkInterface*> childrenModules;
    std::vector<int> childrenSizes;
    std::vector<P2PNetworkInterface*> neighbours;
    int interfaceStatus[NumbOfInterfaces];
    // (Optional) just for logging the connectivity bitmask
    int weight = 0;


bool subtree_size_sent = false;

bool isLeader = false;

    // int unique_id = -1;
    std::vector<P2PNetworkInterface*> children;
 int unique_id = -1;
 int subtree_size = 1;


    LeaderElecLab2BlockCode(BlinkyBlocksBlock *host);
    ~LeaderElecLab2BlockCode() {};

    /**
     * This function is called on startup of the blockCode, it can be used to perform initial
     *  configuration of the host or this instance of the program.
     * @note this can be thought of as the main function of the module
     **/
    void startup() override;



    /**
     * @brief Handler for all events received by the host block
     * @param pev pointer to the received event
     */
    void processLocalEvent(EventPtr pev) override;

    /**
     * @brief Callback function executed whenever the module finishes a motion
     */
    void onMotionEnd() override;

    /**
     * @brief Sample message handler for this instance of the blockcode
     * @param _msg Pointer to the message received by the module, requires casting
     * @param sender Connector of the module that has received the message and that is connected to the sender */
    void handleSampleMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);

    /// Advanced blockcode handlers below

    /**
     * @brief Provides the user with a pointer to the configuration file parser, which can be used to read additional user information from it.
     * @param config : pointer to the TiXmlDocument representing the configuration file, all information related to VisibleSim's core have already been parsed
     *
     * Called from BuildingBlock constructor, only once.
     */
    void parseUserElements(TiXmlDocument *config) override {}

    /**
     * @brief Provides the user with a pointer to the configuration file parser, which can be used to read additional user information from each block config. Has to be overriden in the child class.
     * @param config : pointer to the TiXmlElement representing the block configuration file, all information related to concerned block have already been parsed
     *
     */
    void parseUserBlockElements(TiXmlElement *config) override;

    void exploreNeighbors(std::shared_ptr<Message> msg, P2PNetworkInterface *sender);

    void confirmChild(std::shared_ptr<Message> msg, P2PNetworkInterface *sender);

    void rejectChild(std::shared_ptr<Message> msg, P2PNetworkInterface *sender);

    void reportSize(std::shared_ptr<Message> msg, P2PNetworkInterface *sender);

    void assignId(std::shared_ptr<Message> msg, P2PNetworkInterface *sender);

    void broadcastTotalsToChildren(const SizeWeight &tot);

    void onBroadcastTotals(std::shared_ptr<Message> msg, P2PNetworkInterface *sender);

    void Check();


    /**
     * User-implemented debug function that gets called when a module is selected in the GUI
     */
    void onBlockSelected() override;

    /**
     * User-implemented debug function that gets called when a VS_ASSERT is triggered
     * @note call is made from utils::assert_handler()
     */
    void onAssertTriggered() override;

    /**
     * User-implemented keyboard handler function that gets called when
     *  a key press event could not be caught by openglViewer
     * @param c key that was pressed (see openglViewer.cpp)
     * @param x location of the pointer on the x axis
     * @param y location of the pointer on the y axis
     * @note call is made from GlutContext::keyboardFunc (openglViewer.h)
     */
    void onUserKeyPressed(unsigned char c, int x, int y) override {}

    /**
     * Call by world during GL drawing phase, can be used by a user
     *  to draw custom Gl content into the simulated world
     * @note call is made from World::GlDraw
     */
    void onGlDraw() override {}

    /**
     * @brief This function is called when a module is tapped by the user. Prints a message to the console by default.
     Can be overloaded in the user blockCode
     * @param face face that has been tapped */
    void onTap(int face) override {}


    /**
     * User-implemented keyboard handler function that gets called when
     *  a key press event could not be caught by openglViewer
     * @note call is made from GlutContext::keyboardFunc (openglViewer.h)
     */
    bool parseUserCommandLineArgument(int& argc, char **argv[]) override;

    /**
     * Called by openglviewer during interface drawing phase, can be used by a user
     *  to draw a custom Gl string onto the bottom-left corner of the GUI
     * @note call is made from OpenGlViewer::drawFunc
     * @return a string (can be multi-line with `\n`) to display on the GUI
     */
    string onInterfaceDraw() override;

/*****************************************************************************/
/** needed to associate code to module                                      **/
    static BlockCode *buildNewBlockCode(BuildingBlock *host) {
        return (new LeaderElecLab2BlockCode((BlinkyBlocksBlock*)host));
    };
/*****************************************************************************/
};

#endif /* LeaderElecLab2BlockCode_H_ */