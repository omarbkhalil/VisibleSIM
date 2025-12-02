#ifndef CatomMAPF2CONF2BlockCode_H_
#define CatomMAPF2CONF2BlockCode_H_

#include "robots/catoms3D/catoms3DWorld.h"
#include "robots/catoms3D/catoms3DBlockCode.h"
#include <vector>

constexpr int GO_ID    = 9001;
constexpr int BACK_ID  = 9002;
constexpr int FIRST_ID = 9003;
constexpr int REACHED_ID = 9004;

using namespace Catoms3D;

class CatomMAPF2CONF2BlockCode : public Catoms3DBlockCode {
private:

    int distance;
    Catoms3DBlock *module;
public :
    static Cell3DPosition Origin;
    static Cell3DPosition OriginB;

    std::vector<Cell3DPosition> currentPath;
    size_t pathIdx = 0;
    bool hasPath = false;

    // Sequential movement tracking
    static size_t currentFBIndex;
    static bool isMovementInProgress;
    static std::array<Cell3DPosition, 12> FB_POSITIONS;

    // Farthest-selection state
    int stage = 0;
    int myDist = INT_MAX;
    int bestDist = -1;
    bID bestId = INT_MAX;
    int nbWaitedAns = 0;
    P2PNetworkInterface* parent = nullptr;
    P2PNetworkInterface* winnerChild = nullptr;
    bool farthestWaveStarted = false;

    // helper
    void initFarthest() {
        myDist = INT_MAX;
        bestDist = -1;
        bestId = INT_MAX;
        nbWaitedAns = 0;
        parent = nullptr;
        winnerChild = nullptr;
    }


    int seenFirstWave = -1;
    int seenReachedWave = -1;
    bool allReached = false;
    //bool gReverseActive = false;


    int nextPairIdx = 0;
    // // Payload for GO message with path
    // struct GoPayload {
    //     std::vector<Cell3DPosition> path;
    // };

    struct CBackPayload {
        int kind;   // 0=BACK_UP, 1=SELECT_DOWN
        int dist;   // hop distance from leader
        bID id;     // candidate id (blockId)

    };
    struct CFirstPayload {
        Cell3DPosition target;
        Cell3DPosition goal;
        Cell3DPosition origin; // <— who’s orchestrating (initiator or initiator2)
        int wave;
    };

    struct CReachedPayload {
        Cell3DPosition origin;  // who should consume this
        Cell3DPosition from;    // the module that reached its goal
        int wave;               // sequence number for dedup
        bool isFinal;           // true only for the FINAL
    };

    int nextFBIdx = 0;
    int nextBFIdx = 0;

    Cell3DPosition goalForThisMove;
    bool hasGoal = false;

    int lastFBWaveSeen = -1;
    int lastBFWaveSeen = -1;
    Cell3DPosition myMoveOrigin = Cell3DPosition(-999,-999,-999);

    bool reversePhase = false;
int reverseIndex = -1;
    int seenFinalWave = -1;

    static Cell3DPosition initiator;

    static Cell3DPosition initiator2;

    static Cell3DPosition initiator3;

    static Cell3DPosition initiator4;

    CatomMAPF2CONF2BlockCode(Catoms3DBlock *host);
    ~CatomMAPF2CONF2BlockCode() {};

//    std::array<Cell3DPosition, 12> getTargetFBPlus16X() const;


    static inline std::array<Cell3DPosition, 12>
    shiftPlus16X(const std::array<Cell3DPosition, 12>& src) {
        std::array<Cell3DPosition, 12> out = src;
        const Cell3DPosition shift(16, 0, 0);
        for (auto &p : out) p = p + shift;
        return out;
    }

    template<size_t N>
   static std::array<Cell3DPosition, N>
   shiftPlusX(const std::array<Cell3DPosition, N>& src, int dx) {
        auto out = src;
        for (auto &p : out) {
            p = p + Cell3DPosition(dx, 0, 0);  // no direct .x access
        }
        return out;
    }


    void startup() override;

    void broadcastFirstTo(const Cell3DPosition &target, const Cell3DPosition &goal);

    void handleFirst(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);

    void handleReached(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);

    void handleGo(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);
    void handleBack(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);
    void processLocalEvent(EventPtr pev) override;
    std::vector<Cell3DPosition> loadOptimalPathFromFile(const std::string &phaseLabel);
    void onMotionEnd() override;
    void saveOptimalPath(const std::vector<Cell3DPosition> &path, const std::string &phaseLabel);

    // New helper methods
    void startNextFBModule();
    bool sendMessageToModule(const Cell3DPosition &targetPos, const std::vector<Cell3DPosition> &path);
    bool routeMessageToPosition(const Cell3DPosition &targetPos, const std::vector<Cell3DPosition> &path);

    /// Advanced blockcode handlers below
    void parseUserElements(TiXmlDocument *config) override {}
    void parseUserBlockElements(TiXmlElement *config) override {}
    void onBlockSelected() override;
    void onAssertTriggered() override;
    void onUserKeyPressed(unsigned char c, int x, int y) override {}
    void onGlDraw() override {}
    void onTap(int face) override {}
    bool parseUserCommandLineArgument(int& argc, char **argv[]) override;
    string onInterfaceDraw() override;

    bool scheduleGuardedHop();

    std::vector<Cell3DPosition> findPath(Cell3DPosition &start,
                                         Cell3DPosition &goal);
    bool getAllPossibleMotionsFromPosition(Cell3DPosition position, vector<Cell3DPosition> &reachablePositions);
    void findDestinations();
    void startFarthestWaveFromHere();

    static BlockCode *buildNewBlockCode(BuildingBlock *host) {
        return (new CatomMAPF2CONF2BlockCode((Catoms3DBlock*)host));
    };
};

#endif /* CatomMAPF2CONF2BlockCode_H_ */
