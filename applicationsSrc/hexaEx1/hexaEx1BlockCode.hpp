#ifndef HexaEx1BlockCode_H_
#define HexaEx1BlockCode_H_
#define NEXT_MODULE_MSG_ID 1002

#include "robots/hexanodes/hexanodesWorld.h"
#include "robots/hexanodes/hexanodesBlockCode.h"
#include <set>

constexpr int FLOODSUPPORT_MSG_ID = 1001;
static const int SAMPLE_MSG_ID = 1000;
static const int BACK_MSG_ID = 1001;
static const int REPORT_MSG_ID = 1002;
using namespace Hexanodes;

class HexaEx1BlockCode : public HexanodesBlockCode {
private:
    int distance;
    HexanodesBlock *module;
    inline static size_t nMotions = 0;
    inline static size_t neMotions = 0;
    int ii=0;
     int currentModuleId = 1;
     int motionCount = 0;
    int maxDistance{0};
     const int maxMotions = 500; // Adjust based on your needs
     bool targetReached = false;
    bool hasReachedTarget = false;

    static int totalNumberOfModules;
    int nbWaitedAnswers = 0;
    P2PNetworkInterface* parent;
    static int currentMovingModuleId;

public:

    void chooseAndMoveFarthestModule() ;

    HexanodesBlock* getFarthestModule() ;


    void initiateFarthestModuleMovement() ;

    void startFarthestModuleMovement() ;

    P2PNetworkInterface* pathInterface{nullptr};

    void convergecastReportBack() ;

    void sendMessageToNeighbor(int neighborId, const std::string& message);

    void handleNextModuleMessage(std::shared_ptr<Message> _msg,
                                               P2PNetworkInterface* sender);
    void notifyNextModule() ;

    void broadcastMessage(const string& message, Message* content) ;

    void processModuleMovements(HexanodesWorld* wrl) ;

    void triggerNextModuleMovement() ;
    void signalNextModule(int nextModuleId) ;

    void floodDistance() ;
    void Movement() ;

    int blockId;
    std::vector<int> neighbors;

    void floodMessage(const std::string& message, std::set<int>& visited);

    void initiateFlooding(const std::string& message);


    HexaEx1BlockCode(HexanodesBlock *host);
    ~HexaEx1BlockCode() {};

    void MotionToT() ;
    void scheduleNextModule() ;

    void parseTargetList(const std::string& filePath);
static std::vector<Cell3DPosition> targetPositions;

    bool isPositionFree(const Cell3DPosition& position);
    int getDistance(Cell3DPosition a, Cell3DPosition b);
    bool isPositionCloser(Cell3DPosition newPos, Cell3DPosition target, Cell3DPosition current);
    bool checkIfMotionAdvances(Cell3DPosition newPos, Cell3DPosition target);
    HexanodesMotion* getFirstClockwiseMotion(HexanodesBlock* mod);
    void startup() override;
    void moveToTargetPosition();
    void moveToTarget(HexanodesBlock* mod);
    HexanodesMotion* calculateNextStep(Cell3DPosition current, Cell3DPosition target);
    Cell3DPosition getTargetPositionForModule(int blockId);
    bool closerInLine(Cell3DPosition newPos, Cell3DPosition target, Cell3DPosition current);
    HexanodesMotion* getNextMotionTowardsTarget(HexanodesBlock* mod, Cell3DPosition target);
    void initiateMovement(HexanodesBlock* mod);
    bool positionCloserToTarget(Cell3DPosition newPos, Cell3DPosition target, Cell3DPosition current);
    void processLocalEvent(EventPtr pev) override;
    bool isCloser(Cell3DPosition newPos, Cell3DPosition target, Cell3DPosition current);
    void onMotionEnd() override;
    void handleSampleMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);
    void handleBackMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);
    void handleReportMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);
    void parseUserElements(TiXmlDocument *config) override {}
    void parseUserBlockElements(TiXmlElement *config) override {}
    void onBlockSelected() override;
    void onAssertTriggered() override;
    void onGlDraw() override {}
    void onTap(int face) override {}
    bool parseUserCommandLineArgument(int& argc, char **argv[]) override;
    string onInterfaceDraw() override;

    static BlockCode *buildNewBlockCode(BuildingBlock *host) {
        return (new HexaEx1BlockCode((HexanodesBlock*)host));
    };
};

#endif /* HexaEx1BlockCode_H_ */
