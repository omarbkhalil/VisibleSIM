#ifndef CatomsTest2BlockCode_H_
#define CatomsTest2BlockCode_H_

#include "events/events.h"
#include "robots/catoms3D/catoms3DWorld.h"
#include "robots/catoms3D/catoms3DBlockCode.h"
#include <unordered_set>
#include <vector>
#include <unordered_map>
#include <string>

static const int SAMPLE_MSG_ID = 1000;
static const int BACK_MSG_ID = 1001;
static const int REPORT_MSG_ID = 1002;
static const int REPORTbck_MSG_ID = 1003;

using namespace Catoms3D;

class CatomsTest2BlockCode : public Catoms3DBlockCode {
private:
    int distance;
    Catoms3DBlock *module;

    struct NodeRecord {
        double gScore;
        double fScore;
        int cameFrom; // store parent ID in path
    };

    // Declare A* related functions only once
    std::vector<int> runAStar(int startId, int goalId);
    std::vector<int> reconstructPath(std::unordered_map<int, NodeRecord> &records, int current);
    double heuristic(int currentId, int goalId);
    static map<Cell3DPosition, vector<Cell3DPosition>> cells;
    static vector<Cell3DPosition> visited;
    static vector<Cell3DPosition> teleportedPositions;
    int currentModuleId = 1;
    int motionCount = 0;
    int maxDistance{0};
    const int maxMotions = 500; // Adjust based on your needs
    bool targetReached = false;
    bool hasReachedTarget = false;

    static int totalNumberOfModules;
    int nbWaitedAnswers = 0;
    P2PNetworkInterface* parent = nullptr;
    P2PNetworkInterface* pathInterface = nullptr;
    static int currentMovingModuleId;
    bool stopBFS;
public:
    CatomsTest2BlockCode(Catoms3DBlock *host);
    ~CatomsTest2BlockCode() {};

    void startup() override;
    void processLocalEvent(EventPtr pev) override;
    void onMotionEnd() override;

    void handleSampleMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);
    void floodDistance();
    void handleBackMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);
    void handleReportMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);
    void handleReportbckMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);

    void parseUserElements(TiXmlDocument *config) override {}
    void parseUserBlockElements(TiXmlElement *config) override {}

    void onBlockSelected() override;
    void onAssertTriggered() override;
    void onUserKeyPressed(unsigned char c, int x, int y) override {}
    void onGlDraw() override {}
    void onTap(int face) override {}

    bool parseUserCommandLineArgument(int& argc, char **argv[]) override;
    std::string onInterfaceDraw() override;

    static BlockCode *buildNewBlockCode(BuildingBlock *host) {
        return (new CatomsTest2BlockCode((Catoms3DBlock*)host));
    };
};

#endif /* CatomsTest2BlockCode_H_ */
