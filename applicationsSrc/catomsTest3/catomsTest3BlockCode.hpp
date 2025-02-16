#ifndef CatomsTest1BlockCode_H_
#define CatomsTest1BlockCode_H_

#include "events/events.h"
#include "robots/catoms3D/catoms3DWorld.h"
#include "robots/catoms3D/catoms3DBlockCode.h"
#include <unordered_set>
#include <vector>
#include <unordered_map>
#include <string>
#include <queue>
#include <set>


static const int SAMPLE_MSG_ID = 1000;
static const int BACK_MSG_ID = 1001;
static const int REPORT_MSG_ID = 1002;
static const int REPORTbck_MSG_ID = 1003;

using namespace Catoms3D;

class CatomsTest3BlockCode : public Catoms3DBlockCode {
private:
    int distance;
    Catoms3DBlock *module;



    // Declare A* related functions only once
    static double heuristic(const Cell3DPosition& current, const Cell3DPosition& goal);
    static map<Cell3DPosition, vector<Cell3DPosition>> cells;
    static vector<Cell3DPosition> visited;
    static vector<Cell3DPosition> teleportedPositions;


    static std::priority_queue<std::pair<Cell3DPosition, double>, std::vector<std::pair<Cell3DPosition, double>>, std::function<bool(std::pair<Cell3DPosition, double>, std::pair<Cell3DPosition, double>)>> openSet;
static std::map<Cell3DPosition, double> gScore, fScore;
    static std::map<Cell3DPosition, Cell3DPosition> cameFrom;
    static std::set<Cell3DPosition> closedSet;

    int currentModuleId = 1;
    int motionCount = 0;
    int maxDistance{0};
    const int maxMotions = 500; // Adjust based on your needs
    bool targetReached = false;
    bool hasReachedTarget = false;
    std::vector<Cell3DPosition> discoveredPath;
    std::map<Cell3DPosition, Cell3DPosition> parentMap;
    int teleportIndex = -1;
    static int totalNumberOfModules;
    int nbWaitedAnswers = 0;
    P2PNetworkInterface* parent = nullptr;
    P2PNetworkInterface* pathInterface = nullptr;
    static int currentMovingModuleId;
    bool isReturning = false;
    static const Cell3DPosition goalPosition;


public:
    CatomsTest3BlockCode(Catoms3DBlock *host);
    ~CatomsTest3BlockCode() {};



    //static bool activeModuleRunning;

    static std::queue<Cell3DPosition> startQueue;
    static std::queue<Cell3DPosition> targetQueue;


    void startup() override;
    void processLocalEvent(EventPtr pev) override;
    void onMotionEnd() override;

    void handleSampleMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);
    void floodDistance();
    void handleBackMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);
    void handleReportMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);
    void handleReportbckMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);


    bool runBFS(const Cell3DPosition &startPos,
                                  const Cell3DPosition &goalPos);

    void reconstructPath(const Cell3DPosition &startPos,
                                               const Cell3DPosition &goalPos);
    void scheduleNextTeleport();
    std::vector<Cell3DPosition> getAdjacentCells(const Cell3DPosition &cell) ;
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
        return (new CatomsTest3BlockCode((Catoms3DBlock*)host));
    };
};



#endif /* CatomsTest1BlockCode_H_ */
