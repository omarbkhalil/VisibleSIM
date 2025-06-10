#ifndef CatomsTest3BlockCode_H_
#define CatomsTest3BlockCode_H_

#include "events/events.h"
#include "robots/catoms3D/catoms3DWorld.h"
#include "robots/catoms3D/catoms3DBlockCode.h"
#include <unordered_set>
#include <vector>
#include <unordered_map>
#include <string>
#include <queue>
#include <set>
using namespace std;

// Message IDs
static const int SAMPLE_MSG_ID = 1000;
static const int BACK_MSG_ID = 1001;
static const int REPORT_MSG_ID = 1002;
static const int REPORTbck_MSG_ID = 1003;

using namespace Catoms3D;

class CatomsTest3BlockCode : public Catoms3DBlockCode {
private:
    int distance;
    Catoms3DBlock *module;

    // A* heuristic function (declared as static)
    static double heuristic(const Cell3DPosition &current, const Cell3DPosition &goal);

    void saveOptimalPath(const vector<Cell3DPosition> &path, const std::string &phaseLabel);

    std::vector<Cell3DPosition> loadOptimalPathFromFile(const std::string &phaseLabel);

    void pushOptimalPath(const std::vector<Cell3DPosition> &path, const std::string &filename);

    std::vector<Cell3DPosition> popOptimalPath(const std::string &filename);

    // New helper functions to centralize the A* initialization logic.
    void initializeAStar();

    void initiatePathfinding();

    // Already declared function to initiate pathfinding for the next module.
    void initiateNextModulePathfinding();

    bool getAllPossibleMotionsFromPosition(Cell3DPosition position, vector<pair<short, short>> &links,
                                           vector<Cell3DPosition> &reachablePositions);

    void scheduleOneTeleportD();

    void scheduleOneTeleportT();

    std::vector<Cell3DPosition> findOptimalPath(const Cell3DPosition &start, const Cell3DPosition &goal);

    void serializePath(const std::vector<Cell3DPosition> &path, const std::string &filename);

    bool deserializePath(std::vector<Cell3DPosition> &path, const std::string &filename);

    void savePathToAllFile(int moduleId, const std::vector<Cell3DPosition> &path, const Cell3DPosition &startPos,
                           const Cell3DPosition &goalPos);

    void restoreAllPathsFromFile();

    bool restorePathForThisModule();

    void savePathToFile(const vector<Cell3DPosition> &path);

    bool restorePathFromFile();

    bool hasStartedPathfinding;
    static std::vector<std::vector<Cell3DPosition> > savedPaths;

    static std::map<Cell3DPosition, std::vector<Cell3DPosition> > cells;
    static std::vector<Cell3DPosition> visited;
    static std::vector<Cell3DPosition> teleportedPositions;
    static std::vector<std::vector<Cell3DPosition> > globalOptimalPaths;

    std::vector<Cell3DPosition> matchingPath;


    static std::priority_queue<std::pair<Cell3DPosition, double>,
        std::vector<std::pair<Cell3DPosition, double> >,
        std::function<bool(std::pair<Cell3DPosition, double>, std::pair<Cell3DPosition, double>)> > openSet;
    static std::map<Cell3DPosition, double> gScore, fScore;
    static std::map<Cell3DPosition, Cell3DPosition> cameFrom;
    static std::set<Cell3DPosition> closedSet;
    static bool teleportationInProgress;
    static bool savingEnabled;

    bool useGlobalPath = false;
    int pathIndex = 0;
    std::vector<Cell3DPosition> savedPath;

    static Cell3DPosition Origin;
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
    P2PNetworkInterface *parent = nullptr;
    P2PNetworkInterface *pathInterface = nullptr;
    static int currentMovingModuleId;
    bool isReturning = false;
    static const Cell3DPosition goalPosition;
    bool restoredMode = false;
    Cell3DPosition initialPosition;
    bool finishedD = false;
    bool finishedT = false;
    bool finishedA = false;
    bool aPhaseInitialized = false;
bool initializedAPhase = false;
    bool startedA = false;
 bool tPhasePathSaved = false;
    std::vector<Cell3DPosition> tPhasePath;
    size_t                      tPhaseIndex;
    int aPhaseIndex = 0;
    static std::vector<Cell3DPosition> dPhasePath;
    static int dPhaseIndex;

public:

    std::vector<Cell3DPosition> aPhasePath;
    size_t                       aPhaseIdx = 0;
    std::vector<Cell3DPosition> savedPathForCurrentModule;

    static vector<Cell3DPosition> optimalPath;
    bool loadedFromFile;
    bool usingSavedPath;
    static int pathCount;

    static std::map<int, std::vector<Cell3DPosition> > allRestoredPaths;

    static bool haveRestoredAll;

    static int currentPathIndex;

    static bool fileParsed;

    static std::vector<Cell3DPosition> currentOptimalPath;

    CatomsTest3BlockCode(Catoms3DBlock *host);

    std::vector<std::vector<Cell3DPosition> > loadAllOptimalPaths();

    std::vector<Cell3DPosition> loadOptimalPath();

    void parseSavedPathsFile(const std::string &filename);

    vector<Cell3DPosition> loadSavedPath();

    void followSavedPath(const vector<Cell3DPosition> &savedPath);

    void loadOptimalPaths();

    void moveModuleAlongSavedPath();

    ~CatomsTest3BlockCode() {
    };

    // Queues for start and target positions.
    static std::queue<Cell3DPosition> startQueue;
    static std::queue<Cell3DPosition> targetQueue;

    static std::queue<Cell3DPosition> startQueue1;
    static std::queue<Cell3DPosition> targetQueue1;

    static std::queue<Cell3DPosition> startD;
    static std::queue<Cell3DPosition> targetD;
    static std::queue<Cell3DPosition> startT;
    static std::queue<Cell3DPosition> targetT;
    static std::queue<Cell3DPosition> startA;
    static std::queue<Cell3DPosition> targetA;
 static bool NFound ;

    void startup() override;

    void processLocalEvent(EventPtr pev) override;

    void onMotionEnd() override;

    void handleSampleMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);

    void floodDistance();

    void handleBackMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);

    void handleReportMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);

    void handleReportbckMessage(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);

    bool runBFS(const Cell3DPosition &startPos, const Cell3DPosition &goalPos);

    void reconstructPath(const Cell3DPosition &startPos, const Cell3DPosition &goalPos);

    void scheduleNextTeleport();

    std::vector<Cell3DPosition> getAdjacentCells(const Cell3DPosition &cell);

    void parseUserElements(TiXmlDocument *config) override {
    }

    void parseUserBlockElements(TiXmlElement *config) override {
    }

    void onBlockSelected() override;

    void onAssertTriggered() override;

    void onUserKeyPressed(unsigned char c, int x, int y) override {
    }

    void onGlDraw() override {
    }

    void onTap(int face) override {
    }

    bool parseUserCommandLineArgument(int &argc, char **argv[]) override;

    std::string onInterfaceDraw() override;

    static BlockCode *buildNewBlockCode(BuildingBlock *host) {
        return (new CatomsTest3BlockCode((Catoms3DBlock *) host));
    };
};

#endif /* CatomsTest3BlockCode_H_ */
