#ifndef IDAssignmentCode_H_
#define IDAssignmentCode_H_

#include "robots/blinkyBlocks/blinkyBlocksSimulator.h"
#include "robots/blinkyBlocks/blinkyBlocksWorld.h"
#include "robots/blinkyBlocks/blinkyBlocksBlockCode.h"

// Define message types for each phase
static const int TYPE_1_EXPLORE = 2001;
static const int TYPE_2_CONFIRM_CHILD = 2002;
static const int TYPE_3_REJECT_CHILD = 2003;
static const int TYPE_4_REPORT_SIZE = 2004;
static const int TYPE_5_ASSIGN_ID = 2005;

using namespace BlinkyBlocks;

class IdTreeCode : public BlinkyBlocksBlockCode {
private:
    BlinkyBlocksBlock *module = nullptr;
    int uniqueID;
    int subtreeSize;
    bool subtreeSizeSent;
    int receivedSizeReports;

    bool isLeader;
    bool isDiscovered;

    P2PNetworkInterface *parentModule;
    std::vector<P2PNetworkInterface *> neighbours;
    std::vector<P2PNetworkInterface *> childrenModules;
    std::vector<int> childrenSizes;

public:
    IdTreeCode(BlinkyBlocksBlock *host);
    ~IdTreeCode() {};

    // Primary functions for each phase
    void startup() override;
    void exploreNeighbors(std::shared_ptr<Message> msg, P2PNetworkInterface *sender);
    void confirmChild(std::shared_ptr<Message> msg, P2PNetworkInterface *sender);
    void rejectChild(std::shared_ptr<Message> msg, P2PNetworkInterface *sender);
    void reportSize(std::shared_ptr<Message> msg, P2PNetworkInterface *sender);
    void assignId(std::shared_ptr<Message> msg, P2PNetworkInterface *sender);
    
    // Phase-specific functions
    void CHECK_SubtreeSize();
    void CHECK(int nextID); // Updated to accept int parameter

    // Draw function for displaying information
    string onInterfaceDraw() override;

    // Configuration file parsing
    void parseUserBlockElements(TiXmlElement *config) override;

    // Code instantiation for the module
    static BlockCode *buildNewBlockCode(BuildingBlock *host) {
        return new IdTreeCode((BlinkyBlocksBlock*)host);
    }
};

#endif /* IDAssignmentCode_H_ */

