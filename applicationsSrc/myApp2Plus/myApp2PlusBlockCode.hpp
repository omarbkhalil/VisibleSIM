// MyApp2plusBlockCode.hpp
#ifndef MyApp2plusCode_H_
#define MyApp2plusCode_H_

#include "robots/blinkyBlocks/blinkyBlocksSimulator.h"
#include "robots/blinkyBlocks/blinkyBlocksWorld.h"
#include "robots/blinkyBlocks/blinkyBlocksBlockCode.h"

static const int GO_MSG_ID   = 1001;
static const int BACK_MSG_ID = 1002;
static const int GO2_MSG_ID   = 1003;
static const int BACK2_MSG_ID = 1004;
using namespace BlinkyBlocks;

// ---- ADD THIS: payload used in BACK messages
struct BackPayload {
	int  kind;   // 0 = ACK_UP, 1 = SELECT_DOWN
	int  dist;   // best distance in subtree
	bID  id;     // node id (use bID, not int)
	int distFromB;
};

class MyApp2plusBlockCode : public BlinkyBlocksBlockCode {
private:
	// ---- ADD THESE FIELDS
	bool isA = false;

	BlinkyBlocksBlock *module = nullptr;

	int myDistance  = 0;
	int nbWaitedAns = 0;

	// aggregation for farthest-from-leader
	int  bestDist = -1;
	bID  bestId   = 0;
	P2PNetworkInterface *winnerChild =nullptr;


	P2PNetworkInterface *parent2 = nullptr;
	P2PNetworkInterface *winnerChild2 = nullptr;
	int  myDistance2  = 0;
	int  nbWaitedAns2 = 0;
	int  bestDist2    = -1;
	bID  bestId2      = 0;

	P2PNetworkInterface *parent = nullptr;

public:
	MyApp2plusBlockCode(BlinkyBlocksBlock *host);
	~MyApp2plusBlockCode() {}

	void startup() override;
	void myGoFunc(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);
	void myBackFunc(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);
	void myGo2Func(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);
	void myBack2Func(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender);

	void parseUserBlockElements(TiXmlElement *config) override;

	static BlockCode *buildNewBlockCode(BuildingBlock *host) {
		return(new MyApp2plusBlockCode((BlinkyBlocksBlock*)host));
	}
};

#endif
