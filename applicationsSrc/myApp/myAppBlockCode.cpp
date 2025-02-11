#include "myAppBlockCode.hpp"


using namespace BlinkyBlocks;

myAppBlockCode::myAppBlockCode(BlinkyBlocksBlock *host) : BlinkyBlocksBlockCode(host), module(host) {
  if (not host) return;

  // Register callbacks for message types
  addMessageEventFunc2(BROADCAST_MSG_ID,
                       std::bind(&myAppBlockCode::myBroadcastFunc, this,
                                 std::placeholders::_1, std::placeholders::_2));
  addMessageEventFunc2(ACKNOWLEDGE_MSG_ID,
                       std::bind(&myAppBlockCode::myAcknowledgeFunc, this,
                                 std::placeholders::_1, std::placeholders::_2));
}




void myAppBlockCode::startup() {
  console << "start " << getId() << "\n";

  currentRound = 0;

}

void myAppBlockCode::myBroadcastFunc(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender) {


  MessageOf<pair<int, int>> *msg = static_cast<MessageOf<pair<int, int>> *>(_msg.get());
  pair<int, int> msgData = *msg->getData();

  console << "rec. Flood (" << msgData.first << "," << msgData.second << ") from " << sender->getConnectedBlockId() << "\n";

  if (parent == nullptr || msgData.first < distance) {
    distance = msgData.first;
    setColor(distance);
    currentRound = msgData.second;
    parent = sender;


    string str = "distance(" + to_string(distance + 1) + "," + to_string(currentRound) + ")";
    nbWaitedAnswers = sendMessageToAllNeighbors(
        str.c_str(),
        new MessageOf<pair<int, int>>(BROADCAST_MSG_ID, make_pair(distance + 1, currentRound)),
        1000, 100, 1, sender
    );

    if (nbWaitedAnswers == 0) {
      sendMessage("ack2parent", new Message(ACKNOWLEDGE_MSG_ID), parent, 1000, 100);
    }

  } else {
    sendMessage("ack2sender", new Message(ACKNOWLEDGE_MSG_ID), sender, 1000, 100);
  }

}

void myAppBlockCode::myAcknowledgeFunc(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender) {
  nbWaitedAnswers--;
  console << "rec. Ack(" << nbWaitedAnswers << ") from " << sender->getConnectedBlockId() << "\n";

  if (nbWaitedAnswers == 0) {
    if (parent == nullptr) {
      setColor(WHITE);
    } else {
      sendMessage("ack2parent", new Message(ACKNOWLEDGE_MSG_ID), parent, 1000, 100);
    }
  }
  if (isLeader) {
    setColor(RED);
  }
}

//void myAppBlockCode::parseUserBlockElements(TiXmlElement *config) {
// const char *attr = config->Attribute("leader");
  //if (isLeader) {
    //std::cout << getId() << " is leader!" << std::endl;
 // }
//}

void myAppBlockCode::onUserKeyPressed(unsigned char c, int x, int y) {
  switch (c) {
    case 'a':
      std::cout << "key a" << std::endl;
      break;
    case 'd':
      break;
  }

}

void myAppBlockCode::onTap(int face) {

  isLeader=true;
    setColor(RED);

  distance=0;
  currentRound++;

  nbWaitedAnswers = sendMessageToAllNeighbors(
        "distance(1,1)",
        new MessageOf<pair<int, int>>(BROADCAST_MSG_ID, make_pair(distance + 1, currentRound)),
        1000, 100, 0
    );
  std::cout << "Leader is: " << getId() << std::endl;

}
