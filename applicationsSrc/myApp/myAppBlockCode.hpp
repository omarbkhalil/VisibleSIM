#ifndef MyAppBlockCode_H_
#define MyAppBlockCode_H_

#include "robots/blinkyBlocks/blinkyBlocksSimulator.h"
#include "robots/blinkyBlocks/blinkyBlocksWorld.h"
#include "robots/blinkyBlocks/blinkyBlocksBlockCode.h"

static const int BROADCAST_MSG_ID = 1001;
static const int ACKNOWLEDGE_MSG_ID = 1002;

using namespace BlinkyBlocks;

class myAppBlockCode : public BlinkyBlocksBlockCode {
  private:
    BlinkyBlocksBlock *module = nullptr;
    bool isLeader=false;
    int distance=0;
    int currentRound=0;
    int nbWaitedAnswers=0;
    P2PNetworkInterface *parent= nullptr;
  public :
    myAppBlockCode(BlinkyBlocksBlock *host);
    ~myAppBlockCode() {}

/**
  * This function is called on startup of the blockCode, it can be used to perform initial
  *  configuration of the host or this instance of the program.
  * @note this can be thought of as the main function of the module
  */
  void startup() override;

/**
  * @brief Message handler for the message `broadcast`
  * @param _msg Pointer to the message received by the module, requires casting
  * @param sender Connector of the module that has received the message and that is connected to the sender
  */
  void myBroadcastFunc(std::shared_ptr<Message>_msg,P2PNetworkInterface *sender);

/**
  * @brief Message handler for the message `acknowledge`
  * @param _msg Pointer to the message received by the module, requires casting
  * @param sender Connector of the module that has received the message and that is connected to the sender
  */
  void myAcknowledgeFunc(std::shared_ptr<Message>_msg,P2PNetworkInterface *sender);

/**
  * @brief Provides the user with a pointer to the configuration file parser, which can be used to read additional user information from each block config. Has to be overridden in the child class.
  * @param config : pointer to the TiXmlElement representing the block configuration file, all information related to concerned block have already been parsed
  *
  */
  //void parseUserBlockElements(TiXmlElement *config) override;

/**
  * User-implemented keyboard handler function that gets called when
  *  a key press event could not be caught by openglViewer
  * @param c key that was pressed (see openglViewer.cpp)
  * @param x location of the pointer on the x axis
  * @param y location of the pointer on the y axis
  * @note call is made from GlutContext::keyboardFunc (openglViewer.h)
  */
  void onUserKeyPressed(unsigned char c, int x, int y) override;
/**
  * @brief This function is called when a module is tapped by the user. Prints a message to the console by default.
  * Can be overloaded in the user blockCode
  * @param face face that has been tapped
  */
  void onTap(int face) override;
/*****************************************************************************/
/** needed to associate code to module                                      **/
  static BlockCode *buildNewBlockCode(BuildingBlock *host) {
    return(new myAppBlockCode((BlinkyBlocksBlock*)host));
  }
/*****************************************************************************/
};


#endif /* MyAppBlockCode_H_ */
