#ifndef AssigningIDsBlockCode_H_
#define AssigningIDsBlockCode_H_

#include "robots/blinkyBlocks/blinkyBlocksWorld.h"
#include "robots/blinkyBlocks/blinkyBlocksBlockCode.h"
#include "robots/blinkyBlocks/blinkyBlocksSimulator.h"

static const int TYPE_1_EXPLORE = 2001;
static const int TYPE_2_CONFIRM_CHILD = 2002;
static const int TYPE_3_REJECT_CHILD = 2003;
static const int TYPE_4_REPORT_SIZE = 2004;
static const int TYPE_5_ASSIGN_ID = 2005;


using namespace BlinkyBlocks;

class AssigningIDsBlockCode : public BlinkyBlocksBlockCode {
private:
    int distance;
    BlinkyBlocksBlock *module;
public :


bool subtree_size_sent = false;

bool isLeader = false;
bool isDiscovered = false;
    P2PNetworkInterface* parent ;
    // int unique_id = -1;
    std::vector<P2PNetworkInterface*> neighbours;
    std::vector<P2PNetworkInterface*> children;
    std::vector<int> childrenSizes;
    int receivedSizeReports;
 int unique_id = -1;
 int subtree_size = 1;
 bool subtreeSizeSent = false;


    AssigningIDsBlockCode(BlinkyBlocksBlock *host);
    ~AssigningIDsBlockCode() {};

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
        return (new AssigningIDsBlockCode((BlinkyBlocksBlock*)host));
    };
/*****************************************************************************/
};

#endif /* AssigningIDsBlockCode_H_ */