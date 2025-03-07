
/**
 * @file BBbalanceCode.cpp
 * Generated by VisibleSim BlockCode Generator
 * https://services-stgi.pu-pm.univ-fcomte.fr/visiblesim/generator.php#
 * @author yourName
 * @date 2023-10-06                                                                     
 **/
 
#include "BBbalanceCode.hpp"

CenterData *globalCenter=nullptr;

BBbalanceCode::BBbalanceCode(BlinkyBlocksBlock *host):BlinkyBlocksBlockCode(host),module(host) {
    // @warning Do not remove block below, as a blockcode with a NULL host might be created
    //  for command line parsing
    if (not host) return;
    parent = nullptr;

    // Registers a callback (myBroadcastFunc) to the message of type R
    addMessageEventFunc2(BUILDST_MSG_ID,
                       std::bind(&BBbalanceCode::myBuildSTFunc,this,
                       std::placeholders::_1, std::placeholders::_2));

    // Registers a callback (myAckCentreFunc) to the message of type K
    addMessageEventFunc2(ACKCENTRE_MSG_ID,
                       std::bind(&BBbalanceCode::myAckCentreFunc,this,
                       std::placeholders::_1, std::placeholders::_2));

    // Registers a callback (myAckCentreFunc) to the message of type K
    addMessageEventFunc2(FLOODSUPPORT_MSG_ID,
                         std::bind(&BBbalanceCode::myFloodSupportFunc,this,
                                   std::placeholders::_1, std::placeholders::_2));

    // Registers a callback (myAckCentreFunc) to the message of type K
    addMessageEventFunc2(RESTART_MSG_ID,
                         std::bind(&BBbalanceCode::myRestartFunc,this,
                                   std::placeholders::_1, std::placeholders::_2));
}

void BBbalanceCode::startup() {
    console << "start " << getId() << "\n";
    currentStage=0;
    if (isLeader) { // At least one module must be "leader" in the config file
        target->highlight();
        setColor(RED);
        currentStage++;
        initCenter();
        nbWaitedAnswers=sendMessageToAllNeighbors(new MessageOf<uint8_t>(BUILDST_MSG_ID,currentStage),1000,100,0);
    }	
}

void BBbalanceCode::initCenter() {
    currentCenter.reset();
    Vector3D gridScale = BlinkyBlocksWorld::getWorld()->lattice->gridScale;
    Vector3D corner = module->getGlBlock()->getPosition();
    currentCenter.addCenter(corner+0.5*gridScale);
    if (module->position[2]==0) { // if is on the floor
        Vector2D baseCorner(corner[0],corner[1]);
        currentCenter.addCorners(baseCorner,Vector2D(gridScale[0],gridScale[1]));
    }
    blinkState=0;
}

void BBbalanceCode::myBuildSTFunc(std::shared_ptr<Message>_msg, P2PNetworkInterface*sender) {
    MessageOf<uint8_t>* msg = static_cast<MessageOf<uint8_t>*>(_msg.get());
    uint8_t msgStage = *msg->getData();
    console << "rec. BuildSP(" << int(msgStage) << ") from " << sender->getConnectedBlockId() << "\n";
    if (parent==nullptr || currentStage<msgStage) {
        parent=sender;
        children.clear();
        currentStage = msgStage;
        setColor(currentStage+3);
        Vector3D gridScale = BlinkyBlocksWorld::getWorld()->lattice->gridScale;
        Vector3D corner = module->getGlBlock()->getPosition();
        initCenter();
        nbWaitedAnswers=sendMessageToAllNeighbors(new MessageOf<uint8_t>(BUILDST_MSG_ID,currentStage),1000,100,1,sender);
        if (nbWaitedAnswers==0) {
            sendMessage("ack2parent",new MessageOf<CenterData>(ACKCENTRE_MSG_ID,currentCenter),parent,1000,100);
        }
    } else {
        CenterData nocenter;
        sendMessage("ack2sender",new MessageOf<CenterData>(ACKCENTRE_MSG_ID,nocenter),sender,1000,100);
    }
}

void BBbalanceCode::myAckCentreFunc(std::shared_ptr<Message>_msg, P2PNetworkInterface*sender) {
    MessageOf<CenterData>* msg = static_cast<MessageOf<CenterData>*>(_msg.get());
    CenterData msgData = *msg->getData();

    nbWaitedAnswers--;
    Vector3D center=msgData.getCenter();
    console << "rec. Ack(" << int(nbWaitedAnswers) << "," << center << ") from " << sender->getConnectedBlockId() << "\n";
    if (!msgData.isNull()) {
        currentCenter.addCenter(msgData);
        currentCenter.merge(msgData);
        center=currentCenter.getCenter();
        console << "Center:" << center[0] << "," << center[1] << ","<< center[2]<< "\n";
        auto v = currentCenter.getSupport();
        console << "Poly:" << (v?v->N:0) << "\n";
        children.push_back(sender);
    }
    if (nbWaitedAnswers==0) {
        if (parent==nullptr) {
            setColor(currentCenter.isStable()?GREEN:RED);
            globalCenter=&currentCenter;
            console << "Center:" << globalCenter->getCenter() << "(" << currentCenter.isStable() << ")\n";
            cout << getId() << ": Center=" << globalCenter->getCenter() << endl;
            for (auto &p:children) {
                sendMessage("floodSupport",new MessageOf<CenterData>(FLOODSUPPORT_MSG_ID,currentCenter),p,500,100);
            }
        } else {
            sendMessage("ack2parent",new MessageOf<CenterData>(ACKCENTRE_MSG_ID,currentCenter),parent,1000,100);
        }
        cout << getId() << ": " << endl;
        for (auto &p:children) {
            cout << "[" << p->getConnectedBlockId() << "]";
        }
        cout << endl;
    }
}

void BBbalanceCode::myFloodSupportFunc(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender) {
    MessageOf<CenterData>* msg = static_cast<MessageOf<CenterData>*>(_msg.get());
    CenterData msgData = *msg->getData();
    console << "rec. Support("<< msgData.isStable() << ") from " << sender->getConnectedBlockId() << "\n";

    currentCenter = msgData;
    setColor(currentCenter.isStable()?GREEN:RED);
    for (auto &p:children) {
        sendMessage("floodSupport",new MessageOf<CenterData>(FLOODSUPPORT_MSG_ID,currentCenter),p,500,100);
    }

    blinkState=0;
    Cell3DPosition pos;
    int i=0;
    while (i<6 && !(module->getNeighborPos(i,pos) && target->isInTarget(pos) && !module->getInterface(i)->isConnected())) {
        i++;
    }
    if (i<6) {
            auto newCenter = currentCenter;
            Vector3D gridScale = BlinkyBlocksWorld::getWorld()->lattice->gridScale;
            Vector3D wpos(pos[0]*gridScale[0],pos[1]*gridScale[1],pos[2]*gridScale[2]);
            Vector3D neighborCenter = wpos+0.5*gridScale;
            newCenter.addCenter(neighborCenter);
            setColor(newCenter.isStable()?DARKGREEN:ORANGE);
            console << "+" << pos << ":" << newCenter.isStable() << "\n";
            blinkState=1;
            scheduler->schedule(new InterruptionEvent<pair<bool,uint8_t>>(scheduler->now() + 1000000, this->module, make_pair(newCenter.isStable(),currentStage)));
    }
}

void BBbalanceCode::myRestartFunc(std::shared_ptr<Message> _msg, P2PNetworkInterface *sender) {
    MessageOf<uint8_t>* msg = static_cast<MessageOf<uint8_t>*>(_msg.get());
    uint8_t msgStage = *msg->getData();
    console << "rec. Restart(" << int (msgStage) << ") from " << sender->getConnectedBlockId() << "\n";

    if (parent) {
        if (currentStage==msgStage) {
            sendMessage("restart",new MessageOf<uint8_t>(RESTART_MSG_ID,currentStage),parent,500,100);
        }
    } else {
        // recompute the center
        currentStage++;
        setColor(RED);
        initCenter();
        children.clear();
        nbWaitedAnswers=sendMessageToAllNeighbors(new MessageOf<uint8_t>(BUILDST_MSG_ID,currentStage),1000,100,0);
    }
}

void BBbalanceCode::onInterruptionEvent(shared_ptr<Event> event) {
    InterruptionEvent<pair<bool,uint8_t>>* msg = static_cast<InterruptionEvent<pair<bool,uint8_t>>*>(event.get());
    bool isStable = (msg->data).first;
    uint8_t msgStage = (msg->data).second;
    cout << getId() << ": interruption received (" << isStable << "/" << int(msgStage) << ")" << endl;
    // check if the port is connected
    if (blinkState && msgStage==currentStage) {
            scheduler->schedule(new InterruptionEvent<pair<bool,uint8_t>>(scheduler->now() + 1000000, this->module,make_pair(isStable,currentStage)));
            setColor(blinkState==1?(isStable?DARKGREEN:ORANGE):(currentCenter.isStable()?GREEN:RED));
            blinkState=(blinkState==1)?2:1;
    }
}

void BBbalanceCode::onNeighborChanged(uint64_t face, int action) {
    // ask for a new computation of the center to the leader
    if (parent!=nullptr) {
        console << "NeighborChanged:" << face << " ," << action << "\n";
        sendMessage("restart",new MessageOf<uint8_t>(RESTART_MSG_ID,currentStage),parent,500,100);
    }
}

void BBbalanceCode::parseUserBlockElements(TiXmlElement *config) {
    const char *attr = config->Attribute("leader");
    isLeader = (attr?Simulator::extractBoolFromString(attr):false);
    if (isLeader) {
        std::cout << getId() << " is leader!" << std::endl; // complete with your code
    }
}

/* onGlDraw Functiondraws a 3x3x10 box in each cell of the grid.
 * This function is executed once by the first or the selected module */
void BBbalanceCode::onGlDraw() {
    static const float colorOut[4]={5.0f,1.0f,1.0f,1.0f};
    static const float colorIn[4]={1.0f,5.0f,1.0f,1.0f};

    if (globalCenter) {
        Vector3D gridScale = BlinkyBlocksWorld::getWorld()->lattice->gridScale;

        glDisable(GL_TEXTURE);
        auto support = globalCenter->getSupport();
        auto center = globalCenter->getCenter();
        glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,globalCenter->isStable()?colorIn:colorOut);
        glPushMatrix();
        glTranslatef(center[0],center[1],center[2]);
        glBegin(GL_LINES);
        glVertex3f(-10, -10, -10);
        glVertex3f(0, 0, 0);
        glVertex3f(10,10, -10);
        glVertex3f(0,0, 0);
        glVertex3f(10, -10, -10);
        glVertex3f(0, 0, 0);
        glVertex3f(-10,10, -10);
        glVertex3f(0,0, 0);
        glEnd();
        glPopMatrix();
        glBegin(GL_LINES);
        glVertex3f(center[0],center[1],0);
        glVertex3f(center[0],center[1],center[2]);
        glEnd();
        glBegin(GL_LINE_LOOP);
        for (int i=0; i<support->N; i++) {
            glVertex3f(support->tabVertices[i].x,support->tabVertices[i].y,0.1);
        }
        glEnd();
    }

}