#include "LeaderElecLab2BlockCode.hpp"

using namespace BlinkyBlocks;

#include <fstream>
static std::ofstream id_output("IDs_output.txt");

// Reserve bits inside unique_id: [ ROOT_TAG | BRANCH_TAG | LOCAL_OFFSET ]
static constexpr int SHIFT_BRANCH = 12;   // allows up to 4096 nodes per branch
static constexpr int SHIFT_ROOT   = 20;   // allows up to 2^8 = 256 branches (since 20-12 = 8 bits)

static constexpr int MASK_LOCAL   = (1 << SHIFT_BRANCH) - 1;
static constexpr int MASK_BRANCH  = ((1 << (SHIFT_ROOT - SHIFT_BRANCH)) - 1) << SHIFT_BRANCH;
static constexpr int MASK_ROOT    = ~((1 << SHIFT_ROOT) - 1);


LeaderElecLab2BlockCode::LeaderElecLab2BlockCode(BlinkyBlocksBlock *host) : BlinkyBlocksBlockCode(host) {
    // @warning Do not remove block below, as a blockcode with a NULL host might be created
    //  for command line parsing
    if (not host) return;

    // Registers a callback (handleSampleMessage) to the message of type SAMPLE_MSG_ID
    addMessageEventFunc2(TYPE_1_EXPLORE, std::bind(&LeaderElecLab2BlockCode::exploreNeighbors, this, std::placeholders::_1, std::placeholders::_2));
     addMessageEventFunc2(TYPE_2_CONFIRM_CHILD, std::bind(&LeaderElecLab2BlockCode::confirmChild, this, std::placeholders::_1, std::placeholders::_2));
     addMessageEventFunc2(TYPE_3_REJECT_CHILD, std::bind(&LeaderElecLab2BlockCode::rejectChild, this, std::placeholders::_1, std::placeholders::_2));
     addMessageEventFunc2(TYPE_4_REPORT_SIZE, std::bind(&LeaderElecLab2BlockCode::reportSize, this, std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(TYPE_5_ASSIGN_ID, std::bind(&LeaderElecLab2BlockCode::assignId, this, std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(TYPE_6_BROADCAST_TOTAL,
        std::bind(&LeaderElecLab2BlockCode::onBroadcastTotals, this,
                  std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(TYPE_7_TREE_INFO,
        std::bind(&LeaderElecLab2BlockCode::onTreeInfo, this,
                  std::placeholders::_1, std::placeholders::_2));

    addMessageEventFunc2(TYPE_8_WINNER_CAND,
        std::bind(&LeaderElecLab2BlockCode::onWinnerCand, this,
                  std::placeholders::_1, std::placeholders::_2));

    addMessageEventFunc2(TYPE_10_GRID_COLOR,
        std::bind(&LeaderElecLab2BlockCode::onGridPaint, this,
                  std::placeholders::_1, std::placeholders::_2));

    addMessageEventFunc2(TYPE_11_WINNER_UP,
        std::bind(&LeaderElecLab2BlockCode::onWinnerUp, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Set the module pointer
    module = static_cast<BlinkyBlocksBlock*>(hostBlock);
  }

void LeaderElecLab2BlockCode::startup() {
    console << "start\n";

    // --- reset per-run state ---
    neighbours.clear();
    children.clear();
    childrenSizes.clear();
    subtree_size     = 1;     // each node counts itself
    subtree_weight   = 0;     // will set after we compute BChoice
    isDiscovered     = isDiscovered; // unchanged unless we start as leader

    // --- interface scan + neighbor list + bitmask/weight ---
    int totalConnectedInt = 0;
    unsigned int BChoice  = 0;

    for (int i = 0; i < NumbOfInterfaces; ++i) interfaceStatus[i] = 0;

    for (int i = 0; i < NumbOfInterfaces; ++i) {
        P2PNetworkInterface* ni = module->getInterface(i);
        if (ni && ni->isConnected()) {
            interfaceStatus[i] = 1;
            neighbours.push_back(ni);
            totalConnectedInt++;
            BChoice |= (1u << i);   // set bit i
        }
    }

    // <-- IMPORTANT: set after BChoice is computed
    weight          = static_cast<int>(BChoice);
    subtree_weight  = weight;                 // include my own weight

    // Pretty print binary
    std::string bin;
    bin.reserve(NumbOfInterfaces);
    for (int i = NumbOfInterfaces - 1; i >= 0; --i)
        bin.push_back(interfaceStatus[i] ? '1' : '0');

    console << "interfaces (i=" << (NumbOfInterfaces - 1) << "..0) = "
            << bin
            << "  | decimal=" << weight
            << "  | totalConnected=" << totalConnectedInt
            << "\n";

    // Neighbours log
    console << "Module " << getId() << " initial neighbours: ";
    if (neighbours.empty()) console << "NONE\n";
    else {
        for (auto* n : neighbours) console << n->getConnectedBlockId() << " ";
        console << "\n";
    }

    // Degree-1 nodes are local roots; OR with XML leaders
    const bool Lroot = (totalConnectedInt == 1);
    if (Lroot) isLeader = true;

    // Your existing startup branch
    if (isLeader && !isDiscovered) {
        isDiscovered = true;
        sendMessageToAllNeighbors("Explore", new Message(TYPE_1_EXPLORE), 1000, 0, 0);
        module->setColor(RED);
        distance = 0;
    } else {
        distance = -1; // Unknown distance
        hostBlock->setColor(LIGHTGREY);
    }
}



void LeaderElecLab2BlockCode::handleSampleMessage(std::shared_ptr<Message> _msg,
                                               P2PNetworkInterface* sender) {
    MessageOf<int>* msg = static_cast<MessageOf<int>*>(_msg.get());

    int d = *msg->getData() + 1;
    console << " received d =" << d << " from " << sender->getConnectedBlockId() << "\n";

    if (distance == -1 || distance > d) {
        console << " updated distance = " << d << "\n";
        distance = d;
        module->setColor(Colors[distance % NB_COLORS]);

        // Broadcast to all neighbors but ignore sender
        // sendMessageToAllNeighbors("Sample Broadcast",
        //                           new MessageOf<int>(SAMPLE_MSG_ID,distance),100,200,1,sender);
    }
}

void LeaderElecLab2BlockCode::onMotionEnd() {
    console << " has reached its destination" << "\n";

    // do stuff
    // ...
}

void LeaderElecLab2BlockCode::processLocalEvent(EventPtr pev) {
    std::shared_ptr<Message> message;
    stringstream info;

    // Do not remove line below
    BlinkyBlocksBlockCode::processLocalEvent(pev);

    switch (pev->eventType) {
        case EVENT_ADD_NEIGHBOR: {
            // Do something when a neighbor is added to an interface of the module
            break;
        }

        case EVENT_REMOVE_NEIGHBOR: {
            // Do something when a neighbor is removed from an interface of the module
            break;
        }
    }
}

/// ADVANCED BLOCKCODE FUNCTIONS BELOW

void LeaderElecLab2BlockCode::onBlockSelected() {
    // Debug stuff:
    cerr << endl << "--- PRINT MODULE " << *module << "---" << endl;
}

void LeaderElecLab2BlockCode::onAssertTriggered() {
    console << " has triggered an assert" << "\n";

    // Print debugging some info if needed below
    // ...
}

bool LeaderElecLab2BlockCode::parseUserCommandLineArgument(int &argc, char **argv[]) {
    /* Reading the command line */
    if ((argc > 0) && ((*argv)[0][0] == '-')) {
        switch((*argv)[0][1]) {

            // Single character example: -b
            case 'b':   {
                cout << "-b option provided" << endl;
                return true;
            } break;

            // Composite argument example: --foo 13
            case '-': {
                string varg = string((*argv)[0] + 2); // argv[0] without "--"
                if (varg == string("foo")) { //
                    int fooArg;
                    try {
                        fooArg = stoi((*argv)[1]);
                        argc--;
                        (*argv)++;
                    } catch(std::logic_error&) {
                        stringstream err;
                        err << "foo must be an integer. Found foo = " << argv[1] << endl;
                        throw CLIParsingError(err.str());
                    }

                    cout << "--foo option provided with value: " << fooArg << endl;
                } else return false;

                return true;
            }

            default: cerr << "Unrecognized command line argument: " << (*argv)[0] << endl;
        }
    }

    return false;
}

string LeaderElecLab2BlockCode::onInterfaceDraw() {
    stringstream trace;
    trace << "Some value " << 123;
    return trace.str();
}

void LeaderElecLab2BlockCode::parseUserBlockElements(TiXmlElement *config) {
    const char *attr = config->Attribute("leader");
    isLeader = (attr?Simulator::extractBoolFromString(attr):false);
    if (isLeader) {
        std::cout << getId() << " is leader!" << std::endl; // complete with your code
    }
}


void LeaderElecLab2BlockCode::exploreNeighbors(std::shared_ptr<Message> msg,
                                             P2PNetworkInterface *sender) {
    console << "Module " << getId()
            << " rcv explore message: " << sender->getConnectedBlockId() << "\n";
    module->setColor(GREY);

    if (isDiscovered) {
        // Already discovered → type 3
        sendMessage("RejectChild", new Message(TYPE_3_REJECT_CHILD), sender, 1000, 0);
    } else {
        // First time discovery
        isDiscovered = true;
        parent = sender;

        // neighbours ← neighbours − {sender}
        neighbours.erase(
            std::remove(neighbours.begin(), neighbours.end(), sender),
            neighbours.end()
        );

        sendMessage("ConfirmChild", new Message(TYPE_2_CONFIRM_CHILD), sender, 1000, 0);

        // PSEUDOCODE:
        // if neighbours.size = 0 AND !subtree_size_sent then CHECK()
        // else for each neighbour send type 1
        if (neighbours.empty() && !subtreeSizeSent) {
            Check();    // Phase 2: I am a leaf → report size
        } else {
            for (P2PNetworkInterface *n : neighbours) {
                sendMessage("Explore",
                            new Message(TYPE_1_EXPLORE),
                            n,
                            1000,
                            0);
            }
        }
    }

    // do NOT call Check() again here
}

void LeaderElecLab2BlockCode::confirmChild(std::shared_ptr<Message> msg,
                                           P2PNetworkInterface *sender) {
    console << "Module " << getId()
            << " Confirmed Child: " << sender->getConnectedBlockId() << "\n";
    module->setColor(GREEN);

    // This neighbour is no longer pending
    neighbours.erase(std::remove(neighbours.begin(), neighbours.end(), sender), neighbours.end());

    // Register child
    children.push_back(sender);
    childrenSizes.push_back(0); // will be filled by TYPE_4_REPORT_SIZE

    // Leaf if no neighbours left AND no children (not here since we just added one),
    // otherwise trigger Check when all children report.
    if (neighbours.empty() && allChildrenReported()) {
        Check();
    }
}


void LeaderElecLab2BlockCode::rejectChild(std::shared_ptr<Message> msg,
                                          P2PNetworkInterface *sender) {
    console << "Module " << getId() << " Rejected Child: "
            << sender->getConnectedBlockId() << "\n";
    module->setColor(RED);

    // IMPORTANT: remove from pending neighbours
    neighbours.erase(std::remove(neighbours.begin(), neighbours.end(), sender), neighbours.end());

    // If after removing, there are no neighbours left:
    // - If also no children → leaf → Check() (will report size or start IDs)
    // - If there are children → wait for their TYPE_4_REPORT_SIZE (handled in reportSize)
    if (neighbours.empty() && (children.empty() || allChildrenReported())) {
        Check();
    }
}

void LeaderElecLab2BlockCode::reportSize(std::shared_ptr<Message> msg,
                                         P2PNetworkInterface *sender) {
    module->setColor(GREY);

    // read the pair (size, weight) instead of a single int
    auto* m = static_cast<MessageOf<SizeWeight>*>(msg.get());
    const SizeWeight child = *(m->getData());   // { size, weight }

    // find child index and store its subtree size (for ID spacing)
    for (int i = 0; i < static_cast<int>(children.size()); ++i) {
        if (children[i] == sender) {
            childrenSizes[i] += child.size;     // keep using size for ID layout
            break;
        }
    }

    // accumulate global totals
    subtree_size   += child.size;
    subtree_weight += child.weight;             // <-- now weight propagates

    if (neighbours.empty() && allChildrenReported()) {
        Check();
    }
}

void LeaderElecLab2BlockCode::assignId(std::shared_ptr<Message> msg, P2PNetworkInterface *sender) {
    MessageOf<int>* m = static_cast<MessageOf<int>*>(msg.get());
    int base_id = *(m->getData());

    // Store & log my ID
    int final_id = base_id;
    unique_id = final_id;
    id_output << "Module " << getId() << " = " << final_id << "\n";
    id_output.flush();

    // ---- LOCAL COLORING (no messages) ----
    int rootTag   = (final_id >> SHIFT_ROOT) & 0xFF;                   // if SHIFT_ROOT-SHIFT_BRANCH == 8
    int branchTag = (final_id >> SHIFT_BRANCH) & ((1 << (SHIFT_ROOT - SHIFT_BRANCH)) - 1);

    // Pick color scheme: by branch (distinct subtrees) or by root (all same per root)
    int colorIdx = (branchTag != 0) ? ((rootTag * 7 + branchTag) % NB_COLORS)
                                    : (rootTag % NB_COLORS);
    module->setColor(Colors[colorIdx]);

    //  Every node (including leaves) announces its tree info once
    sendTreeInfoToAllNeighbors();

    // ---- Forward IDs to my children, preserving high bits ----
    int highBits   = final_id & ~MASK_LOCAL;      // keep [ROOT_TAG | BRANCH_TAG]
    int next_local = (final_id & MASK_LOCAL) + 1; // start after me

    for (int i = 0; i < static_cast<int>(children.size()); ++i) {
        int childStartId = highBits | next_local;

        sendMessage("AssignId",
                    new MessageOf<int>(TYPE_5_ASSIGN_ID, childStartId),
                    children[i], 1000, 0);

        next_local += childrenSizes[i];   // skip this child's subtree width
    }

    if (!subtreeSizeSent) {
        Check();
    }
}


void LeaderElecLab2BlockCode::broadcastTotalsToChildren(const SizeWeight& tot) {
    // Send totals to all direct children; they will forward recursively.
    for (auto* ch : children) {
        sendMessage("BroadcastTotals",
            new MessageOf<SizeWeight>(TYPE_6_BROADCAST_TOTAL, tot),
            ch, 1000, 0);
    }

    // Also apply a local visualization if you want the root to display totals too:
    // (Optional) e.g., blue tint for "I know totals now"
    // module->setColor(BLUE);
}

void LeaderElecLab2BlockCode::onBroadcastTotals(std::shared_ptr<Message> msg,
                                                P2PNetworkInterface* sender) {
    auto* m = static_cast<MessageOf<SizeWeight>*>(msg.get());
    const SizeWeight tot = *(m->getData());

    // Cache my tree's global totals locally
    global_total_size   = tot.size;
    global_total_weight = tot.weight;
    global_total_score  = tot.size + tot.weight;

    console << "Module " << getId()
            << " got GLOBAL TOTALS: size=" << tot.size
            << ", weight=" << tot.weight
            << " (from parent " << (sender ? sender->getConnectedBlockId() : -1) << ")\n";
    maybeSendWinnerCandUp();

    // Forward to my children (do NOT echo back to sender)
    for (auto* ch : children) {
        if (ch != sender) {
            sendMessage("BroadcastTotals",
                new MessageOf<SizeWeight>(TYPE_6_BROADCAST_TOTAL, tot),
                ch, 1000, 0);
        }
    }

    // After knowing the totals, (re)advertise my TreeInfo to all physical neighbors
    sendTreeInfoToAllNeighbors();
}



void LeaderElecLab2BlockCode::Check() {
    if (subtreeSizeSent) return;

    // Ready to finalize if (a) I’m a leaf OR (b) all children reported,
    // AND there are no pending neighbour explorations.
    bool allChildrenDone = true;
    for (int i = 0; i < static_cast<int>(childrenSizes.size()); ++i) {
        if (childrenSizes[i] == 0) { allChildrenDone = false; break; }
    }
    const bool leaf = children.empty();
    const bool noPendingNeighbours = neighbours.empty();

    if ((leaf || allChildrenDone) && noPendingNeighbours) {
        if (!isLeader) {
            // Send my TOTAL upward once: includes me + all descendants.
            // when !isLeader:
            sendMessage("ReportSize",
                new MessageOf<SizeWeight>(TYPE_4_REPORT_SIZE,
                                          SizeWeight{subtree_size, subtree_weight}),
                parent, 1000, 0);


            console << "Module " << getId()
                    << " sent TOTAL (size="  << subtree_size
                    << ", weight="           << subtree_weight
                    << ") to parent "        << parent->getConnectedBlockId()
                    << "\n";

            subtreeSizeSent = true;
            return;
        }

        // === Leader: we have global totals; start ID + (implicit) color plan ===
        id_output << "Module " << getId() << " = 0\n"; id_output.flush();

        // Root tag derived from my id (stable & bounded by palette)
        int rootTag = (getId() % NB_COLORS);
        unsigned highRoot = (unsigned(rootTag) << SHIFT_ROOT);

        // Leader keeps only ROOT_TAG in its unique_id (branch=0, local=0)
        unique_id = (int)highRoot;
        module->setColor(Colors[rootTag % NB_COLORS]);

        sendTreeInfoToAllNeighbors();

        // Assign packed IDs to each child subtree: [ROOT|BRANCH|LOCAL]
        int next_local = 1; // local offset starts right after leader
        for (int i = 0; i < static_cast<int>(children.size()); ++i) {
            int branchTag = i + 1; // 1..#children
            unsigned highBits = highRoot | (unsigned(branchTag) << SHIFT_BRANCH);
            int childStartId = (int)(highBits | (unsigned)next_local);

            sendMessage("AssignId",
                new MessageOf<int>(TYPE_5_ASSIGN_ID, childStartId),
                children[i], 1000, 0);

            next_local += childrenSizes[i]; // skip this child's whole subtree span
        }

        console << "Leader " << getId()
                << " totals — size="   << subtree_size
                << ", weight="         << subtree_weight
                << ". ID assignment launched.\n";

        subtreeSizeSent = true; // prevent re-entry

        // After computing leader totals and before/after ID assignment as you prefer:
        SizeWeight totals{subtree_size, subtree_weight};
        broadcastTotalsToChildren(totals);

    }
}

bool LeaderElecLab2BlockCode::allChildrenReported() const {
    for (int sz : childrenSizes) if (sz == 0) return false;
    return true;
}
bool LeaderElecLab2BlockCode::isLeafNow() const {
    return children.empty() && neighbours.empty();
}


void LeaderElecLab2BlockCode::sendTreeInfoToAllNeighbors() {
    // Derive root tag as before
    int myRootTag = (unique_id >= 0) ? ((unique_id >> SHIFT_ROOT) & 0xFF)
                                     : (getId() % NB_COLORS);

    int myTreeDegree = (parent ? 1 : 0) + (int)children.size();
    int myIsLeaf     = children.empty() ? 1 : 0;

    TreeInfo info{
        myRootTag,
        myTreeDegree,
        myIsLeaf,
        global_total_size,
        global_total_weight,
        (global_total_size >= 0 && global_total_weight >= 0)
            ? (global_total_size + global_total_weight)
            : -1
    };

    // Broadcast to ALL physical neighbors (tree edge or not)
    for (int i = 0; i < module->getNbInterfaces(); ++i) {
        P2PNetworkInterface* ni = module->getInterface(i);
        if (ni && ni->isConnected()) {
            sendMessage("TreeInfo",
                new MessageOf<TreeInfo>(TYPE_7_TREE_INFO, info),
                ni, 1000, 0);
        }
    }
}


void LeaderElecLab2BlockCode::onTreeInfo(std::shared_ptr<Message> msg,
                                         P2PNetworkInterface* sender) {
    auto* m = static_cast<MessageOf<TreeInfo>*>(msg.get());
    const TreeInfo nbr = *(m->getData());

    int myRootTag = (unique_id >= 0) ? ((unique_id >> SHIFT_ROOT) & 0xFF)
                                     : (getId() % NB_COLORS);

    if (sender == parent) return;
    if (global_total_score < 0) return;
    if (nbr.totalScore     < 0) return;

    if (nbr.rootTag != myRootTag) {
        const int myScore    = global_total_score;
        const int theirScore = nbr.totalScore;

        // NEW: remember best competitor we have seen
        if (theirScore > local_max_other_score)
            local_max_other_score = theirScore;

        if (myScore > theirScore) {
            console << "Compare@Module " << getId()
                    << ": MY TREE bigger (" << myScore << " > " << theirScore
                    << ") vs neighbor " << sender->getConnectedBlockId()
                    << " (rootTag " << nbr.rootTag << ")\n";
        } else if (myScore < theirScore) {
            console << "Compare@Module " << getId()
                    << ": NEIGHBOR TREE bigger (" << theirScore << " > " << myScore
                    << ") at neighbor " << sender->getConnectedBlockId()
                    << " (rootTag " << nbr.rootTag << ")\n";
        } else {
            console << "Compare@Module " << getId()
                    << ": TREES EQUAL (" << myScore << ") with neighbor "
                    << sender->getConnectedBlockId()
                    << " (rootTag " << nbr.rootTag << ")\n";
        }
    }
}

void LeaderElecLab2BlockCode::maybeSendWinnerCandUp() {
    // Only non-roots send up; root evaluates directly
    if (!parent) { tryCrownIfRoot(); return; }

    // Send the best competitor score we've seen so far
    sendMessage("WinnerCandUp",
        new MessageOf<WinnerCand>(TYPE_8_WINNER_CAND,
                                  WinnerCand{local_max_other_score}),
        parent, 1000, 0);
}

void LeaderElecLab2BlockCode::onWinnerCand(std::shared_ptr<Message> msg,
                                           P2PNetworkInterface* sender) {
    auto* m = static_cast<MessageOf<WinnerCand>*>(msg.get());
    const WinnerCand wc = *(m->getData());
    if (wc.maxOtherScore > child_max_other_score) {
        child_max_other_score = wc.maxOtherScore;
    }

    if (parent) {
        int best = std::max(local_max_other_score, child_max_other_score);
        sendMessage("WinnerCandUp",
            new MessageOf<WinnerCand>(TYPE_8_WINNER_CAND, WinnerCand{best}),
            parent, 1000, 0);



        console << "[Elected the winner ]";
} else {
        // I'm the root — check if I can crown now
        tryCrownIfRoot();
    console << "[I am the winner ]";

    }
}

void LeaderElecLab2BlockCode::tryCrownIfRoot() {
    if (parent) return;                        // only root crowns
    if (isGlobalLeader) return;               // already crowned
    if (global_total_score < 0) return;       // I don't know my total yet

    int bestOther = std::max(local_max_other_score, child_max_other_score);

    int myRootTag = (unique_id >= 0) ? ((unique_id >> SHIFT_ROOT) & 0xFF)
                                     : (getId() % NB_COLORS);

    bool iWin = false;
    if (bestOther < 0) {
        // No competitors reported → I win by default
        iWin = true;
    } else if (global_total_score > bestOther) {
        iWin = true;
    } else if (global_total_score == bestOther) {
        // Optional tie-breaker by tag (or by getId())
        iWin = true; // or: iWin = (myRootTag < theirTagIfKnown); we don't know theirs → allow self-win
    }

    if (iWin) {
        isGlobalLeader = true;
        module->setColor(Colors[myRootTag % NB_COLORS]);

        console << "[ROOT " << getId() << "] GLOBAL LEADER elected. "
                << "score=" << global_total_score
                << " (bestOther=" << bestOther << ")\n";

        // Paint the whole grid with my color
        startGridPaint(myRootTag, myRootTag % NB_COLORS);
    }
}
void LeaderElecLab2BlockCode::startGridPaint(int leaderTag, int colorIdx) {
    int seq = ++gridPaintSeq;

    // I am the global leader root here.
    // Locally store winner info.
    globalLeaderKnown  = true;
    globalLeaderTag    = leaderTag;
    globalLeaderRootId = (int)getId();
    isInWinningTree    = true;  // root is obviously in the winning tree

    module->setColor(Colors[colorIdx % NB_COLORS]);
    paintSeenLeader = leaderTag;
    paintSeenSeq    = seq;

    // NEW: include leaderRootId in the payload
    GridPaint gp{leaderTag, colorIdx, seq, (int)getId()};

    for (int i = 0; i < module->getNbInterfaces(); ++i) {
        P2PNetworkInterface* ni = module->getInterface(i);
        if (ni && ni->isConnected()) {
            sendMessage("GridPaint",
                new MessageOf<GridPaint>(TYPE_10_GRID_COLOR, gp),
                ni, 1000, 0);
        }
    }
}


void LeaderElecLab2BlockCode::onGridPaint(std::shared_ptr<Message> msg,
                                          P2PNetworkInterface* sender) {
    auto* m = static_cast<MessageOf<GridPaint>*>(msg.get());
    const GridPaint gp = *(m->getData());

    // Deduplicate older/duplicate sequences
    if (paintSeenLeader == gp.leaderTag && paintSeenSeq >= gp.seq) return;

    paintSeenLeader = gp.leaderTag;
    paintSeenSeq    = gp.seq;
    module->setColor(Colors[gp.colorIdx % NB_COLORS]);

    // NEW: every node learns who the global winner root is
    globalLeaderKnown  = true;
    globalLeaderTag    = gp.leaderTag;
    globalLeaderRootId = gp.leaderRootId;
    isInWinningTree    = (myRootTag() == globalLeaderTag);

    // Existing winner-up logic (only winner tree forwards upward):
    maybeSendWinnerUp(gp.leaderTag, gp.seq);

    // Flood to all neighbors except sender
    for (int i = 0; i < module->getNbInterfaces(); ++i) {
        P2PNetworkInterface* ni = module->getInterface(i);
        if (ni && ni->isConnected() && ni != sender) {
            sendMessage("GridPaint",
                new MessageOf<GridPaint>(TYPE_10_GRID_COLOR, gp),
                ni, 1000, 0);
        }
    }
}

int LeaderElecLab2BlockCode::myRootTag() const {
    return (unique_id >= 0) ? ((unique_id >> SHIFT_ROOT) & 0xFF)
                            : (getId() % NB_COLORS);
}

void LeaderElecLab2BlockCode::maybeSendWinnerUp(int leaderTag, int seq) {
    // Remember what we know
    if (seq > winnerSeqKnown) {
        winnerSeqKnown = seq;
        winnerTagKnown = leaderTag;
    }

    // Only nodes that belong to the winner's tree forward
    if (myRootTag() != leaderTag) return;

    // Root doesn't forward up; it's the destination
    if (!parent) {

        console << "[WINNER] ";


        console << "[ROOT " << getId() << "] received/knows winnerTag=" << leaderTag
                << " (seq=" << seq << ")\n";
    }
else {
    console << "[ROOT " << getId() << "] received/knows winnerTag=" << leaderTag
                  << " (seq=" << seq << ")\n";

}
    // Deduplicate by sequence
    if (seq <= lastWinnerUpSeqSent) return;
    lastWinnerUpSeqSent = seq;

    sendMessage("WinnerUp",
        new MessageOf<WinnerUp>(TYPE_11_WINNER_UP, WinnerUp{leaderTag, seq, (int)getId()}),
        parent, 1000, 0);
}
void LeaderElecLab2BlockCode::onWinnerUp(std::shared_ptr<Message> msg,
                                         P2PNetworkInterface* sender) {
    auto* m = static_cast<MessageOf<WinnerUp>*>(msg.get());
    const WinnerUp wu = *(m->getData());

    // Only the winning tree relays this upward
    if (myRootTag() != wu.leaderTag) return;

    // Root consumes; everyone else relays once per seq
    if (!parent) {
        console << "[ROOT " << getId() << "] final WINNER-UP reached from child "
                << (sender ? sender->getConnectedBlockId() : -1)
                << " leaderTag=" << wu.leaderTag << " seq=" << wu.seq << "\n";
        return;
    }

    if (wu.seq <= lastWinnerUpSeqSent) return;
    lastWinnerUpSeqSent = wu.seq;

    sendMessage("WinnerUp",
        new MessageOf<WinnerUp>(TYPE_11_WINNER_UP, wu),
        parent, 1000, 0);
}
