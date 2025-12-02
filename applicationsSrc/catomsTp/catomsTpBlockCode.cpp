#include "catomsTpBlockCode.hpp"
#include <vector>
#include <queue>
#include <set>
#include <map>
#include <algorithm>
#include <unordered_set>

#include "robots/catoms3D/catoms3DMotionEngine.h"

using namespace Catoms3D;

static std::vector<Cell3DPosition> gPrecomputedPaths[2];
static bool gPathsComputed = false;


// Global multi-agent reservation table
struct ReservationTable {
    std::unordered_set<long long> occ;    // vertex-time
    std::unordered_set<unsigned long long> trans; // edge-time

    static inline long long enc_occ(const Cell3DPosition &p, int t) {
        return ((long long)p[0] << 40) | ((long long)p[1] << 20) | (unsigned long long)t;
    }
    static inline unsigned long long enc_trans(const Cell3DPosition &from, const Cell3DPosition &to, int t) {
        return ((unsigned long long)(from[0] & 0xFFF) << 44)
             | ((unsigned long long)(from[1] & 0xFFF) << 32)
             | ((unsigned long long)(to[0] & 0xFFF) << 20)
             | ((unsigned long long)(to[1] & 0xFFF) << 8)
             | (unsigned long long)t;
    }

    bool blocked_vertex(const Cell3DPosition &p, int t) const {
        return occ.count(enc_occ(p, t)) > 0;
    }
    bool blocked_swap(const Cell3DPosition &a, const Cell3DPosition &b, int t) const {
        return trans.count(enc_trans(b, a, t)) > 0;
    }

    void reserve_path(const std::vector<Cell3DPosition> &path) {
        for (int t = 0; t < (int)path.size(); ++t) {
            occ.insert(enc_occ(path[t], t));
            if (t > 0) trans.insert(enc_trans(path[t-1], path[t], t));
        }
        // extend goal reservation
        for (int pad = 1; pad <= 3; ++pad)
            occ.insert(enc_occ(path.back(), (int)path.size() - 1 + pad));
    }

    void saveToFile(const std::string &filename = "reservation_table.txt") const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "[ERROR] Cannot open " << filename << " for writing.\n";
        return;
    }

    file << "=== Vertex Reservations ===\n";
    for (const auto &v : occ) {
        int x = (v >> 40) & 0xFFFFF;
        int y = (v >> 20) & 0xFFFFF;
        int t = v & 0xFFFFF;
        file << "(" << x << "," << y << ") @t=" << t << "\n";
    }

    file << "\n=== Edge (Transition) Reservations ===\n";
    for (const auto &e : trans) {
        int fromX = (e >> 44) & 0xFFF;
        int fromY = (e >> 32) & 0xFFF;
        int toX   = (e >> 20) & 0xFFF;
        int toY   = (e >> 8)  & 0xFFF;
        int t     = e & 0xFF;
        file << "(" << fromX << "," << fromY << ") -> (" << toX << "," << toY << ") @t=" << t << "\n";
    }

    file.close();
    std::cout << "[LOG] Reservation table saved to " << filename << "\n";
}

};
static ReservationTable gReservations;



Cell3DPosition CatomsTpBlockCode::initiator  = Cell3DPosition(6, 4, 5);
Cell3DPosition CatomsTpBlockCode::initiator2  = Cell3DPosition(10, 3, 5);
Cell3DPosition CatomsTpBlockCode::initiator3  = Cell3DPosition(14, 4, 5);
Cell3DPosition CatomsTpBlockCode::initiator4  = Cell3DPosition(18, 3, 5);

Cell3DPosition CatomsTpBlockCode::Origin  = Cell3DPosition(4, 4, 2);
Cell3DPosition CatomsTpBlockCode::OriginB = Cell3DPosition(8, 4, 2);

static const std::array<Cell3DPosition, 12> FB_RELATIVE_POSITIONS = {
    Cell3DPosition(-2, -1, 3), Cell3DPosition(-1, -1, 2), Cell3DPosition(-2, -1, 1),
    Cell3DPosition(-1, -1, 3), Cell3DPosition(-1, -1, 1), Cell3DPosition(0, 0, 4),
    Cell3DPosition(0, 0, 0),   Cell3DPosition(1, 0, 4),   Cell3DPosition(1, 0, 0),
    Cell3DPosition(1, 0, 3),   Cell3DPosition(1, 0, 1),   Cell3DPosition(2, 1, 2)
};

// Test
static const std::array<Cell3DPosition, 12> BF_RELATIVE_POSITIONS = {
    Cell3DPosition(-2, 0, 3),  Cell3DPosition(-1, 1, 2), Cell3DPosition(-2, 0, 1),
    Cell3DPosition(-1, 0, 3),  Cell3DPosition(-1, 0, 1),  Cell3DPosition(0, 0, 4),
    Cell3DPosition(0, 0, 0),   Cell3DPosition(1, 0, 4),   Cell3DPosition(1, 0, 0),
    Cell3DPosition(1, -1, 3),  Cell3DPosition(1, -1, 1),  Cell3DPosition(2, -1, 2)
};

static std::array<Cell3DPosition, 12> gFBStartPositions;
static bool gFBStartsBuilt = false;

static std::array<Cell3DPosition, 12> gFBTargetPositions;
static bool gFBTargetsBuilt = false;



static std::array<Cell3DPosition, 12> gBFStartPositions;
static std::array<Cell3DPosition, 12> gBFTargetPositions;
static bool gBFStartsBuilt  = false;
static bool gBFTargetsBuilt = false;


static std::array<Cell3DPosition, 12> gFB8StartPositions;
static std::array<Cell3DPosition, 12> gFB8TargetPositions;
static bool gFB8StartsBuilt  = false;
static bool gFB8TargetsBuilt = false;


static std::array<Cell3DPosition, 12> gBF16StartPositions;
static std::array<Cell3DPosition, 12> gBF16TargetPositions;
static bool gBF16StartsBuilt  = false;
static bool gBF16TargetsBuilt = false;


static int gFirstWave = 0;
static int gReachedWave = 0;


// ---------------------------
// Constructor: register message handlers only
// ---------------------------
CatomsTpBlockCode::CatomsTpBlockCode(Catoms3DBlock *host) : Catoms3DBlockCode(host) {
    if (!host) return;

    addMessageEventFunc2(GO_ID,   std::bind(&CatomsTpBlockCode::handleGo,   this,
                                            std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(BACK_ID, std::bind(&CatomsTpBlockCode::handleBack, this,
                                            std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(FIRST_ID,
        std::bind(&CatomsTpBlockCode::handleFirst, this,
                  std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(REACHED_ID,
        std::bind(&CatomsTpBlockCode::handleReached, this,
                  std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(STALL_ID,
        std::bind(&CatomsTpBlockCode::handleStall, this,
                  std::placeholders::_1, std::placeholders::_2));



    module = static_cast<Catoms3DBlock*>(hostBlock);
}

// ---------------------------
// Minimal startup (no assignments, no coloring)


void CatomsTpBlockCode::startup() {
    int id = module->blockId;

    // Build FB start & target arrays once
    if (!gFBStartsBuilt) {
        for (size_t i = 0; i < FB_RELATIVE_POSITIONS.size(); ++i)
            gFBStartPositions[i] = Origin + FB_RELATIVE_POSITIONS[i];
        gFBStartsBuilt = true;
    }
    if (!gFBTargetsBuilt) {
        gFBTargetPositions = shiftPlus16X(gFBStartPositions);
        gFBTargetsBuilt = true;
    }

    // Only initiator computes both paths once globally
    if (module->position == initiator && !gPathsComputed) {
        console << "[INITIATOR] Computing both test paths...\n";

        gPrecomputedPaths[0] = findPath(gFBStartPositions[0], gFBTargetPositions[0]); // for 47
        gPrecomputedPaths[1] = findPath(gFBStartPositions[2], gFBTargetPositions[1]); // for 48

        if (!gPrecomputedPaths[0].empty()) {
            console << "[INITIATOR] Path 0 reserved for module 47 with "
                    << gPrecomputedPaths[0].size() << " steps.\n";
        }
        if (!gPrecomputedPaths[1].empty()) {
            console << "[INITIATOR] Path 1 reserved for module 48 with "
                    << gPrecomputedPaths[1].size() << " steps.\n";
        }

        gPathsComputed = true;

        // Send start signals
        CFirstPayload msg47{ gFBStartPositions[0], gFBTargetPositions[0], initiator, ++gFirstWave };
        CFirstPayload msg48{ gFBStartPositions[2], gFBTargetPositions[1], initiator, ++gFirstWave };

        sendMessageToAllNeighbors("FIRST_47", new MessageOf<CFirstPayload>(FIRST_ID, msg47), 1000, 100, 0);
        sendMessageToAllNeighbors("FIRST_48", new MessageOf<CFirstPayload>(FIRST_ID, msg48), 1000, 100, 0);
        return;
    }

    // Module 47 uses precomputed path 0
    if (id == 47 && gPathsComputed && !gPrecomputedPaths[0].empty()) {
        currentPath = gPrecomputedPaths[0];
        pathIdx = 0;
        hasPath = true;
        hasGoal = true;
        goalForThisMove = currentPath.back();
        console << "[MODULE 47] Using precomputed path.\n";
        scheduleGuardedHop();
        return;
    }

    // Module 48 uses precomputed path 1
    if (id == 48 && gPathsComputed && !gPrecomputedPaths[1].empty()) {
        currentPath = gPrecomputedPaths[1];
        pathIdx = 0;
        hasPath = true;
        hasGoal = true;
        goalForThisMove = currentPath.back();
        console << "[MODULE 48] Using precomputed path.\n";
        scheduleGuardedHop();
        return;
    }

    // Everyone else idle
    setColor(LIGHTGREY);
}



void CatomsTpBlockCode::broadcastFirstTo(const Cell3DPosition& target,
                                         const Cell3DPosition& goal) {
    // Allow all initiators (1–4)
    if (module->position != initiator &&
        module->position != initiator2 &&
        module->position != initiator3 &&
        module->position != initiator4)
        return;

    const Cell3DPosition originPos =
        (module->position == initiator2) ? initiator2 :
        (module->position == initiator3) ? initiator3 :
        (module->position == initiator4) ? initiator4 :
                                           initiator;

    CFirstPayload fp{ target, goal, originPos, ++gFirstWave };

    sendMessageToAllNeighbors(
        "FIRST_fwd",
        new MessageOf<CFirstPayload>(FIRST_ID, fp),
        1000, 100, 0
    );
}





void CatomsTpBlockCode::handleFirst(std::shared_ptr<Message> _msg, P2PNetworkInterface* sender) {
    auto* m  = static_cast<MessageOf<CFirstPayload>*>(_msg.get());
    CFirstPayload fp = *m->getData();

    if (module->position == fp.target) {
        myMoveOrigin = fp.origin;  // <— FB (initiator) or BF (initiator2)
        setColor(GREEN);

        Cell3DPosition start = module->position;
        Cell3DPosition goal  = fp.goal;

        goalForThisMove = goal;
        hasGoal = true;

       // auto path = findPath(start, goalForThisMove);
       // if (!path.empty()) {
         //   pathIdx = 0;
           // if (!currentPath.empty() && currentPath[0] == start) pathIdx = 1;

            //if (!scheduleGuardedHop()) {
              //  console << "[PLAN] Could not start movement safely. Waiting.\n";
               // hasPath = false;
           // }
        //} else {
         //   hasPath = false;
        //}

        //return;
    }

    if (fp.wave <= seenFirstWave) return;
    seenFirstWave = fp.wave;

    sendMessageToAllNeighbors("FIRST_fwd",
        new MessageOf<CFirstPayload>(FIRST_ID, fp), 1000, 100, 1, sender);
}



void CatomsTpBlockCode::handleReached(std::shared_ptr<Message> _msg, P2PNetworkInterface* sender) {
  auto* m = static_cast<MessageOf<CReachedPayload>*>(_msg.get());
CReachedPayload rp = *m->getData();

// FINAL ping (only consume at initiator2)
    // FINAL ping (consumed at rp.origin)
    if (rp.isFinal) {
        if (rp.wave <= seenFinalWave) return;
        seenFinalWave = rp.wave;

        if (module->position == rp.origin) {
            // If it’s initiator2: keep your current behavior (kick off BF)
            if (module->position == initiator2) {
                console << "[INITIATOR2] Received FB completion from " << rp.from << "!\n";
                setColor(BLACK);

                if (gBFStartsBuilt && gBFTargetsBuilt && !gBFStartPositions.empty()) {
                    console << "[INITIATOR2] Starting BF: "
                            << gBFStartPositions[0] << " → " << gBFTargetPositions[0] << "\n";
                    broadcastFirstTo(gBFStartPositions[0], gBFTargetPositions[0]);
                }
                return; // consumed at initiator2
            }

            // If it’s initiator3: just color it (your request)
            // If it’s initiator3: start its own FB+8X phase when BF is done
            // If it’s initiator3: start its own FB+8X phase when BF is done
            // If it’s initiator3: start its own FB+8X phase when BF is done
            // If it’s initiator3: start its own FB+8X phase when BF is done
            if (module->position == initiator3) {
                console << "[INITIATOR3] Received BF completion from " << rp.from << "!\n";
                setColor(CYAN);

                if (gFB8StartsBuilt && gFB8TargetsBuilt && !gFB8StartPositions.empty()) {
                    console << "[INITIATOR3] Starting FB+8X phase: "
                            << gFB8StartPositions[0] << " → " << gFB8TargetPositions[0] << "\n";
                    broadcastFirstTo(gFB8StartPositions[0], gFB8TargetPositions[0]);
                } else {
                    console << "[INITIATOR3] Warning: FB+8X arrays not built yet!\n";
                }
                return;
            }


            if (module->position == initiator4) {
                console << "[INITIATOR4] Received FB+8X completion from " << rp.from << "!\n";
                setColor(CYAN);

                // === Build BF+16X arrays ===
                if (!gBF16StartsBuilt) {
                    for (size_t i = 0; i < BF_RELATIVE_POSITIONS.size(); ++i)
                        gBF16StartPositions[i] = OriginB + BF_RELATIVE_POSITIONS[i] + Cell3DPosition(8, 0, 0);
                    gBF16StartsBuilt = true;

                    console << "[Initiator4][BF+16X Start] (OriginB + BF_RELATIVE_POSITIONS + 16X):\n";
                    for (size_t i = 0; i < gBF16StartPositions.size(); ++i)
                        console << "  [BF+16X Start] i=" << i << " -> " << gBF16StartPositions[i] << "\n";
                }

                if (!gBF16TargetsBuilt) {
                    for (size_t i = 0; i < gBF16StartPositions.size(); ++i)
                        gBF16TargetPositions[i] = gBF16StartPositions[i] + Cell3DPosition(16, 0, 0);
                    gBF16TargetsBuilt = true;

                    console << "[Initiator4][BF+16X Target] (+16X of BF+16X Start):\n";
                    for (size_t i = 0; i < gBF16TargetPositions.size(); ++i)
                        console << "  [BF+16X Target] i=" << i << " -> " << gBF16TargetPositions[i] << "\n";
                }

                // === Launch first candidate ===
                if (!gBF16StartPositions.empty()) {
                    console << "[INITIATOR4] Starting BF+16X phase: "
                            << gBF16StartPositions[0] << " → " << gBF16TargetPositions[0] << "\n";
                    broadcastFirstTo(gBF16StartPositions[0], gBF16TargetPositions[0]);
                } else {
                    console << "[INITIATOR4] No start positions found!\n";
                }
                return;
            }



            // (If you ever reuse FINAL for other destinations, handle them here.)
            return;
        }

        // Forward FINAL so it can reach its destination (rp.origin)
        sendMessageToAllNeighbors("REACHED_FINAL",
            new MessageOf<CReachedPayload>(REACHED_ID, rp), 1000, 100, 1, sender);
        return;
    }

// Normal REACHED (FB, BF, or FB+8X)
if (rp.wave <= seenReachedWave) return;
seenReachedWave = rp.wave;


    if (rp.wave <= seenReachedWave) return;
    seenReachedWave = rp.wave;

    if (module->position == rp.origin) {
        if (module->position == initiator || module->position == initiator2 ||
            module->position == initiator3 || module->position == initiator4) {

            console << "[INITIATOR] Received COLLISION alert from "
                    << rp.from << " at position " << rp.from << "\n";
            setColor(RED);
            return;
            }
    }


if (module->position == rp.origin) {
    console << "[REACHED] Module at " << rp.from << " reached its target.\n";
    setColor(ORANGE);

    nextPairIdx++;

    // Determine which phase is currently driving
    const bool drivingFB    = (rp.origin == initiator);
    const bool drivingBF    = (rp.origin == initiator2);
    const bool drivingFB8X  = (rp.origin == initiator3);
    const bool drivingBF16X = (rp.origin == initiator4);


    const auto& starts  = drivingFB    ? gFBStartPositions
                        : drivingBF    ? gBFStartPositions
                        : drivingFB8X  ? gFB8StartPositions
                        : drivingBF16X ? gBF16StartPositions
                        : gFBStartPositions;

    const auto& targets = drivingFB    ? gFBTargetPositions
                     : drivingBF    ? gBFTargetPositions
                     : drivingFB8X  ? gFB8TargetPositions
                     : drivingBF16X ? gBF16TargetPositions
                     : gFBTargetPositions;


    const char* phaseName = drivingFB    ? "FB"
                          : drivingBF    ? "BF"
                          : drivingFB8X  ? "FB+8X"
                          : drivingBF16X ? "BF+16X"
                          : "Unknown";


    if (nextPairIdx < static_cast<int>(starts.size())) {
        Cell3DPosition nextTarget = starts[nextPairIdx];
        Cell3DPosition nextGoal   = targets[nextPairIdx];
        console << "[" << phaseName << "] Launching next module " << nextPairIdx
                << " from " << nextTarget << " → " << nextGoal << "\n";
        broadcastFirstTo(nextTarget, nextGoal);
    } else {
        // Completed this phase
        console << "[" << phaseName << "] Phase complete.\n";
        setColor(PURPLE);
        gReachedWave += 100;

        if (drivingFB) {
            // Notify initiator2
            CReachedPayload finalMsg{ initiator2, module->position, gReachedWave, true };
            sendMessageToAllNeighbors("REACHED_FINAL",
                new MessageOf<CReachedPayload>(REACHED_ID, finalMsg), 1000, 100, 0);

        } else if (drivingBF) {
            // Notify initiator3
            CReachedPayload finalMsgToI3{ initiator3, module->position, gReachedWave, true };
            sendMessageToAllNeighbors("REACHED_FINAL",
                new MessageOf<CReachedPayload>(REACHED_ID, finalMsgToI3), 1000, 100, 0);

        } else if (drivingFB8X) {
            console << "[INITIATOR3] FB+8X phase completed!\n";
            setColor(GOLD);

            // Notify initiator4 to start BF+16X phase
            console << "[INITIATOR3] Sending FINAL to initiator4 (" << initiator4 << ")...\n";
            gReachedWave += 100;

            CReachedPayload finalMsgToI4{ initiator4, module->position, gReachedWave, true };
            sendMessageToAllNeighbors(
                "REACHED_FINAL",
                new MessageOf<CReachedPayload>(REACHED_ID, finalMsgToI4),
                1000, 100, 0);
        }

    }
    return;
}


// Relay normal REACHED along the network
sendMessageToAllNeighbors("REACHED_fwd",
    new MessageOf<CReachedPayload>(REACHED_ID, rp), 1000, 100, 1, sender);

}






// ---------------------------
// Message: GO (tree building for farthest-node selection)
// ---------------------------
void CatomsTpBlockCode::handleGo(std::shared_ptr<Message> _msg, P2PNetworkInterface* sender) {
    auto* m = static_cast<MessageOf<std::pair<int,int>>*>(_msg.get());
    auto [msgStage, msgDist] = *m->getData();

    if (msgStage > stage) { // new wave
        stage = msgStage;
        initFarthest();
    }

    if (parent == nullptr || msgDist < myDist) {
        parent = sender;
        myDist = msgDist;

        nbWaitedAns = sendMessageToAllNeighbors(
            "GO-fwd",
            new MessageOf<std::pair<int,int>>(GO_ID, {stage, myDist + 1}),
            1000, 100, 1, parent);

        if (nbWaitedAns == 0 && parent) {
            CBackPayload b{0, myDist, module->blockId};
            sendMessage("BACK_UP", new MessageOf<CBackPayload>(BACK_ID, b), parent, 100, 100);
        }
    } else {
        CBackPayload rej{0, myDist, module->blockId};
        sendMessage("BACK_reject", new MessageOf<CBackPayload>(BACK_ID, rej), sender, 100, 100);
    }
}

// ---------------------------
// Message: BACK (aggregation + select-down of the farthest)
// ---------------------------
void CatomsTpBlockCode::handleBack(std::shared_ptr<Message> _msg, P2PNetworkInterface* sender) {
    auto* m = static_cast<MessageOf<CBackPayload>*>(_msg.get());
    CBackPayload bp = *m->getData();

    // SELECT-DOWN propagation
    if (bp.kind == 1) {
        if (module->blockId == bp.id) {
            // This node is the selected farthest one.
            // If a path is already prepared, start executing it.
            if (hasPath && !currentPath.empty()) {
                const Cell3DPosition start = module->position;

                // Start from the next waypoint after current position if present
                pathIdx = 0;
                if (!currentPath.empty() && currentPath[0] == start) {
                    pathIdx = 1;
                } else {
                    for (size_t i = 0; i < currentPath.size(); ++i) {
                        if (currentPath[i] == start) { pathIdx = static_cast<int>(i) + 1; break; }
                    }
                }

                if (pathIdx < static_cast<int>(currentPath.size())) {
                    Cell3DPosition nextPos = currentPath[pathIdx];
                    if (!scheduleGuardedHop()) {
                        console << "[SELECT_DOWN] Infeasible next hop; aborting this move.\n";
                        hasPath = false;
                    }

                }
            }
        } else if (winnerChild) {
            sendMessage("SELECT_DOWN",
                        new MessageOf<CBackPayload>(BACK_ID, bp),
                        winnerChild, 100, 100);
        }
        return; // do not count against nbWaitedAns
    }

    // Regular BACK_UP from a child: update best candidate
    if (bp.dist > bestDist || (bp.dist == bestDist && bp.id < bestId)) {
        bestDist    = bp.dist;
        bestId      = bp.id;
        winnerChild = sender;
    }

    // Count children; when all replied, send result upward or start select-down at root
    nbWaitedAns--;
    if (nbWaitedAns == 0) {
        if (parent) {
            // Compare myself as candidate as well
            if (myDist > bestDist || (myDist == bestDist && module->blockId < bestId)) {
                bestDist    = myDist;
                bestId      = module->blockId;
                winnerChild = nullptr;
            }
            CBackPayload up{0, bestDist, bestId};
            sendMessage("BACK_UP", new MessageOf<CBackPayload>(BACK_ID, up), parent, 100, 100);
        } else {
            // Root: announce winner and start select-down
            if (winnerChild) {
                CBackPayload sel{1, bestDist, bestId};
                sendMessage("SELECT_DOWN",
                            new MessageOf<CBackPayload>(BACK_ID, sel),
                            winnerChild, 100, 100);
            } else {
                // Root itself is the winner (isolated case). If it already has a path, start it.
                if (hasPath && !currentPath.empty()) {
                    const Cell3DPosition start = module->position;
                    pathIdx = 0;
                    if (!currentPath.empty() && currentPath[0] == start) {
                        pathIdx = 1;
                    } else {
                        for (size_t i = 0; i < currentPath.size(); ++i) {
                            if (currentPath[i] == start) { pathIdx = static_cast<int>(i) + 1; break; }
                        }
                    }
                    if (pathIdx < static_cast<int>(currentPath.size())) {
                        Cell3DPosition nextPos = currentPath[pathIdx];
                        if (!scheduleGuardedHop()) {
                            console << "[SELECT_DOWN] Infeasible next hop; aborting this move.\n";
                            hasPath = false;
                        }

                    }
                }
            }
        }
    }
}

// ---------------------------
// Motion continuation: follow currentPath
// ---------------------------
void CatomsTpBlockCode::onMotionEnd() {
    if (!hasPath) return;

    pathIdx++;
    while (pathIdx < static_cast<int>(currentPath.size()) &&
           currentPath[pathIdx] == module->position) {
        pathIdx++;
    }

    if (pathIdx < static_cast<int>(currentPath.size())) {
        console << "[MOVE] Next step " << pathIdx << "/" << currentPath.size()
                << " → " << currentPath[pathIdx] << "\n";

        if (!scheduleGuardedHop()) {
            console << "[MOVE] Next hop infeasible even after replan. Aborting this move.\n";
            hasPath = false;
        }
        return;
    }


    // finished
    hasPath = false;
    console << "[MOVE] Reached final waypoint at " << module->position << "\n";

    // If this mover is NOT an initiator, report to whoever started my move (FB or BF)
    if (module->position != initiator && module->position != initiator2) {
        console << "[REACHED] Notifying origin " << myMoveOrigin << "...\n";
        CReachedPayload rp{ myMoveOrigin, module->position, ++gReachedWave, false };
        sendMessageToAllNeighbors(
            "REACHED_fwd", new MessageOf<CReachedPayload>(REACHED_ID, rp), 1000, 100, 0);
        return;
    }

    // Initiator itself moved (only happens in FB in your flow)
    console << "[ORIGIN] Reached my goal. Launching next module...\n";
    setColor(ORANGE);

    nextPairIdx++;
    if (nextPairIdx < static_cast<int>(gFBStartPositions.size())) {
        Cell3DPosition nextTarget = gFBStartPositions[nextPairIdx];
        Cell3DPosition nextGoal   = gFBTargetPositions[nextPairIdx];
        console << "[ORIGIN] Launching next module " << nextPairIdx
                << " from " << nextTarget << " → " << nextGoal << "\n";
        broadcastFirstTo(nextTarget, nextGoal);
        return;
    }

    // FB done — send FINAL to initiator2
    console << "[ORIGIN] All FB modules have completed their moves!\n";
    setColor(PURPLE);

    gReachedWave += 100;
    CReachedPayload finalMsg{ initiator2, module->position, gReachedWave, true };
    sendMessageToAllNeighbors(
        "REACHED_FINAL", new MessageOf<CReachedPayload>(REACHED_ID, finalMsg), 1000, 100, 0);
    console << "[ORIGIN] Sent FINAL completion message to initiator2\n";
}




// ---------------------------
// BFS pathfinding over motion rules
// ---------------------------
std::vector<Cell3DPosition>
CatomsTpBlockCode::findPath(Cell3DPosition &start, Cell3DPosition &goal) {
    static CAStar::Table globalTable; // shared between modules (priority planning)

    // Define lambda that returns all reachable neighbor positions
    auto neighborFunc = [&](const Cell3DPosition& p)->std::vector<Cell3DPosition>{
        std::vector<Cell3DPosition> neigh;
        getAllPossibleMotionsFromPosition(p, neigh);
        return neigh;
    };

    auto path = CAStar::plan(start, goal, neighborFunc, globalTable, 200);
    if(!path.empty())
        globalTable.reserve_path(path); // reserve for next module

    return path;
}



// ---------------------------
// Enumerate all reachable moves from a given position
// ---------------------------
bool CatomsTpBlockCode::getAllPossibleMotionsFromPosition(
    Cell3DPosition position,
    std::vector<Cell3DPosition> &reachablePositions) {

    Catoms3DBlock* mod = static_cast<Catoms3DBlock*>(lattice->getBlock(position));
    if (mod) {
        for (auto &neighPos: lattice->getFreeNeighborCells(position)) {
            if (mod->canMoveTo(neighPos)) {
                reachablePositions.push_back(neighPos);
            }
        }
        return !reachablePositions.empty();
    }

    bool found = false;
    for (auto &neighPos: lattice->getActiveNeighborCells(position)) {
        Catoms3DBlock* neigh = static_cast<Catoms3DBlock*>(lattice->getBlock(neighPos));
        std::vector<Catoms3DMotionRulesLink*> vec;
        Catoms3DMotionRules motionRulesInstance;
        short conFrom = neigh->getConnectorId(position);
        motionRulesInstance.getValidMotionListFromPivot(
            neigh, conFrom, vec, static_cast<FCCLattice*>(lattice), nullptr);
        for (auto link: vec) {
            Cell3DPosition toPos;
            neigh->getNeighborPos(link->getConToID(), toPos);
            reachablePositions.push_back(toPos);
            found = true;
        }
    }
    return found;
}




// ---------------------------
// Start a new farthest-node wave from this module
// ---------------------------
void CatomsTpBlockCode::startFarthestWaveFromHere() {
    if (farthestWaveStarted) return;
    farthestWaveStarted = true;

    stage++;
    initFarthest();
    myDist = 0;
    parent = nullptr;

    nbWaitedAns = sendMessageToAllNeighbors(
        "GO", new MessageOf<std::pair<int,int>>(GO_ID, {stage, 0}),
        1000, 100, 0);

    if (nbWaitedAns == 0) { // isolated root
        bestDist = myDist;
        bestId   = module->blockId;
    }
}



// ---------------------------
// Required virtuals (stubbed so the linker is happy)
// ---------------------------
bool CatomsTpBlockCode::parseUserCommandLineArgument(int &argc, char **argv[]) {
    (void)argc; (void)argv; // unused
    return false;
}

void CatomsTpBlockCode::processLocalEvent(EventPtr pev) {
    // Keep base behaviour; no extra local processing needed in pruned build
    Catoms3DBlockCode::processLocalEvent(pev);
}

void CatomsTpBlockCode::onBlockSelected() {
    // no-op
}

void CatomsTpBlockCode::onAssertTriggered() {
    // no-op
}

std::string CatomsTpBlockCode::onInterfaceDraw() {
    return std::string();
}
bool CatomsTpBlockCode::scheduleGuardedHop() {
    if (!hasPath || !hasGoal) return false;
    if (pathIdx >= static_cast<int>(currentPath.size())) return false;

    Cell3DPosition nextPos = currentPath[pathIdx];
    if (nextPos == module->position) {
        pathIdx++;
        if (pathIdx >= static_cast<int>(currentPath.size())) return false;
        nextPos = currentPath[pathIdx];
    }

    // (1) Runtime contention: destination currently occupied
    if (lattice->cellHasBlock(nextPos)) {
        console << "[STALL] Module " << module->blockId
                << " blocked: cell occupied " << nextPos << "\n";

        CStallPayload sp{ myMoveOrigin, module->position, nextPos, ++gReachedWave,
                          StallReason::OCCUPIED };
        sendMessageToAllNeighbors("STALL_notify",
            new MessageOf<CStallPayload>(STALL_ID, sp), 1000, 100, 0);

        setColor(RED);
        return false;
    }

    // (2) Motion infeasible now: no pivot/rotation for that hop
    if (!module->canMoveTo(nextPos)) {
        console << "[STALL] No pivot for " << module->position << " → " << nextPos << "\n";

        // Try a quick local replan once before notifying (optional but useful)
        auto replanned = findPath(module->position, goalForThisMove);
        if (!replanned.empty()) {
            console << "[REPLAN] New path with " << replanned.size() << " steps.\n";
            pathIdx = 0;
            // NOTE: currentPath already set in findPath
            // Retry scheduling:
            if (!currentPath.empty()) {
                Cell3DPosition np = currentPath[pathIdx];
                if (np == module->position && pathIdx+1 < (int)currentPath.size())
                    np = currentPath[++pathIdx];
                if (np != module->position && module->canMoveTo(np)) {
                    getScheduler()->schedule(
                        new Catoms3DRotationStartEvent(getScheduler()->now() + 100, module, np));
                    return true;
                }
            }
        }

        // Still can’t move → notify origin
        CStallPayload sp{ myMoveOrigin, module->position, nextPos, ++gReachedWave,
                          StallReason::NO_PIVOT };
        sendMessageToAllNeighbors("STALL_notify",
            new MessageOf<CStallPayload>(STALL_ID, sp), 1000, 100, 0);

        setColor(RED);
        return false;
    }

    console << "[MOVE] Module " << module->blockId << " rotating to " << nextPos << "\n";
    getScheduler()->schedule(
        new Catoms3DRotationStartEvent(getScheduler()->now() + 100, module, nextPos));
    return true;
}

void CatomsTpBlockCode::handleStall(std::shared_ptr<Message> _msg, P2PNetworkInterface* sender) {
    auto* m = static_cast<MessageOf<CStallPayload>*>(_msg.get());
    CStallPayload sp = *m->getData();

    if (sp.wave <= seenStallWave) return;
    seenStallWave = sp.wave;

    // If this is the initiator (origin), consume it
    if (module->position == sp.origin) {
        console << "[INITIATOR] STALL received from module at " << sp.from
                << " → wanted " << sp.want
                << " reason=" << (sp.reason == StallReason::OCCUPIED ? "OCCUPIED" :
                                   sp.reason == StallReason::NO_PIVOT ? "NO_PIVOT" : "UNKNOWN")
                << "\n";
        setColor(RED);
        return;
    }

    // Otherwise, forward along the network
    sendMessageToAllNeighbors("STALL_fwd",
        new MessageOf<CStallPayload>(STALL_ID, sp), 1000, 100, 1, sender);
}
