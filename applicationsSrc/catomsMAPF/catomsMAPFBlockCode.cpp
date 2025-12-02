#include "CatomsMAPFBlockCode.hpp"
#include "ReservationTable.h"

#include <vector>
#include <queue>
#include <set>
#include <map>
#include <algorithm>

#include "robots/catoms3D/catoms3DMotionEngine.h"

static ReservationTable gReservationTable;


using namespace Catoms3D;

static std::map<int, bool> gMoveInFlightPerModule;
static std::map<int, bool> gWaitingForRetry;  // moduleId  waiting

// Block ofDEBUG
// Replan instrumentation
static std::map<int,int> gReplanCount;   // moduleId -> #replans
static std::set<int>     gReplanOnce;    // moduleIds that replanned at least once

static inline void noteReplan(int moduleId) {
    gReplanCount[moduleId]++;
    gReplanOnce.insert(moduleId);
}

static inline void dumpReplanModulesToFile(const std::string& filename = "replan_modules.txt") {
    std::ofstream out(filename, std::ios::trunc);
    if (!out.is_open()) return;
    out << "# moduleIds that triggered a replan (one line per module)\n";
    for (int id : gReplanOnce) out << id << "\n";
    out.close();
}

static inline void dumpReplanStatsToFile(const std::string& filename = "replan_stats.txt") {
    std::ofstream out(filename, std::ios::trunc);
    if (!out.is_open()) return;
    out << "# moduleId replanCount\n";
    for (auto& kv : gReplanCount) out << kv.first << " " << kv.second << "\n";
    out.close();
}

// ---- Plan logging -----------------------------------------------------------
static inline void resetPlansFile(const std::string& filename = "plans.log") {
    std::ofstream out(filename, std::ios::trunc);
    // empty file (truncate)
}

static inline void appendPlanToFile(int moduleId,
                                    const Cell3DPosition& moveOrigin,  // who ordered me
                                    const Cell3DPosition& start,
                                    const Cell3DPosition& goal,
                                    const std::vector<Cell3DPosition>& path,
                                    int baseTime,
                                    const std::string& filename = "plans.log",
                                    bool isReplan = false) {
    std::ofstream out(filename, std::ios::app);
    if (!out.is_open()) return;

    out << "================ PLAN =================\n";
    out << "moduleId=" << moduleId
        << "  origin=" << moveOrigin
        << "  start="  << start
        << "  goal="   << goal
        << "  baseTime=" << baseTime
        << (isReplan ? "  [REPLAN]\n" : "\n");

    if (path.empty()) {
        out << "  (empty path)\n\n";
        return;
    }

    // Print step-by-step timeline with absolute time and rotation hop
    int t = baseTime;
    Cell3DPosition prev = start;
    for (const auto& step : path) {
        ++t;
        out << "  t=" << t << " : " << prev << " -> " << step << "\n";
        prev = step;
    }
    out << "\n";
}

// Block ofDEBUG

//static bool gMoveInFlight = false;
// parallel launch window (per origin)
static constexpr int kParallel = 2; // move at most 2 modules concurrently

// Track per-origin state
static std::map<Cell3DPosition,int> gInFlightByOrigin;   // how many currently moving
static std::map<Cell3DPosition,int> gNextToLaunchByOrigin; // next index to launch
static std::map<Cell3DPosition,int> gCompletedByOrigin;  // how many finished


static std::set<Cell3DPosition> gReservedCells;

static void launchWindowFromOrigin(
    CatomsMAPFBlockCode* self,
    const Cell3DPosition& origin,
    const std::vector<Cell3DPosition>& starts,
    const std::vector<Cell3DPosition>& targets)
{
    int& inFlight = gInFlightByOrigin[origin];
    int& nextIdx  = gNextToLaunchByOrigin[origin];

    while (inFlight < kParallel && nextIdx < (int)starts.size()) {
        self->console << "[WINDOW] Launching idx=" << nextIdx
                      << " from " << starts[nextIdx] << " → " << targets[nextIdx] << "\n";
        self->broadcastFirstTo(starts[nextIdx], targets[nextIdx]);
        ++inFlight;
        ++nextIdx;
    }
}
static std::vector<Cell3DPosition> vecFrom(const std::array<Cell3DPosition,12>& a) {
    return std::vector<Cell3DPosition>(a.begin(), a.end());
}


static std::map<int, Cell3DPosition> gLastDest;


Cell3DPosition CatomsMAPFBlockCode::initiator  = Cell3DPosition(6, 4, 5);
Cell3DPosition CatomsMAPFBlockCode::initiator2  = Cell3DPosition(10, 3, 5);
Cell3DPosition CatomsMAPFBlockCode::initiator3  = Cell3DPosition(14, 4, 5);
Cell3DPosition CatomsMAPFBlockCode::initiator4  = Cell3DPosition(18, 3, 5);

Cell3DPosition CatomsMAPFBlockCode::Origin  = Cell3DPosition(4, 4, 2);
Cell3DPosition CatomsMAPFBlockCode::OriginB = Cell3DPosition(8, 4, 2);

static const std::array<Cell3DPosition, 12> FB_RELATIVE_POSITIONS = {
    Cell3DPosition(-2, -1, 3),  Cell3DPosition(-2, -1, 1),Cell3DPosition(-1, -1, 2),
    Cell3DPosition(-1, -1, 3), Cell3DPosition(-1, -1, 1), Cell3DPosition(0, 0, 4),
    Cell3DPosition(0, 0, 0),   Cell3DPosition(1, 0, 4),   Cell3DPosition(1, 0, 0),
    Cell3DPosition(1, 0, 3),   Cell3DPosition(1, 0, 1),   Cell3DPosition(2, 1, 2)
};

// Test
static const std::array<Cell3DPosition, 12> BF_RELATIVE_POSITIONS = {
    Cell3DPosition(-2, 0, 3),   Cell3DPosition(-2, 0, 1),Cell3DPosition(-1, 1, 2),
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


CatomsMAPFBlockCode::CatomsMAPFBlockCode(Catoms3DBlock *host) : Catoms3DBlockCode(host) {
    if (!host) return;

    addMessageEventFunc2(GO_ID,   std::bind(&CatomsMAPFBlockCode::handleGo,   this,
                                            std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(BACK_ID, std::bind(&CatomsMAPFBlockCode::handleBack, this,
                                            std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(FIRST_ID,
        std::bind(&CatomsMAPFBlockCode::handleFirst, this,
                  std::placeholders::_1, std::placeholders::_2));
    addMessageEventFunc2(REACHED_ID,
        std::bind(&CatomsMAPFBlockCode::handleReached, this,
                  std::placeholders::_1, std::placeholders::_2));



    module = static_cast<Catoms3DBlock*>(hostBlock);
}




void CatomsMAPFBlockCode::startup() {
    if (module->position == initiator) {
        if (!gFBStartsBuilt) {
            resetPlansFile();
            for (size_t i = 0; i < FB_RELATIVE_POSITIONS.size(); ++i)
                gFBStartPositions[i] = Origin + FB_RELATIVE_POSITIONS[i];
            gFBStartsBuilt = true;

            for (size_t i = 0; i < gFBStartPositions.size(); ++i)
                console << "[FBStart] i=" << i << " -> " << gFBStartPositions[i] << "\n";
        }

        if (!gFBTargetsBuilt) {
gFBTargetPositions = shiftPlus16X(gFBStartPositions);
            gFBTargetsBuilt = true;

            for (size_t i = 0; i < gFBTargetPositions.size(); ++i)
                console << "[FBTarget] i=" << i << " -> " << gFBTargetPositions[i] << "  (+16X)\n";
        }

        setColor(RED);
        gInFlightByOrigin[initiator]   = 0;
        gNextToLaunchByOrigin[initiator] = 0;
        gCompletedByOrigin[initiator]  = 0;

        auto FBstarts  = vecFrom(gFBStartPositions);
        auto FBtargets = vecFrom(gFBTargetPositions);
        launchWindowFromOrigin(this, initiator, FBstarts, FBtargets);

        return;
    }


    if (module->position == initiator2) {
        // Build BF starts from OriginB + BF_RELATIVE_POSITIONS
        if (!gBFStartsBuilt) {
            for (size_t i = 0; i < BF_RELATIVE_POSITIONS.size(); ++i)
                gBFStartPositions[i] = OriginB + BF_RELATIVE_POSITIONS[i];
            gBFStartsBuilt = true;

            console << "\n[Initiator2][BFStart] Absolute positions (OriginB + BF_RELATIVE_POSITIONS):\n";
            for (size_t i = 0; i < gBFStartPositions.size(); ++i)
                console << "  [BFStart] i=" << i << " -> " << gBFStartPositions[i] << "\n";
        }

        // Build BF targets as +16X shift of starts
        if (!gBFTargetsBuilt) {
gBFTargetPositions = shiftPlus16X(gBFStartPositions);
            gBFTargetsBuilt = true;

            console << "\n[Initiator2][BFTarget] (+16X of BFStart):\n";
            for (size_t i = 0; i < gBFTargetPositions.size(); ++i)
                console << "  [BFTarget] i=" << i << " -> " << gBFTargetPositions[i] << "\n";
        }

        setColor(PINK);
        return;
    }
    if (module->position == initiator3) {
        // Build FB+8X starts from Origin + FB_RELATIVE_POSITIONS + (8,0,0)
        if (!gFB8StartsBuilt) {
            for (size_t i = 0; i < FB_RELATIVE_POSITIONS.size(); ++i)
                gFB8StartPositions[i] = Origin + FB_RELATIVE_POSITIONS[i] + Cell3DPosition(8, 0, 0);
            gFB8StartsBuilt = true;

            console << "\n[Initiator3][FB+8X Start] (Origin + FB_RELATIVE_POSITIONS + 8X):\n";
            for (size_t i = 0; i < gFB8StartPositions.size(); ++i)
                console << "  [FB+8X Start] i=" << i << " -> " << gFB8StartPositions[i] << "\n";
        }

        if (!gFB8TargetsBuilt) {
            for (size_t i = 0; i < gFB8StartPositions.size(); ++i)
                gFB8TargetPositions[i] = gFB8StartPositions[i] + Cell3DPosition(16, 0, 0);
            gFB8TargetsBuilt = true;

            console << "\n[Initiator3][FB+8X Target] (+16X of FB+8X Start):\n";
            for (size_t i = 0; i < gFB8TargetPositions.size(); ++i)
                console << "  [FB+8X Target] i=" << i << " -> " << gFB8TargetPositions[i] << "\n";
        }

        setColor(LIGHTGREEN);
        return;
    }




    if (module->position == Origin) {
        setColor(YELLOW);
        return;
    }

    if (module->position == OriginB) {
        setColor(BLUE);
        return;
    }

    setColor(LIGHTGREY);
}


void CatomsMAPFBlockCode::broadcastFirstTo(const Cell3DPosition& target,
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





void CatomsMAPFBlockCode::handleFirst(std::shared_ptr<Message> _msg, P2PNetworkInterface* sender) {
    auto* m  = static_cast<MessageOf<CFirstPayload>*>(_msg.get());
    CFirstPayload fp = *m->getData();

    // ignore non-initiator4 origins during reverse
    // if (gReverseActive && fp.origin != initiator4) {
    //     if (fp.wave > seenFirstWave) {
    //         sendMessageToAllNeighbors("FIRST_fwd",
    //             new MessageOf<CFirstPayload>(FIRST_ID, fp), 1000, 100, 1, sender);
    //     }
    //     return;
    // }

    // If Im the intended starter of this, recompute a fresh path
    if (module->position == fp.target) {
        currentPath.clear();
        hasPath = false;

        myMoveOrigin = fp.origin;
        setColor(GREEN);

        Cell3DPosition start = module->position;
        Cell3DPosition goal  = fp.goal;

        goalForThisMove = goal;
        hasGoal = true;

        auto path = findPath(start, goalForThisMove);
        if (!path.empty()) {
            pathIdx = 0;
            if (!scheduleGuardedHop()) {
                // We have a valid path; a hop just couldn’t be scheduled *now*.
                // Keep the path and mark that we’re waiting to retry.
                console << "[PLAN] Could not start movement safely. Will retry.\n";
                gWaitingForRetry[module->blockId] = true;
                // do NOT set hasPath=false
            }
        } else {
            console << "[PLAN] No path found from " << start << " to " << goalForThisMove << ".\n";
            hasPath = false; // only clear if no path exists at all
        }

        return;
    }

    // Wave guard + forward
    if (fp.wave <= seenFirstWave) return;
    seenFirstWave = fp.wave;

    sendMessageToAllNeighbors("FIRST_fwd",
        new MessageOf<CFirstPayload>(FIRST_ID, fp), 1000, 100, 1, sender);
}




void CatomsMAPFBlockCode::handleReached(std::shared_ptr<Message> _msg, P2PNetworkInterface* sender) {
    auto* m  = static_cast<MessageOf<CReachedPayload>*>(_msg.get());
    CReachedPayload rp = *m->getData();

    // -----------------------------
    // FINAL (phase-complete) message
    // -----------------------------
    if (rp.isFinal) {
        if (rp.wave <= seenFinalWave) return;
        seenFinalWave = rp.wave;

        // During REVERSE, ignore finals not destined to initiator4 (but keep forwarding)
        // if (gReverseActive && rp.origin != initiator4) {
        //     sendMessageToAllNeighbors("REACHED_fwd",
        //         new MessageOf<CReachedPayload>(REACHED_ID, rp), 1000, 100, 1, sender);
        //     return;
        // }

        if (module->position == rp.origin) {
            // --- Start next phase with a fresh 2-slot window ---
            if (module->position == initiator2) {
                console << "[INITIATOR2] Received FB completion from " << rp.from << "!\n";
                setColor(PURPLE);
                if (gBFStartsBuilt && gBFTargetsBuilt && !gBFStartPositions.empty()) {
                    gInFlightByOrigin[initiator2]     = 0;
                    gNextToLaunchByOrigin[initiator2] = 0;
                    gCompletedByOrigin[initiator2]    = 0;
                    auto BFstarts  = vecFrom(gBFStartPositions);
                    auto BFtargets = vecFrom(gBFTargetPositions);
                    launchWindowFromOrigin(this, initiator2, BFstarts, BFtargets);
                }
                return;
            }

            if (module->position == initiator3) {
                console << "[INITIATOR3] Received BF completion from " << rp.from << "!\n";
                setColor(PURPLE);
                if (gFB8StartsBuilt && gFB8TargetsBuilt && !gFB8StartPositions.empty()) {
                    gInFlightByOrigin[initiator3]     = 0;
                    gNextToLaunchByOrigin[initiator3] = 0;
                    gCompletedByOrigin[initiator3]    = 0;
                    auto FB8starts  = vecFrom(gFB8StartPositions);
                    auto FB8targets = vecFrom(gFB8TargetPositions);
                    launchWindowFromOrigin(this, initiator3, FB8starts, FB8targets);
                } else {
                    console << "[INITIATOR3] Warning: FB+8X arrays not built yet!\n";
                }
                return;
            }

            if (module->position == initiator4) {
                console << "[INITIATOR4] Received FB+8X completion from " << rp.from << "!\n";
                setColor(CYAN);

                // Build BF+16X once
                if (!gBF16StartsBuilt) {
                    for (size_t i = 0; i < BF_RELATIVE_POSITIONS.size(); ++i)
                        gBF16StartPositions[i] = OriginB + BF_RELATIVE_POSITIONS[i] + Cell3DPosition(8,0,0);
                    gBF16StartsBuilt = true;

                    console << "[Initiator4][BF+16X Start]:\n";
                    for (size_t i = 0; i < gBF16StartPositions.size(); ++i)
                        console << "  i=" << i << " -> " << gBF16StartPositions[i] << "\n";
                }
                if (!gBF16TargetsBuilt) {
                    for (size_t i = 0; i < gBF16StartPositions.size(); ++i)
                        gBF16TargetPositions[i] = gBF16StartPositions[i] + Cell3DPosition(16,0,0);
                    gBF16TargetsBuilt = true;

                    console << "[Initiator4][BF+16X Target]:\n";
                    for (size_t i = 0; i < gBF16TargetPositions.size(); ++i)
                        console << "  i=" << i << " -> " << gBF16TargetPositions[i] << "\n";
                }

                if (!gBF16StartPositions.empty()) {
                    gInFlightByOrigin[initiator4]     = 0;
                    gNextToLaunchByOrigin[initiator4] = 0;
                    gCompletedByOrigin[initiator4]    = 0;
                    auto BF16starts  = vecFrom(gBF16StartPositions);
                    auto BF16targets = vecFrom(gBF16TargetPositions);
                    launchWindowFromOrigin(this, initiator4, BF16starts, BF16targets);
                } else {
                    console << "[INITIATOR4] No BF+16X start positions found!\n";
                }
                return;
            }
        }

        // Forward FINAL so it can reach its origin
        sendMessageToAllNeighbors("REACHED_FINAL",
            new MessageOf<CReachedPayload>(REACHED_ID, rp), 1000, 100, 1, sender);
        return;
    }

    // -----------------------------
    // Normal REACHED (one mover done)
    // -----------------------------
    if (rp.wave <= seenReachedWave) return;
    seenReachedWave = rp.wave;

    // Only the origin of the phase manages the window
    if (module->position == rp.origin) {
        console << "[REACHED] Module at " << rp.from << " reached target.\n";
        setColor(ORANGE);

        // Identify phase
        const bool drivingFB    = (rp.origin == initiator);
        const bool drivingBF    = (rp.origin == initiator2);
        const bool drivingFB8X  = (rp.origin == initiator3);
        const bool drivingBF16X = (rp.origin == initiator4);
        // const bool reverse      = (gReverseActive && drivingBF16X);

        // Convert arrays → vectors
        std::vector<Cell3DPosition> starts, targets;
        // if (reverse) {
        //     starts  = vecFrom(gBF16TargetPositions);
        //     targets = vecFrom(gBF16StartPositions);
        // }
 if (drivingFB) {
            starts  = vecFrom(gFBStartPositions);
            targets = vecFrom(gFBTargetPositions);
        } else if (drivingBF) {
            starts  = vecFrom(gBFStartPositions);
            targets = vecFrom(gBFTargetPositions);
        } else if (drivingFB8X) {
            starts  = vecFrom(gFB8StartPositions);
            targets = vecFrom(gFB8TargetPositions);
        } else { // BF+16X
            starts  = vecFrom(gBF16StartPositions);
            targets = vecFrom(gBF16TargetPositions);
        }

        // Update window counters
        gCompletedByOrigin[rp.origin] += 1;
        if (gInFlightByOrigin[rp.origin] > 0) gInFlightByOrigin[rp.origin] -= 1;

        // Refill window
        launchWindowFromOrigin(this, rp.origin, starts, targets);

        // Phase completion?
        const int totalNeeded = static_cast<int>(starts.size());
        if (gCompletedByOrigin[rp.origin] >= totalNeeded) {
            // --- PHASE COMPLETE: chain to next or start reverse ---
            setColor(PURPLE);
            console << "[PHASE] " << ( drivingFB    ? "FB" :
                         drivingBF    ? "BF" :
                         drivingFB8X  ? "FB+8X" : "BF+16X")
                     << " phase complete.\n";

            gReachedWave += 100;

            dumpReplanModulesToFile();  // writes replan_modules.txt
            dumpReplanStatsToFile();    // writes replan_stats.txt
            if (drivingFB) {
                CReachedPayload finalToI2{ initiator2, module->position, gReachedWave, true };
                sendMessageToAllNeighbors("REACHED_FINAL",
                    new MessageOf<CReachedPayload>(REACHED_ID, finalToI2), 1000, 100, 0);
            } else if (drivingBF ) {
                CReachedPayload finalToI3{ initiator3, module->position, gReachedWave, true };
                sendMessageToAllNeighbors("REACHED_FINAL",
                    new MessageOf<CReachedPayload>(REACHED_ID, finalToI3), 1000, 100, 0);
            } else if (drivingFB8X) {
                CReachedPayload finalToI4{ initiator4, module->position, gReachedWave, true };
                sendMessageToAllNeighbors("REACHED_FINAL",
                    new MessageOf<CReachedPayload>(REACHED_ID, finalToI4), 1000, 100, 0);
            } else {
                // BF+16X completed → start REVERSE
                console << "\n[GLOBAL] All 4 phases done. Starting REVERSE (windowed).\n";

                // Reverse arrays so farthest return first
          //     std::reverse(gBF16StartPositions.begin(),  gBF16StartPositions.end());
            //    std::reverse(gBF16TargetPositions.begin(), gBF16TargetPositions.end());

                // gReverseActive = true;

                // Clear reservations / state
                gMoveInFlightPerModule[module->blockId] = false;
                gReservedCells.clear();
                gLastDest.clear();

                // Isolate reverse messages with new waves
                gFirstWave      += 1000;
                gReachedWave    += 1000;
                seenFirstWave    = gFirstWave;
                seenReachedWave  = gReachedWave;

                // Reset any per-module motion state at origin
                currentPath.clear();
                hasPath = false;
                hasGoal = false;

                // Init reverse window (initiator4 is the reverse origin)
                gInFlightByOrigin[initiator4]     = 0;
                gNextToLaunchByOrigin[initiator4] = 0;
                gCompletedByOrigin[initiator4]    = 0;

                auto Rstarts  = vecFrom(gBF16TargetPositions); // reversed order already applied
                auto Rtargets = vecFrom(gBF16StartPositions);
                launchWindowFromOrigin(this, initiator4, Rstarts, Rtargets);
            }
        }

        return;
    }

    // Not the origin → relay the REACHED so it can propagate to the origin
    sendMessageToAllNeighbors("REACHED_fwd",
        new MessageOf<CReachedPayload>(REACHED_ID, rp), 1000, 100, 1, sender);
}






void CatomsMAPFBlockCode::handleGo(std::shared_ptr<Message> _msg, P2PNetworkInterface* sender) {
    // auto* m = static_cast<MessageOf<std::pair<int,int>>*>(_msg.get());
    // auto [msgStage, msgDist] = *m->getData();
    //
    // if (msgStage > stage) { // new wave
    //     stage = msgStage;
    //     initFarthest();
    // }
    //
    // if (parent == nullptr || msgDist < myDist) {
    //     parent = sender;
    //     myDist = msgDist;
    //
    //     nbWaitedAns = sendMessageToAllNeighbors(
    //         "GO-fwd",
    //         new MessageOf<std::pair<int,int>>(GO_ID, {stage, myDist + 1}),
    //         1000, 100, 1, parent);
    //
    //     if (nbWaitedAns == 0 && parent) {
    //         CBackPayload b{0, myDist, module->blockId};
    //         sendMessage("BACK_UP", new MessageOf<CBackPayload>(BACK_ID, b), parent, 100, 100);
    //     }
    // } else {
    //     CBackPayload rej{0, myDist, module->blockId};
    //     sendMessage("BACK_reject", new MessageOf<CBackPayload>(BACK_ID, rej), sender, 100, 100);
    // }
}


void CatomsMAPFBlockCode::handleBack(std::shared_ptr<Message> _msg, P2PNetworkInterface* sender) {
    // auto* m = static_cast<MessageOf<CBackPayload>*>(_msg.get());
    // CBackPayload bp = *m->getData();
    //
    // // SELECT-DOWN propagation
    // if (bp.kind == 1) {
    //     if (module->blockId == bp.id) {
    //         // This node is the selected farthest one.
    //         // If a path is already prepared, start executing it.
    //         if (hasPath && !currentPath.empty()) {
    //             const Cell3DPosition start = module->position;
    //
    //             // Start from the next waypoint after current position if present
    //             pathIdx = 0;
    //             if (!currentPath.empty() && currentPath[0] == start) {
    //                 pathIdx = 1;
    //             } else {
    //                 for (size_t i = 0; i < currentPath.size(); ++i) {
    //                     if (currentPath[i] == start) { pathIdx = static_cast<int>(i) + 1; break; }
    //                 }
    //             }
    //
    //             if (pathIdx < static_cast<int>(currentPath.size())) {
    //                 Cell3DPosition nextPos = currentPath[pathIdx];
    //                 if (!scheduleGuardedHop()) {
    //                     console << "[SELECT_DOWN] Infeasible next hop; aborting this move.\n";
    //                     hasPath = false;
    //                 }
    //
    //             }
    //         }
    //     } else if (winnerChild) {
    //         sendMessage("SELECT_DOWN",
    //                     new MessageOf<CBackPayload>(BACK_ID, bp),
    //                     winnerChild, 100, 100);
    //     }
    //     return; // do not count against nbWaitedAns
    // }
    //
    // // Regular BACK_UP from a child: update best candidate
    // if (bp.dist > bestDist || (bp.dist == bestDist && bp.id < bestId)) {
    //     bestDist    = bp.dist;
    //     bestId      = bp.id;
    //     winnerChild = sender;
    // }
    //
    // // Count children; when all replied, send result upward or start select-down at root
    // nbWaitedAns--;
    // if (nbWaitedAns == 0) {
    //     if (parent) {
    //         // Compare myself as candidate as well
    //         if (myDist > bestDist || (myDist == bestDist && module->blockId < bestId)) {
    //             bestDist    = myDist;
    //             bestId      = module->blockId;
    //             winnerChild = nullptr;
    //         }
    //         CBackPayload up{0, bestDist, bestId};
    //         sendMessage("BACK_UP", new MessageOf<CBackPayload>(BACK_ID, up), parent, 100, 100);
    //     } else {
    //         // Root: announce winner and start select-down
    //         if (winnerChild) {
    //             CBackPayload sel{1, bestDist, bestId};
    //             sendMessage("SELECT_DOWN",
    //                         new MessageOf<CBackPayload>(BACK_ID, sel),
    //                         winnerChild, 100, 100);
    //         } else {
    //             // Root itself is the winner (isolated case). If it already has a path, start it.
    //             if (hasPath && !currentPath.empty()) {
    //                 const Cell3DPosition start = module->position;
    //                 pathIdx = 0;
    //                 if (!currentPath.empty() && currentPath[0] == start) {
    //                     pathIdx = 1;
    //                 } else {
    //                     for (size_t i = 0; i < currentPath.size(); ++i) {
    //                         if (currentPath[i] == start) { pathIdx = static_cast<int>(i) + 1; break; }
    //                     }
    //                 }
    //                 if (pathIdx < static_cast<int>(currentPath.size())) {
    //                     Cell3DPosition nextPos = currentPath[pathIdx];
    //                     if (!scheduleGuardedHop()) {
    //                         console << "[SELECT_DOWN] Infeasible next hop; aborting this move.\n";
    //                         hasPath = false;
    //                     }
    //
    //                 }
    //             }
    //         }
    //     }
    // }
}


void CatomsMAPFBlockCode::onMotionEnd() {
    // Mark this module as free to schedule another hop
    gMoveInFlightPerModule[module->blockId] = false;
    gWaitingForRetry[module->blockId] = false; // we’ll immediately try next step

    // Release the destination cell we had reserved for this hop
    auto it = gLastDest.find(module->blockId);
    if (it != gLastDest.end()) {
        gReservedCells.erase(it->second);
        gLastDest.erase(it);
    }

    // If no active path, nothing to continue
    if (!hasPath || currentPath.empty()) {
        console << "[MOVE] No stored path for module " << module->blockId << " — stopping.\n";
        hasPath = false;
        return;
    }

    // Advance to next waypoint (skip a rare duplicate of current cell once)
    pathIdx++;
    if (pathIdx < static_cast<int>(currentPath.size()) &&
        currentPath[pathIdx] == module->position) {
        pathIdx++;
    }

    // If there are waypoints left, try to schedule the next hop
    if (pathIdx < static_cast<int>(currentPath.size())) {
        console << "[MOVE] Module " << module->blockId
                << " continuing to step " << pathIdx
                << "/" << currentPath.size() << " → "
                << currentPath[pathIdx] << "\n";

        if (!scheduleGuardedHop()) {
            console << "[MOVE] Could not schedule next hop; waiting for progress.\n";
            // scheduleGuardedHop() will set gWaitingForRetry[moduleId] = true when blocked
        }
        return;
    }

    // Path fully completed
    console << "[MOVE] Module " << module->blockId
            << " reached final waypoint at " << module->position << "\n";
    hasPath = false;


    if ((module->blockId % 5) == 0) {
        dumpReplanModulesToFile();
        dumpReplanStatsToFile();
    }

    // If this mover is NOT an initiator, report completion to its origin
    if (module->position != initiator && module->position != initiator2) {
        console << "[REACHED] Notifying origin " << myMoveOrigin << "...\n";
        CReachedPayload rp{ myMoveOrigin, module->position, ++gReachedWave, false };
        sendMessageToAllNeighbors(
            "REACHED_fwd",
            new MessageOf<CReachedPayload>(REACHED_ID, rp),
            1000, 100, 0);
        return;
    }

    // Initiator-specific: trigger next pair or phase completion (unchanged)
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

    // FB phase complete → send FINAL to initiator2
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

static int gPathPlanCount = 0; // counts how many modules planned so far
static const int kGroupSize = 2; // every 2 modules → reset

static void resetReservationTableIfNeeded() {
    gPathPlanCount++;
    if (gPathPlanCount % kGroupSize == 0) {

        gReservationTable.clear();
    }
}

std::vector<Cell3DPosition>
CatomsMAPFBlockCode::findPath(Cell3DPosition &start, Cell3DPosition &goal) {
    // Optional group-based cleanup, keep your logic
    resetReservationTableIfNeeded();

    // After reservations succeed



    // ---- Heuristic (L1 on lattice) ----
    auto h = [](const Cell3DPosition& a, const Cell3DPosition& b) -> int {
        return std::abs(a[0] - b[0]) + std::abs(a[1] - b[1]) + std::abs(a[2] - b[2]);
    };

    struct Node {
        Cell3DPosition pos;
        int g;   // steps so far
        int t;   // time steps since start (for time-aware reservations)
        int f;   // g + h
    };
    struct Cmp {
        bool operator()(const Node& a, const Node& b) const {
            if (a.f != b.f) return a.f > b.f;    // min-heap by f
            if (a.t != b.t) return a.t > b.t;    // tie-break: earlier arrival
            return a.g < b.g;                    // stable-ish
        }
    };

    std::priority_queue<Node, std::vector<Node>, Cmp> open;
    std::map<Cell3DPosition, Cell3DPosition> cameFrom;
    std::map<Cell3DPosition, int> gScore;  // best known g
    std::map<Cell3DPosition, int> tBest;   // best arrival time (for tie-breaking)
    std::set<Cell3DPosition> closed;

    //const int baseTime = getScheduler()->now();

    gScore[start] = 0;
    tBest[start]  = 0;
    open.push(Node{start, 0, 0, h(start, goal)});


    bool found = false;
    Cell3DPosition endPos;

    while (!open.empty()) {
        Node cur = open.top(); open.pop();

        // Closed-set check (position, not time; admissible for unit-cost)
        if (closed.count(cur.pos)) continue;
        closed.insert(cur.pos);

        if (cur.pos == goal) {
            found = true;
            endPos = cur.pos;
            break;
        }

        // Expand neighbors from motion rules (physical feasibility)
        std::vector<Cell3DPosition> nbrs;
        bool any = getAllPossibleMotionsFromPosition(cur.pos, nbrs);

        // Add wait action to break conflicts & deadlocks
        nbrs.push_back(cur.pos);

        if (!any && nbrs.size() == 1) continue; // only wait exists; still allowed below

        for (const auto& n : nbrs) {
            const int nextG = cur.g + 1;
            const int nextT = cur.t + 1;

            // Reservation checks (using step time)
            if (gReservationTable.isOccupied(n, nextT)) continue;
            if (gReservationTable.isEdgeReserved(cur.pos, n, nextT)) continue;

            // If we already have a better or equal g (and time) for n, skip
            auto itG = gScore.find(n);
            bool better = (itG == gScore.end()) || (nextG < itG->second)
                        || (nextG == itG->second && nextT < tBest[n]);
            if (!better) continue;

            cameFrom[n] = cur.pos;
            gScore[n]   = nextG;
            tBest[n]    = nextT;

            const int f = nextG + h(n, goal);
            open.push(Node{n, nextG, nextT, f});
        }
    }

    std::vector<Cell3DPosition> path;
    if (!found) {
        console << "[A*] no path " << start << " → " << goal << "\n";
        hasPath = false;
        return path;
    }

    // Reconstruct (position-only path; times from tBest)
    {
        Cell3DPosition p = endPos;
        while (p != start) {
            path.push_back(p);
            p = cameFrom[p];
        }
        std::reverse(path.begin(), path.end());
    }

    // Reserve along absolute time (vertex + edge), including the start at baseTime
    {
        int t = 0;
        gReservationTable.reserve(start, t);

        Cell3DPosition prev = start;
        for (const auto& step : path) {
            ++t;  // next step index
            gReservationTable.reserve(step, t);
            gReservationTable.reserveEdge(prev, step, t);
            prev = step;
        }

    }

    appendPlanToFile(
      module->blockId,
      myMoveOrigin,
      start,
      goal,
      path,
      0,  // baseTime now unused
      "plans.log",
      /*isReplan=*/false
  );



    currentPath = path;
    hasPath     = !path.empty();
    pathIdx     = 0;
    return path;
}





// ---------------------------
// Enumerate all reachable moves from a given position
// ---------------------------
bool CatomsMAPFBlockCode::getAllPossibleMotionsFromPosition(
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


void CatomsMAPFBlockCode::startFarthestWaveFromHere() {
    // if (farthestWaveStarted) return;
    // farthestWaveStarted = true;
    //
    // stage++;
    // initFarthest();
    // myDist = 0;
    // parent = nullptr;
    //
    // nbWaitedAns = sendMessageToAllNeighbors(
    //     "GO", new MessageOf<std::pair<int,int>>(GO_ID, {stage, 0}),
    //     1000, 100, 0);
    //
    // if (nbWaitedAns == 0) { // isolated root
    //     bestDist = myDist;
    //     bestId   = module->blockId;
    // }
}


// ---------------------------
// Required virtuals (stubbed so the linker is happy)
// ---------------------------
bool CatomsMAPFBlockCode::parseUserCommandLineArgument(int &argc, char **argv[]) {
    (void)argc; (void)argv; // unused
    return false;
}

void CatomsMAPFBlockCode::processLocalEvent(EventPtr pev) {
    Catoms3DBlockCode::processLocalEvent(pev);
}

void CatomsMAPFBlockCode::onBlockSelected() {
}

void CatomsMAPFBlockCode::onAssertTriggered() {
}

std::string CatomsMAPFBlockCode::onInterfaceDraw() {
    return std::string();
}

bool CatomsMAPFBlockCode::scheduleGuardedHop() {
    gWaitingForRetry[module->blockId] = false;

    if (gMoveInFlightPerModule[module->blockId]) return false;
    if (!hasPath || !hasGoal) return false;
    if (pathIdx >= static_cast<int>(currentPath.size())) return false;

    while (pathIdx < static_cast<int>(currentPath.size()) &&
           currentPath[pathIdx] == module->position)
        pathIdx++;
    if (pathIdx >= static_cast<int>(currentPath.size())) return false;

    auto feasibleNow = [&](const Cell3DPosition &p) -> bool {
        if (!lattice->isInGrid(p) || !lattice->isFree(p)) return false;
        if (!module->canMoveTo(p)) return false;
        return true;
    };

    auto attempt = [&]() -> bool {
        if (pathIdx >= static_cast<int>(currentPath.size())) return false;

        const Cell3DPosition nextPos = currentPath[pathIdx];
        auto mineIt = gLastDest.find(module->blockId);
        bool reservedByMe = (mineIt != gLastDest.end() && mineIt->second == nextPos);

        if (!reservedByMe && gReservedCells.count(nextPos)) return false;
        if (!feasibleNow(nextPos)) return false;

        gReservedCells.insert(nextPos);
        gLastDest[module->blockId] = nextPos;

        //  Priority-based jitter: modules with lower IDs move slightly earlier
        const uint64_t when = getScheduler()->now() + 100 + (module->blockId % 7) * 20;
        getScheduler()->schedule(new Catoms3DRotationStartEvent(when, module, nextPos));

        gMoveInFlightPerModule[module->blockId] = true;
        gWaitingForRetry[module->blockId] = false;
        return true;
    };

    if (attempt()) return true;

    //  Replan with immediate re-reservation
    currentPath.clear();
    hasPath = false;
    noteReplan(module->blockId);

    // log context for the replan
    const Cell3DPosition fromNow = module->position;
    const int baseNow = getScheduler()->now();

    auto newPath = findPath(module->position, goalForThisMove);
    if (newPath.empty()) {
        gWaitingForRetry[module->blockId] = true;
        return false;
    }

    // tag this entry as a replan
    appendPlanToFile(module->blockId, myMoveOrigin, fromNow, goalForThisMove,
                     newPath, baseNow, "plans.log", /*isReplan=*/true);


    // Immediately re-reserve the new path (findPath already did)
    currentPath = newPath;
    hasPath = true;
    pathIdx = 0;

    // retry move with new path
    if (attempt()) return true;

    //  action fallback to prevent deadlock
    console << "[PLAN] Deadlock risk, issuing wait-step before retry.\n";
    uint64_t delay = 1000 + (module->blockId % 5) * 200;
    getScheduler()->schedule(new CodeStartEvent(getScheduler()->now() + delay, module));

    gWaitingForRetry[module->blockId] = true;
    return false;
}


