#ifndef CONSTRAINT_TREE_H
#define CONSTRAINT_TREE_H

#include <map>
#include <vector>
#include <queue>
#include <functional>
#include <optional>
#include <limits>
#include <algorithm>
#include <unordered_set>


#include "robots/catoms3D/catoms3DWorld.h"

// Keep the same using as before to avoid churn
using namespace Catoms3D;

/* ===========================================================
   Constraint Tree (CBS) – types & helpers
   Replaces the old ReservationTable with CBS primitives.
   =========================================================== */

enum class CBSCostMetric { SumOfCosts, Makespan };

// A single constraint applied to exactly one agent.
struct Constraint {
    int agentId;                  // which agent it applies to
    int t;                        // discrete time
    bool isVertex;                // true = vertex constraint, false = edge
    Cell3DPosition v;             // forbidden vertex at time t (if isVertex)
    Cell3DPosition from, to;      // forbidden directed edge at time t (if !isVertex)
};

using ConstraintSet = std::vector<Constraint>;

// A detected conflict between two agents in a joint solution.
struct Conflict {
    int a1 = -1, a2 = -1;
    int t = -1;
    bool isVertex = true;
    Cell3DPosition v;             // vertex conflict @ t
    Cell3DPosition a1_from, a1_to;  // edge conflict fields
    Cell3DPosition a2_from, a2_to;
    bool valid() const { return a1 >= 0 && a2 >= 0; }
};

// A Constraint Tree node: constraints + per-agent paths + cost.
struct CTNode {
    ConstraintSet constraints;                                   // all constraints active in this node
    std::map<int, std::vector<Cell3DPosition>> solution;         // path per agent
    int cost = std::numeric_limits<int>::max();                  // node cost
    int id = 0;                                                  // optional for debugging
};

// Check if a move (from -> to at time t) violates any constraint for 'agentId'.
inline bool violatesConstraints(int agentId,
                                const Cell3DPosition& from,
                                const Cell3DPosition& to,
                                int t,
                                const ConstraintSet& C)
{
    for (const auto& c : C) {
        if (c.agentId != agentId) continue;

        if (c.isVertex) {
            if (c.t == t && c.v == to) return true;
        } else {
            if (c.t == t && c.from == from && c.to == to) return true;
            // also forbid swap in the same instant if you like symmetrical constraints
            // if (c.t == t && c.from == to && c.to == from) return true;
        }
    }
    return false;
}

// Utility to fetch position at time t, with "wait at end" semantics.
inline Cell3DPosition atTime(const std::vector<Cell3DPosition>& path, int t) {
    if (path.empty()) return Cell3DPosition(-9999,-9999,-9999);
    if (t < 0) return path.front();
    if (t >= static_cast<int>(path.size())) return path.back();
    return path[t];
}

// Find the first pairwise conflict (vertex or edge) between any two agents.
inline std::optional<Conflict>
findFirstConflict(const std::map<int, std::vector<Cell3DPosition>>& sol)
{
    if (sol.size() < 2) return std::nullopt;

    // Determine the horizon to check (max length among paths).
    int T = 0;
    for (auto& kv : sol) T = std::max(T, static_cast<int>(kv.second.size()));
    if (T == 0) return std::nullopt;

    // Check all pairs
    for (auto it1 = sol.begin(); it1 != sol.end(); ++it1) {
        for (auto it2 = std::next(it1); it2 != sol.end(); ++it2) {
            int a = it1->first, b = it2->first;
            const auto& Pa = it1->second;
            const auto& Pb = it2->second;

            for (int t = 0; t < T; ++t) {
                Cell3DPosition a_t   = atTime(Pa, t);
                Cell3DPosition b_t   = atTime(Pb, t);
                Cell3DPosition a_t1  = atTime(Pa, t+1);
                Cell3DPosition b_t1  = atTime(Pb, t+1);

                // Vertex conflict
                if (a_t == b_t) {
                    Conflict c;
                    c.a1 = a; c.a2 = b; c.t = t; c.isVertex = true; c.v = a_t;
                    return c;
                }
                // Edge conflict (swap)
                if (a_t == b_t1 && b_t == a_t1) {
                    Conflict c;
                    c.a1 = a; c.a2 = b; c.t = t; c.isVertex = false;
                    c.a1_from = a_t; c.a1_to = a_t1;
                    c.a2_from = b_t; c.a2_to = b_t1;
                    return c;
                }
            }
        }
    }
    return std::nullopt;
}

// Cost functions
inline int computeSumOfCosts(const std::map<int, std::vector<Cell3DPosition>>& sol) {
    int sum = 0;
    for (auto& kv : sol) sum += static_cast<int>(kv.second.size());
    return sum;
}

inline int computeMakespan(const std::map<int, std::vector<Cell3DPosition>>& sol) {
    int m = 0;
    for (auto& kv : sol) m = std::max(m, static_cast<int>(kv.second.size()));
    return m;
}

inline int computeCost(const std::map<int, std::vector<Cell3DPosition>>& sol,
                       CBSCostMetric metric)
{
    return (metric == CBSCostMetric::SumOfCosts)
        ? computeSumOfCosts(sol)
        : computeMakespan(sol);
}

/* ===========================================================
   Minimal high-level CBS driver (single-file, header-only).
   You pass a low-level planner (A*) as a std::function.
   =========================================================== */

class CBSPlanner {
public:
    // Low-level single-agent planner you already have:
    // plan(agentId, start, goal, constraints) -> path (may be empty if impossible)
    using LowLevelPlanFn = std::function<std::vector<Cell3DPosition>(
        int /*agentId*/,
        const Cell3DPosition& /*start*/,
        const Cell3DPosition& /*goal*/,
        const ConstraintSet&  /*constraints*/
    )>;

    struct AgentSpec { int id; Cell3DPosition start, goal; };

    CBSPlanner(std::vector<AgentSpec> agents,
               LowLevelPlanFn lowLevel,
               CBSCostMetric metric = CBSCostMetric::SumOfCosts)
      : agents_(std::move(agents)), lowLevel_(std::move(lowLevel)), metric_(metric) {}

    // Returns a conflict-free solution or an empty map if impossible
    std::map<int, std::vector<Cell3DPosition>> solve() {
        auto cmp = [](const CTNode& a, const CTNode& b){ return a.cost > b.cost; };
        std::priority_queue<CTNode, std::vector<CTNode>, decltype(cmp)> open(cmp);

        // Root node: no constraints, plan all agents
        CTNode root;
        root.constraints = {};
        if (!planAll(root)) return {};
        root.cost = computeCost(root.solution, metric_);
        root.id = ++uid_;
        open.push(root);

        while (!open.empty()) {
            CTNode node = open.top(); open.pop();

            auto conflict = findFirstConflict(node.solution);
            if (!conflict.has_value()) {
                // Success
                return node.solution;
            }

            const Conflict& cf = *conflict;

            // Child 1: add constraint for a1
            {
                CTNode c = node;
                Constraint k;
                k.agentId = cf.a1; k.t = cf.t; k.isVertex = cf.isVertex;
                if (cf.isVertex) { k.v = cf.v; }
                else { k.from = cf.a1_from; k.to = cf.a1_to; }
                c.constraints.push_back(k);

                // Replan only the constrained agent
                auto path = lowLevel_(cf.a1, agentStart(cf.a1), agentGoal(cf.a1), c.constraints);
                if (!path.empty()) {
                    c.solution[cf.a1] = std::move(path);
                    c.cost = computeCost(c.solution, metric_);
                    c.id = ++uid_;
                    open.push(std::move(c));
                }
            }

            // Child 2: add constraint for a2
            {
                CTNode c = node;
                Constraint k;
                k.agentId = cf.a2; k.t = cf.t; k.isVertex = cf.isVertex;
                if (cf.isVertex) { k.v = cf.v; }
                else { k.from = cf.a2_from; k.to = cf.a2_to; }
                c.constraints.push_back(k);

                auto path = lowLevel_(cf.a2, agentStart(cf.a2), agentGoal(cf.a2), c.constraints);
                if (!path.empty()) {
                    c.solution[cf.a2] = std::move(path);
                    c.cost = computeCost(c.solution, metric_);
                    c.id = ++uid_;
                    open.push(std::move(c));
                }
            }
        }

        // No solution
        return {};
    }

private:
    bool planAll(CTNode& n) {
        n.solution.clear();
        for (const auto& a : agents_) {
            auto path = lowLevel_(a.id, a.start, a.goal, n.constraints);
            if (path.empty()) return false;
            n.solution[a.id] = std::move(path);
        }
        return true;
    }

    Cell3DPosition agentStart(int id) const {
        for (auto& a : agents_) if (a.id == id) return a.start;
        return Cell3DPosition();
    }
    Cell3DPosition agentGoal(int id) const {
        for (auto& a : agents_) if (a.id == id) return a.goal;
        return Cell3DPosition();
    }

    std::vector<AgentSpec> agents_;
    LowLevelPlanFn lowLevel_;
    CBSCostMetric metric_;
    int uid_ = 0; // node ids
};

#endif
