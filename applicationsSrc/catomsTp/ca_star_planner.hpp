#pragma once
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <vector>
#include "robots/catoms3D/catoms3DWorld.h"
using namespace Catoms3D;

namespace CAStar {

static inline long long enc_occ(const Cell3DPosition& p, int t) {
    return ((long long)p[0] << 40) | ((long long)p[1] << 20) | (long long)t;
}
static inline unsigned long long enc_trans(const Cell3DPosition& from,
                                           const Cell3DPosition& to,
                                           int t) {
    return ((unsigned long long)(unsigned int)from[0] << 44)
         | ((unsigned long long)(unsigned int)from[1] << 24)
         | ((unsigned long long)(unsigned int)to[0] << 12)
         | ((unsigned long long)(unsigned int)t);
}

struct Table {
    std::unordered_set<long long> occ;
    std::unordered_set<unsigned long long> trans;

    bool blocked_vertex(const Cell3DPosition& p, int t) const {
        return occ.count(enc_occ(p,t));
    }
    bool blocked_swap(const Cell3DPosition& a, const Cell3DPosition& b, int t) const {
        return trans.count(enc_trans(b,a,t));
    }
    void reserve_path(const std::vector<Cell3DPosition>& path) {
        for(int t=0;t<(int)path.size();++t){
            occ.insert(enc_occ(path[t],t));
            if(t>0) trans.insert(enc_trans(path[t-1],path[t],t));
        }
        Cell3DPosition g=path.back();
        for(int pad=1;pad<=3;++pad)
            occ.insert(enc_occ(g,(int)path.size()-1+pad));
    }
};

struct State {
    Cell3DPosition p;
    int t,g,f;
    Cell3DPosition parent;
    int pt;
};
struct Cmp { bool operator()(const State&a,const State&b)const{return a.f>b.f;} };

static inline int h_zero(const Cell3DPosition&, const Cell3DPosition&){return 0;}

    template <typename NeighborFunc>
    inline std::vector<Cell3DPosition>
    plan(const Cell3DPosition& start,
         const Cell3DPosition& goal,
         NeighborFunc neighbors,
         const Table& table,
         int horizon=200)

{
    std::priority_queue<State,std::vector<State>,Cmp> open;
    std::unordered_map<long long,int> best;
    std::unordered_map<long long,std::pair<Cell3DPosition,int>> parent;

    auto push=[&](const Cell3DPosition&p,int t,int g,int f,const Cell3DPosition&pp,int pt){
        long long key=enc_occ(p,t);
        if(best.count(key)&&best[key]<=g) return;
        best[key]=g; parent[key]={pp,pt};
        open.push({p,t,g,f,pp,pt});
    };

    push(start,0,0,h_zero(start,goal),{-999,-999,-999},-1);

    while(!open.empty()){
        auto s=open.top();open.pop();
        if(s.p==goal){
            // reconstruct
            std::vector<Cell3DPosition> path;
            Cell3DPosition p=s.p;int t=s.t;
            while(!(p==Cell3DPosition(-999,-999,-999))){
                path.push_back(p);
                auto it=parent.find(enc_occ(p,t));
                if(it==parent.end()) break;
                auto [pp,pt]=it->second; p=pp; t=pt;
            }
            std::reverse(path.begin(),path.end());
            return path;
        }
        if(s.t>=horizon) continue;

        // WAIT
        int nt=s.t+1;
        if(!table.blocked_vertex(s.p,nt))
            push(s.p,nt,s.g+1,s.g+1+h_zero(s.p,goal),s.p,s.t);

        // MOVE
        for(auto &n:neighbors(s.p)){
            if(table.blocked_vertex(n,nt)) continue;
            if(table.blocked_swap(s.p,n,nt)) continue;
            push(n,nt,s.g+1,s.g+1+h_zero(n,goal),s.p,s.t);
        }
    }
    return {};
}

} // namespace CAStar
