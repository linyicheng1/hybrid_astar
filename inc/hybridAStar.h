#ifndef HYBRID_ASTAR_HYBRIDASTAR_H
#define HYBRID_ASTAR_HYBRIDASTAR_H
#include "node.h"
#include "CollisionDetection.h"
#include "ReedsSheppPath.h"

namespace HybridAStar {
    class hybridAStar
    {
    public:
        explicit hybridAStar(CollisionDetection *map);
        ~hybridAStar();
        Node3D* search_planer(Node3D& start, Node3D& goal);
        float aStar(Node2D& start, Node2D& goal, float scale = 1);
        void interpolate(const ReedsShepp::pos *from, double t,
                         ReedsShepp::pos *state) const;
    private:
        void updateH(Node3D& start, const Node3D& goal);
        Node2D* m_nodes2D{};
        Node3D* m_nodes3D{};
        CollisionDetection *m_map{};
        ReedsShepp m_RS{10};
        ReedsShepp::ReedsSheppPath m_RS_path;
    };
}
#endif //HYBRID_ASTAR_HYBRIDASTAR_H
