#ifndef HYBRID_ASTAR_HYBRIDASTAR_H
#define HYBRID_ASTAR_HYBRIDASTAR_H
#include "node.h"
#include "CollisionDetection.h"

namespace HybridAStar {
    class hybridAStar
    {
    public:
        explicit hybridAStar(CollisionDetection *map);
        ~hybridAStar();
        Node3D* search_planer(bool reverse);
        float aStar(Node2D& start, Node2D& goal);
    private:
        void updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup);
        Node3D* dubinsShot(Node3D& start, const Node3D& goal);
        Node2D *m_nodes2D{};
        CollisionDetection *m_map{};
    };
}
#endif //HYBRID_ASTAR_HYBRIDASTAR_H
