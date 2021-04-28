#ifndef HYBRID_ASTAR_PLANNER_H
#define HYBRID_ASTAR_PLANNER_H
#include "node.h"
#include "CollisionDetection.h"
#include "hybridAStar.h"
#include "smooth.h"
#include "dynamicVoronoi.h"

namespace HybridAStar
{
    class Planner
    {
    public:
        Planner(unsigned char *data,int width,int height);
        void setStart(Node3D start){m_start = start;validStart = true;}
        void setGoal(Node3D goal){m_goal = goal;validGoal = true;}
        void plan();
        std::vector<Node3D> getSmoothPath(){return m_smooth_path;}
        std::vector<Node3D> getPath(){return m_path;}
    private:
        Node3D m_start;
        Node3D m_goal;
        bool validStart = false;
        bool validGoal = false;
        CollisionDetection *m_map;
        DynamicVoronoi *m_voronoi;
        hybridAStar *m_planer;
        Smoother m_smoother;
        std::vector<Node3D> m_path;
        std::vector<Node3D> m_smooth_path;
    };
}

#endif //HYBRID_ASTAR_PLANNER_H
