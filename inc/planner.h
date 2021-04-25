#ifndef HYBRID_ASTAR_PLANNER_H
#define HYBRID_ASTAR_PLANNER_H
#include "node.h"

namespace HybridAStar
{
    class Planner
    {
        class position
        {
        public:
            float x = 0;
            float y = 0;
            float angle = 0;
            position() = default;
            position(float _x,float _y,float _angle)
            {
                x = _x;
                y = _y;
                angle = _angle;
            }
            position & operator=(const position &t1)= default;
        };
    public:
        Planner();
        // operator
        void setMap(unsigned char *data,int length,int width,float resolution);
        void setStart(position start){m_start = start;validStart = true;}
        void setGoal(position goal){m_goal = goal;validGoal = true;}
        void plan();
        // get messages

    private:
        position m_start;
        position m_goal;
        bool validStart = false;
        bool validGoal = false;
        unsigned char * m_map;
        int m_width;
        int m_length;
        int m_inv_resolution;
        float getMapById(int index_x,int index_y);
        float getMapByPos(float x,float y);
        Node3D* hybridAStar(bool reverse);
    };
}

#endif //HYBRID_ASTAR_PLANNER_H
