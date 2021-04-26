#ifndef HYBRID_ASTAR_COLLISIONDETECTION_H
#define HYBRID_ASTAR_COLLISIONDETECTION_H
#include <cstring>
#include "node.h"

namespace HybridAStar
{
    class CollisionDetection {
    public:

        CollisionDetection(unsigned char *data,int width,int height,int inv_resolution);

        bool isTraversable(const Node2D* node);
        bool isTraversable(const Node3D* node);
        bool configurationTest(float x, float y, float t);

        void updateGrid(unsigned char *data,int width,int height,int inv_resolution);
        int getSize() const{return m_width*m_height*m_inv_resolution*m_inv_resolution;}
        int getWidthSize() const{return m_width*m_inv_resolution;}
        int getHeightSize() const{return m_height*m_inv_resolution;}
    private:
        int m_width;
        int m_height;
        int m_inv_resolution;
        unsigned char * m_map;
        param::config collisionLookup[param::headings * param::positions];
    };
}
#endif //HYBRID_ASTAR_COLLISIONDETECTION_H
