#ifndef HYBRID_ASTAR_COLLISIONDETECTION_H
#define HYBRID_ASTAR_COLLISIONDETECTION_H
#include <cstring>
#include "node.h"

namespace HybridAStar
{
    class CollisionDetection {
    public:

        CollisionDetection(unsigned char *data,int width,int height);

        bool isTraversable(const Node2D* node);
        bool isTraversable(const Node3D* node);
        bool configurationTest(float x, float y, float t);

        void updateGrid(unsigned char *data,int width,int height);
        int getSize() const{return m_width*m_height;}
        int getWidthSize() const{return m_width;}
        int getHeightSize() const{return m_height;}
        int getWidth() const{return m_width;}
        int getHeight() const{return m_height;}
    private:
        int m_width;
        int m_height;
        unsigned char * m_map;
        param::config collisionLookup[param::headings * param::positions];
    };
}
#endif //HYBRID_ASTAR_COLLISIONDETECTION_H
