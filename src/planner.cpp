#include "planner.h"
#include <cstring>
#include <iostream>
#include <boost/heap/binomial_heap.hpp>
using namespace HybridAStar;

HybridAStar::Planner::Planner()
{

}

void HybridAStar::Planner::plan()
{
    if(!validGoal || !validStart)
    {
        std::cerr<<"error !!"<<" start state : "<<validStart<<" goal state : "<<validGoal<<std::endl;
        return;
    }
    Node3D* nSolution = hybridAStar(true);
}

void HybridAStar::Planner::setMap(unsigned char *data, int length, int width, float resolution)
{
    m_length = length;
    m_width = width;
    m_inv_resolution = (int)(1.f / resolution);
    int num = (length * m_inv_resolution) * (width * m_inv_resolution);
    m_map = new unsigned char[num];
    memcpy(m_map,data,num*sizeof(unsigned char));
}

// 0----> x
// |
// | y
//
float HybridAStar::Planner::getMapById(int index_x, int index_y)
{
    if(index_x > m_length*m_inv_resolution ||
       index_y > m_width*m_inv_resolution ||
       index_y < 0 ||
       index_x <0)
    {
        return 0.f;
    }
    int id = index_x + index_y * m_length;
    return m_map[id];
}

//  ----> x
// |
// | y
//
float HybridAStar::Planner::getMapByPos(float x, float y)
{
    int idx = (int)(x * (float)m_inv_resolution);
    int idy = (int)(y * (float)m_inv_resolution);
    return getMapById(idx,idy);
}


struct CompareNodes {
    /// Sorting 3D nodes by increasing C value - the total estimated cost
    bool operator()(const Node3D* lhs, const Node3D* rhs) const {
        return lhs->getC() > rhs->getC();
    }
    /// Sorting 2D nodes by increasing C value - the total estimated cost
    bool operator()(const Node2D* lhs, const Node2D* rhs) const {
        return lhs->getC() > rhs->getC();
    }
};

HybridAStar::Node3D *HybridAStar::Planner::hybridAStar(bool reverse)
{
    int dir = reverse ? 6 : 3;
    int iterations = 0;
    typedef boost::heap::binomial_heap<Node3D*,
            boost::heap::compare<CompareNodes>
    > priorityQueue;
    priorityQueue open;
}
