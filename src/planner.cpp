#include "planner.h"
#include <iostream>
using namespace HybridAStar;

Planner::Planner(unsigned char *data,int width,int height)
{
    m_map = new CollisionDetection(data, width ,height);
    m_planer = new hybridAStar(m_map);
    m_voronoi = new DynamicVoronoi();
    bool** binMap;//二维数组，
    binMap = new bool*[height];
    for (int x = 0; x < height; x++) { binMap[x] = new bool[width]; }
    for (int x = 0; x < height; ++x)
    {
        for (int y = 0; y < width; ++y)
        {
            binMap[x][y] = data[y + x * width] < 250;
        }
    }//转化为二值地图
    m_voronoi->initializeMap(height,width,binMap);
    m_voronoi->update();
}

void Planner::plan()
{
    if(!validGoal || !validStart)
    {
        std::cerr<<"error !!"<<" start state : "<<validStart<<" goal state : "<<validGoal<<std::endl;
        return;
    }
    auto nSolution = m_planer->search_planer(m_start,m_goal,0.1);
    m_smoother.tracePath(nSolution);
    m_path = m_smoother.getPath();
    m_smoother.smoothPath(*m_voronoi);
    m_smooth_path = m_smoother.getPath();
}









