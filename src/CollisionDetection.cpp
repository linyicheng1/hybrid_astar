#include "CollisionDetection.h"

using namespace HybridAStar;

void getConfiguration(const Node2D* node, float& x, float& y, float& t)
{
    x = node->getX();
    y = node->getY();
    // avoid 2D collision checking
    t = 99; //2D的getConfiguration为多态函数，2D网格时统一将t=99
}

void getConfiguration(const Node3D* node, float& x, float& y, float& t)
{
    x = node->getX();
    y = node->getY();
    t = node->getT();
}

HybridAStar::CollisionDetection::CollisionDetection(
        unsigned char *data, int width,
        int height):
        m_height(height),m_width(width)
{
    m_map = new unsigned char[height*width];
    memcpy(m_map,data,height*width*sizeof(unsigned char));
}

bool HybridAStar::CollisionDetection::isTraversable(const Node3D* node)
{
    float x;
    float y;
    float t;
    getConfiguration(node, x, y, t);
    if(x > m_height  || x < 0 || y > m_width  || y <0)
    {
        return false;
    }
    //@TODO
    int id = Node2D(x,y,0,0,nullptr).setIdx(getWidthSize());
    return m_map[id] > 250 && m_map[id+1] > 250
           && m_map[id+m_width] > 250 && m_map[id+1+m_width] > 250;
    //return configurationTest(x, y, t);
}

bool HybridAStar::CollisionDetection::isTraversable(const Node2D *node)
{
    //可通行性检验：1) 标准方法：使用空间占据计算检查；2) 其他：使用2D代价地图和导航器检查
    float x;
    float y;
    float t;
    // assign values to the configuration
    getConfiguration(node, x, y, t);
    if(x > m_height  || x < 0 || y > m_width  || y <0)
    {
        return false;
    }
    return m_map[node->getIdx()] > 250 && m_map[node->getIdx()+1] > 250
        && m_map[node->getIdx()+m_width] > 250 && m_map[node->getIdx()+1+m_width] > 250;
}

bool HybridAStar::CollisionDetection::configurationTest(float x, float y, float t)
{
    int X = (int)x;
    int Y = (int)y;

    int iX = (int)((x - (float)((int)x)) * param::positionResolution);//得出X方向在cell中的偏移量
    iX = iX > 0 ? iX : 0;
    int iY = (int)((y - (float)((int)y)) * param::positionResolution);//Y方向在cell中的偏移量
    iY = iY > 0 ? iY : 0;
    int iT = (int)(t / param::deltaHeadingRad);
    int idx = iY * param::positionResolution * param::headings + iX * param::headings + iT;
    int cX;
    int cY;

    for (int i = 0; i < collisionLookup[idx].length; ++i)
    {
        cX = (X + collisionLookup[idx].pos[i].x);
        cY = (Y + collisionLookup[idx].pos[i].y);

        // make sure the configuration coordinates are actually on the grid
        if (cX >= 0 && (unsigned int)cX < m_height && cY >= 0 && (unsigned int)cY < m_width)
        {
            if (m_map[cY * m_width + cX] < 250)
            {
                return false;
            }//若grid的某个小网格存在值，说明有障碍，则返回false表示不在自由网格
        }
    }
    return true;//所有检测都没有检测到被占用，说明没有障碍，可以通行
}

void HybridAStar::CollisionDetection::updateGrid(unsigned char *data, int width, int height)
{
    delete m_map;
    m_width = width;
    m_height = height;
    m_map = new unsigned char[height*width];
    memcpy(m_map,data,height*width*sizeof(unsigned char));
}
