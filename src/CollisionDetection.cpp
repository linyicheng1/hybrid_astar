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
        int height, int inv_resolution):m_inv_resolution(inv_resolution),
        m_height(height),m_width(width)
{
    m_map = new unsigned char[height*width*inv_resolution*inv_resolution];
    memcpy(m_map,data,height*width*inv_resolution*inv_resolution*sizeof(unsigned char));
}


bool HybridAStar::CollisionDetection::isTraversable(const Node2D *node)
{
    //可通行性检验：1) 标准方法：使用空间占据计算检查；2) 其他：使用2D代价地图和导航器检查
    float cost = 0;
    float x;
    float y;
    float t;
    // assign values to the configuration
    getConfiguration(node, x, y, t);

    if (t == 99)
    {// 2D
        return m_map[node->getIdx()] > 250;
    }
    else
    {// 3D
        return configurationTest(x, y, t);
    }
}


bool HybridAStar::CollisionDetection::configurationTest(float x, float y, float t)
{
//    int X = (int)x;
//    int Y = (int)y;
//
//    int iX = (int)((x - (long)x) * Constants::positionResolution);//得出X方向在cell中的偏移量
//    iX = iX > 0 ? iX : 0;
//    int iY = (int)((y - (long)y) * Constants::positionResolution);//Y方向在cell中的偏移量
//    iY = iY > 0 ? iY : 0;
//    int iT = (int)(t / Constants::deltaHeadingRad);
//    int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
//    int cX;
//    int cY;
//
//    for (int i = 0; i < collisionLookup[idx].length; ++i)
//    {
//        cX = (X + collisionLookup[idx].pos[i].x);
//        cY = (Y + collisionLookup[idx].pos[i].y);
//
//        // make sure the configuration coordinates are actually on the grid
//        if (cX >= 0 && (unsigned int)cX < grid->info.width && cY >= 0 && (unsigned int)cY < grid->info.height)
//        {
//            if (grid->data[cY * grid->info.width + cX]) {
//                return false;
//            }//若grid的某个小网格存在值，说明有障碍，则返回false表示不在自由网格
//        }
//    }

    return true;//所有检测都没有检测到被占用，说明没有障碍，可以通行
}

void HybridAStar::CollisionDetection::updateGrid(unsigned char *data, int width, int height, int inv_resolution)
{
    delete m_map;
    m_width = width;
    m_height = height;
    m_inv_resolution = inv_resolution;
    m_map = new unsigned char[height*width*inv_resolution*inv_resolution];
    memcpy(m_map,data,height*width*inv_resolution*inv_resolution*sizeof(unsigned char));
}
