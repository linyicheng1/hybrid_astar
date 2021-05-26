#include "CollisionDetection.h"
#include <opencv2/opencv.hpp>
#include "node.h"
#include "dynamicVoronoi.h"

using namespace HybridAStar;

int main()
{
    cv::Mat map = cv::imread("../test/map.png",cv::IMREAD_GRAYSCALE);
    cv::resize(map,map,cv::Size(map.cols/25,map.rows/25));
    DynamicVoronoi voronoi;
    bool** binMap;//二维数组，
    binMap = new bool*[map.cols];
    for (int x = 0; x < map.cols; x++) { binMap[x] = new bool[map.rows]; }
    for (int x = 0; x < map.cols; ++x)
    {
        for (int y = 0; y < map.rows; ++y)
        {
            binMap[x][y] = map.data[y * map.cols + x] < 250;
        }
    }//转化为二值地图
    voronoi.initializeMap(map.cols,map.rows,binMap);
    voronoi.update();
    voronoi.visualize("../pic/result.pgm");
    for (int x = 0; x < map.rows; ++x)
    {
        for (int y = 0; y < map.cols; ++y)
        {
            if(!voronoi.isOccupied(y,x))
            {
                std::cout<<" ";
            }
            else
            {
                std::cout<<"*";
            }
        }
        std::cout<<std::endl;
    }
}


