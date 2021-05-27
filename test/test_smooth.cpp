#include "CollisionDetection.h"
#include <opencv2/opencv.hpp>
#include "node.h"
#include "hybridAStar.h"
#include "smooth.h"
#include "dynamicVoronoi.h"

using namespace HybridAStar;

int main()
{
    cv::Mat map = cv::imread("../test/map.png",cv::IMREAD_GRAYSCALE);
    int inv_resolution = 1;
    cv::resize(map,map,cv::Size(map.cols/25,map.rows/25));
    cv::Mat raw = map.clone();
    CollisionDetection map_data(map.data,map.cols ,map.rows );
    hybridAStar planer(&map_data);
    Node3D start(8,8,0,0,0, nullptr);
    Node3D goal(260,120,1.5,0,0, nullptr);
    auto nSolution = planer.search_planer(start,goal,0.2);
    auto tmp_show = nSolution;
    while(tmp_show != nullptr)
    {
        int x = (int)tmp_show->getX()*inv_resolution;
        int y = (int)tmp_show->getY()*inv_resolution;
        map.at<uchar>(x, y) = 125;
        if(raw.at<uchar>(x, y) < 250)
        {
            std::cerr<<" error !! "<<(int)raw.at<uchar>(y, x)<<" pos: x "<<x<<" y "<<y<<" angle "<<tmp_show->getT()<<std::endl;
        }
        //std::cout<<"pos: x "<<x<<" y "<<y<<" angle "<<ptr->getT()<<std::endl;
        tmp_show = tmp_show->getPred();
        if(tmp_show != nullptr)
        {
            int next_x = (int)tmp_show->getX()*inv_resolution;
            int next_y = (int)tmp_show->getY()*inv_resolution;
            //cv::line(map,cv::Point(y,x),cv::Point(next_y,next_x),cv::Scalar(0,0,0));
        }
    }
    DynamicVoronoi voronoi;
    bool** binMap;//二维数组，
    binMap = new bool*[map.rows];
    for (int x = 0; x < map.rows; x++) { binMap[x] = new bool[map.cols]; }
    for (int x = 0; x < map.rows; ++x)
    {
        for (int y = 0; y < map.cols; ++y)
        {
            binMap[x][y] = raw.data[y + x * map.cols] < 250;
        }
    }//转化为二值地图
    voronoi.initializeMap(map.rows,map.cols,binMap);
    voronoi.update();
    Smoother smoother;
    smoother.tracePath(nSolution);
    smoother.smoothPath(voronoi);
    auto smooth_path = smoother.getPath();
    for(auto pt:smooth_path)
    {
        int x = (int)pt.getX();
        int y = (int)pt.getY();
        map.at<uchar>(x, y) = 0;
    }
    //cv::imwrite("../pic/smooth.png",map);
    cv::imshow("result",map);
    cv::waitKey(0);
}





