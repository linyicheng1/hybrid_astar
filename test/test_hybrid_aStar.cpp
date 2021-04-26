#include "CollisionDetection.h"
#include <opencv2/opencv.hpp>
#include "node.h"
#include "hybridAStar.h"
using namespace HybridAStar;

int main()
{
    cv::Mat map = cv::imread("../test/map.png",cv::IMREAD_GRAYSCALE);
    cv::resize(map,map,cv::Size(map.cols/50,map.rows/50));
    CollisionDetection map_data(map.data,map.cols/2  ,map.rows/2 ,2);
    hybridAStar planer(&map_data);
    Node3D start(10,10,0,0,0, nullptr);
    Node3D goal(150,85,0,0,0, nullptr);
    planer.search_planer(start,goal);
    // show
    Node3D* ptr = goal.getPred();
    while(ptr != nullptr)
    {
        int x = ptr->getX();
        int y = ptr->getY();
        map.at<uchar>(y, x) = 0;
        ptr = ptr->getPred();
    }
    ptr = goal.getPred();
    for(int i = 0;i < 500;i ++)
    {
        float t = (float)i / 500.f;
        ReedsShepp::pos p;
        ReedsShepp::pos st(ptr->getX(),ptr->getY(),ptr->getT());
        planer.interpolate(&st,t,&p);
        std::cout<<"pos: x "<<p.x<<" y "<<p.y<<" angle "<<p.angle<<std::endl;
        int x = (int)(p.x);
        int y = (int)(p.y);
        map.at<uchar>(y, x) = 120;
    }
    //cv::imwrite("../pic/hybridAStar.png",map);
    //cv::resize(map,map,cv::Size(map.cols*3,map.rows*3));
    cv::imshow("result",map);
    cv::waitKey(0);
}



