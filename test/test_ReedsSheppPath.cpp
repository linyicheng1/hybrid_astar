#include "ReedsSheppPath.h"
#include <opencv2/opencv.hpp>
using namespace HybridAStar;

int main()
{
    cv::Mat map = cv::imread("../test/write.png",cv::IMREAD_GRAYSCALE);
    ReedsShepp planner(10);
    ReedsShepp::pos start(1,1,0);
    ReedsShepp::pos goal(50,30,3.14);
    auto path = planner.plan(start,goal);

    for(int i = 0;i < 500;i ++)
    {
        float t = (float)i / 500.f;
        ReedsShepp::pos p;
        planner.interpolate(&start,path,t,&p);
        std::cout<<"pos: x "<<p.x<<" y "<<p.y<<" angle "<<p.angle<<std::endl;
        int x = (int)(p.x * 10);
        int y = (int)(p.y * 10);
        float dx = 50*cos(p.angle);
        float dy = 50*sin(p.angle);
        for(int j = 0;j < 50;j ++)
        {
            int x1 = (int)((float)x + dx * (float)j / 50.f);
            int y1 = (int)((float)y + dy * (float)j / 50.f);
            map.at<uchar>(y1, x1) = 125;
        }
        map.at<uchar>(y, x) = 0;
    }
    //cv::imwrite("../pic/RS.png",map);
    cv::imshow("test",map);
    cv::waitKey(0);
}

