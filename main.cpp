#include <iostream>
#include "planner.h"
#include <opencv2/opencv.hpp>
using namespace HybridAStar;

int main()
{
    Node3D start(8,8,3.14,0,0, nullptr);
    Node3D goal(320,160,3.14,0,0, nullptr);
    cv::Mat map = cv::imread("../test/map.png",cv::IMREAD_GRAYSCALE);
    cv::resize(map,map,cv::Size(map.cols/25,map.rows/25));
    Planner planner(map.data,map.cols ,map.rows);
    planner.setStart(start);
    planner.setGoal(goal);
    planner.plan();

    auto path = planner.getPath();
    auto smooth_path = planner.getSmoothPath();
    for(auto pt:path)
    {
        int x = (int)pt.getX();
        int y = (int)pt.getY();
        map.at<uchar>(y, x) = 120;
    }
    for(auto pt:smooth_path)
    {
        int x = (int)pt.getX();
        int y = (int)pt.getY();
        map.at<uchar>(y, x) = 0;
        if(pt.getPrim() == -2)
        {
            map.at<uchar>(y, x+1) = 0;
            map.at<uchar>(y+1, x) = 0;
            map.at<uchar>(y+1, x+1) = 0;
        }
    }
    cv::resize(map,map,cv::Size(map.cols*3,map.rows*3));
    cv::imshow("result",map);
    cv::waitKey(0);
    return 0;
}
