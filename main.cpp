#include <iostream>
#include "planner.h"
#include <opencv2/opencv.hpp>
using namespace HybridAStar;

int main()
{
    int start_x = 8,start_y = 8,end_x=8,end_y=138;
    int start_angle = 0,end_angle = 0;
    cv::Mat map = cv::imread("../test/map.png",cv::IMREAD_GRAYSCALE);
    cv::resize(map,map,cv::Size(map.cols/25,map.rows/25));
    Planner planner(map.data,map.cols ,map.rows);
    while (1)
    {
        cv::Mat show = map.clone();
        Node3D start(start_x,start_y,(float)start_angle*0.1+1.708f,0,0, nullptr);
        Node3D goal(end_x,end_y,(float)end_angle*0.1+1.708f,0,0, nullptr);
        planner.setStart(start);
        planner.setGoal(goal);
        planner.plan();

        auto path = planner.getPath();
        auto smooth_path = planner.getSmoothPath();
        for(auto pt:path)
        {
            int x = (int)pt.getX();
            int y = (int)pt.getY();
            show.at<uchar>(x, y) = 120;
        }
        int last_x = -1, last_y = -1;
        for(auto pt:smooth_path)
        {
            int x = (int)pt.getX();
            int y = (int)pt.getY();
            show.at<uchar>(x, y) = 0;
            if(pt.getPrim() == -2)
            {
                show.at<uchar>(x+1, y) = 0;
                show.at<uchar>(x, y+1) = 0;
                show.at<uchar>(x+1, y+1) = 0;
            }
            if(last_x != -1 && last_y != -1)
            {
                float dx = x - last_x;
                float dy = y - last_y;
                //cv::line(map,cv::Point(last_x-2*dx,last_y-2*dy),cv::Point(x,y),cv::Scalar(0,0,0));
            }
            last_x = x;
            last_y = y;
        }
        cv::createTrackbar("start","result",&start_angle,100);
        cv::createTrackbar("end","result",&end_angle,100);
        cv::createTrackbar("end x","result",&end_x,800);
        cv::createTrackbar("end y","result",&end_y,800);
        cv::createTrackbar("start x","result",&start_x,800);
        cv::createTrackbar("start y","result",&start_y,800);
        cv::resize(show,show,cv::Size(show.cols*3,show.rows*3));
        cv::imshow("result",show);
        cv::waitKey(10);
    }

}
