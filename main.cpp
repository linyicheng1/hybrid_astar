#include <iostream>
#include "planner.h"
#include <opencv2/opencv.hpp>
using namespace HybridAStar;

int main()
{
    int start_x = (int)(25*4),start_y = (int)(4*4),end_x=132,end_y=260;
    int start_angle = 0,end_angle = 0;
    cv::Mat map = cv::imread("../test/map.jpg",cv::IMREAD_GRAYSCALE);
    cv::resize(map,map,cv::Size(map.cols/2,map.rows/2));
    cv::Mat show1 = map.clone();
    threshold(map,map,130,255,cv::THRESH_BINARY);

    //cv::Mat show1 = map.clone();
    cv::flip(map, map, -1);
    Planner planner(map.data,map.cols ,map.rows);
    int width = map.cols;
    int height = map.rows;
    int show_id = 0;
    while (1)
    {
        cv::Mat show = show1.clone();
        Node3D start(start_x,start_y,(float)start_angle*0.1,0,0, nullptr);
        Node3D goal(end_x,end_y,(float)end_angle*0.1,0,0, nullptr);
        planner.setStart(start);
        planner.setGoal(goal);
        planner.plan();

        auto path = planner.getPath();
        auto smooth_path = planner.getSmoothPath();
        int cnt = 0;
        for(auto pt:path)
        {
            cnt ++;
            int x = (int)pt.getX();
            int y = (int)pt.getY();
            show.at<uchar>(x, y) = 120;
            if (cnt == show_id)
            {
                float angle = pt.getT();
                cv::Point dir(width - y - 30*sinf(angle),height - x - 30*cosf(angle));
                cv::line(show,cv::Point(width - y,height - x),dir,cv::Scalar(120,120,120));
            }
            //std::cout<<"pt angle "<<pt.getT()<<std::endl;
        }
        for(auto pt:smooth_path)
        {
            if ((pt.getH()+1) < 0.001 && (pt.getG()+1) < 0.001)
            {
                show.at<uchar>(height - pt.getX(), width - pt.getY()) = 120;
            }
            else if ((pt.getH()-1) < 0.001 && (pt.getG()-1) < 0.001)
            {
                show.at<uchar>(height - pt.getX(), width - pt.getY()) = 0;
            }
        }
        int last_x = -1, last_y = -1;
        cnt = 0;
        for(auto pt:smooth_path)
        {
            cnt ++;
            int x = (int)pt.getX();
            int y = (int)pt.getY();
            show.at<uchar>(height - x, width - y) = 0;
            if(pt.getPrim() == -2)
            {
                show.at<uchar>(height - x+1, width - y) = 0;
                show.at<uchar>(height - x, width - y+1) = 0;
                show.at<uchar>(height - x+1, width - y+1) = 0;
            }
            if(last_x != -1 && last_y != -1)
            {
                //cv::line(show,cv::Point(last_y,last_x),cv::Point(y,x),cv::Scalar(0,0,0));
            }
            if (cnt == show_id)
            {
                float angle = pt.getT();
                cv::Point dir(width - y-30*sinf(angle),height - x-30*cosf(angle));
                cv::line(show,cv::Point(width - y,height - x),dir,cv::Scalar(0,0,0));
            }
            last_x = x;
            last_y = y;
        }
        cv::circle(show,cv::Point(width-start_y,height-start_x),3,cv::Scalar(0,0,0));
        cv::circle(show,cv::Point(width-end_y,height-end_x),3,cv::Scalar(0,0,0));
        cv::createTrackbar("start","result",&start_angle,100);
        cv::createTrackbar("end","result",&end_angle,100);
        cv::createTrackbar("end x","result",&end_x,800);
        cv::createTrackbar("end y","result",&end_y,800);
        cv::createTrackbar("start x","result",&start_x,800);
        cv::createTrackbar("start y","result",&start_y,800);
        cv::createTrackbar("id","result",&show_id,80);
        cv::resize(show,show,cv::Size(show.cols*3,show.rows*3));
        cv::imshow("result",show);
        cv::waitKey(10);
    }
}
