#include "CollisionDetection.h"
#include <opencv2/opencv.hpp>
#include "node.h"
#include "hybridAStar.h"
using namespace HybridAStar;

int main()
{
    cv::Mat map = cv::imread("../test/map.jpg",cv::IMREAD_GRAYSCALE);
    cv::Mat show1 = map.clone();
    cv::flip(map, map, -1);

    int inv_resolution = 10;
    cv::resize(map,map,cv::Size(map.cols,map.rows));
    CollisionDetection map_data(map.data,map.cols  ,map.rows);
    hybridAStar planer(&map_data);
    int start_x = 10,start_y = 10;
    int end_x = 150; int end_y = 180;

    int width = map.cols;
    int height = map.rows;
    while (1)
    {
        cv::Mat show = show1.clone();
        Node2D start(start_x,start_y,0,0, nullptr);
        Node2D goal(end_x,end_y,0,0, nullptr);
        planer.aStar(start,goal,0.05);
        // show
        Node2D* ptr = goal.getPred();
        while (ptr != nullptr)
        {
            int x = ptr->getX();
            int y = ptr->getY();
            static int last_x = x;
            static int last_y = y;
            cv::line(show,cv::Point(width-y,height-x),cv::Point(width-last_y,height-last_x),cv::Scalar(0,0,0));
            last_x = x;
            last_y = y;
            show.at<uchar>(height-x, width-y) = 0;
            //std::cout<<"x "<<x<<" y "<<y;
            //std::cout<<"G: "<<ptr->getG()<<std::endl;
            ptr = ptr->getPred();
        }
        //cv::resize(map,map,cv::Size(map.cols/10,map.rows/10));
        cv::circle(show,cv::Point(width-start_y,height-start_x),3,cv::Scalar(0,0,0));
        cv::circle(show,cv::Point(width-end_y,height-end_x),3,cv::Scalar(0,0,0));
        cv::createTrackbar("start","result",&start_x,800);
        cv::createTrackbar("end","result",&start_y,800);
        cv::createTrackbar("end x","result",&end_x,800);
        cv::createTrackbar("end y","result",&end_y,800);
        cv::imshow("result",show);
        cv::waitKey(10);
    }

}


