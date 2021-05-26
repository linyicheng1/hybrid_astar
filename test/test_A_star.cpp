#include "CollisionDetection.h"
#include <opencv2/opencv.hpp>
#include "node.h"
#include "hybridAStar.h"
using namespace HybridAStar;

int main()
{
    cv::Mat map = cv::imread("../test/map.png",cv::IMREAD_GRAYSCALE);
    int inv_resolution = 10;
    cv::resize(map,map,cv::Size(map.cols/50,map.rows/50));
    CollisionDetection map_data(map.data,map.cols  ,map.rows);
    hybridAStar planer(&map_data);
    Node2D start(10,10,0,0, nullptr);
    Node2D goal(150,150,0,0, nullptr);
    planer.aStar(start,goal,0.05);
    // show
    Node2D* ptr = goal.getPred();

    while (ptr != nullptr)
    {
        int x = ptr->getX();
        int y = ptr->getY();
        static int last_x = x;
        static int last_y = y;
        cv::line(map,cv::Point(y,x),cv::Point(last_y,last_x),cv::Scalar(0,0,0));
        last_x = x;
        last_y = y;
        map.at<uchar>(x, y) = 0;
        std::cout<<"x "<<x<<" y "<<y;
        std::cout<<"G: "<<ptr->getG()<<std::endl;
        ptr = ptr->getPred();
    }
    //cv::resize(map,map,cv::Size(map.cols/10,map.rows/10));
    cv::imshow("result",map);
    cv::waitKey(0);
}


