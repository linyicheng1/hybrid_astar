#include "CollisionDetection.h"
#include <opencv2/opencv.hpp>
#include "node.h"
#include "hybridAStar.h"
using namespace HybridAStar;

int main()
{
    cv::Mat map = cv::imread("../test/map.png",cv::IMREAD_GRAYSCALE);
    cv::resize(map,map,cv::Size(map.cols/10,map.rows/10));
    CollisionDetection map_data(map.data,map.cols/10  ,map.rows/10 ,10);
    hybridAStar planer(&map_data);
    Node2D start(50,50,0,0, nullptr);
    Node2D goal(650,750,0,0, nullptr);
    planer.aStar(start,goal,0.05);
    // show
    Node2D* ptr = goal.getPred();
    while (ptr != nullptr)
    {
        int x = ptr->getX();
        int y = ptr->getY();
        map.at<uchar>(y, x) = 0;
        ptr = ptr->getPred();
        std::cout<<"G: "<<ptr->getG()<<std::endl;
    }
    //cv::resize(map,map,cv::Size(map.cols/10,map.rows/10));
    cv::imshow("result",map);
    cv::waitKey(0);
}


