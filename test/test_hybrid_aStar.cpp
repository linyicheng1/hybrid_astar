#include "CollisionDetection.h"
#include <opencv2/opencv.hpp>
#include "node.h"
#include "hybridAStar.h"
using namespace HybridAStar;

int main()
{
    cv::Mat map = cv::imread("../test/map.png",cv::IMREAD_GRAYSCALE);
    int inv_resolution = 1;
    cv::resize(map,map,cv::Size(map.cols/50,map.rows/50));
    cv::Mat raw = map.clone();
    CollisionDetection map_data(map.data,map.cols ,map.rows );
    hybridAStar planer(&map_data);
    Node3D start(4,4,0,0,0, nullptr);
    Node3D goal(130,125,0,0,0, nullptr);
//    for(int i = -1;i < 2; i ++)
//    {
//        for(int j = -1;j < 2;j ++)
//        {
//            int x = (int)start.getX() * inv_resolution + i;
//            int y = (int)start.getY() * inv_resolution + j;
//            map.at<uchar>(y, x) = 0;
//            x = (int)goal.getX() * inv_resolution + i;
//            y = (int)goal.getY() * inv_resolution + j;
//            map.at<uchar>(y, x) = 0;
//        }
//    }
//    cv::imshow("set",map);
//    cv::waitKey(0);

    planer.search_planer(start,goal,0.2);
    // show
    Node3D* ptr = goal.getPred();
    if(planer.isShootSuccess())
    {
        ptr = goal.getPred();
        for(int i = 0;i < 500;i ++)
        {
            float t = (float)i / 500.f;
            ReedsShepp::pos p;
            ReedsShepp::pos st(ptr->getX(),ptr->getY(),ptr->getT());
            planer.interpolate(&st,t,&p);
            //std::cout<<"pos: x "<<p.x<<" y "<<p.y<<" angle "<<p.angle<<std::endl;
            int x = (int)(p.x*inv_resolution);
            int y = (int)(p.y*inv_resolution);
            map.at<uchar>(y, x) = 120;
        }
    }
    while(ptr != nullptr)
    {
        int x = (int)ptr->getX()*inv_resolution;
        int y = (int)ptr->getY()*inv_resolution;
        map.at<uchar>(y, x) = 0;
        if(raw.at<uchar>(y, x) < 250)
        {
            std::cerr<<" error !! "<<(int)raw.at<uchar>(y, x)<<" pos: x "<<x<<" y "<<y<<" angle "<<ptr->getT()<<std::endl;
        }
        //std::cout<<"pos: x "<<x<<" y "<<y<<" angle "<<ptr->getT()<<std::endl;
        ptr = ptr->getPred();
        if(ptr != nullptr)
        {
            int next_x = (int)ptr->getX()*inv_resolution;
            int next_y = (int)ptr->getY()*inv_resolution;
            cv::line(map,cv::Point(x,y),cv::Point(next_x,next_y),cv::Scalar(0,0,0));
        }
    }
    //cv::imwrite("../pic/hybridAStar.png",map);
    //cv::resize(map,map,cv::Size(map.cols*3,map.rows*3));
    cv::imshow("result",map);
    cv::waitKey(0);
}



