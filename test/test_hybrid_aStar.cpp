#include "CollisionDetection.h"
#include <opencv2/opencv.hpp>
#include "node.h"
#include "hybridAStar.h"
using namespace HybridAStar;

int main()
{
    cv::Mat map = cv::imread("../test/map.png",cv::IMREAD_GRAYSCALE);
    int inv_resolution = 1;
    cv::resize(map,map,cv::Size(map.cols/25,map.rows/25));
    cv::Mat raw = map.clone();
    CollisionDetection map_data(map.data,map.cols ,map.rows );
    hybridAStar planer(&map_data);

    int start_angle = 0,end_angle = 0;
    int end_x = 96; int end_y = 376;
    while (1)
    {
        cv::Mat show = map.clone();
        Node3D start(8,8,(float)(start_angle)*0.1f + 1.5708,0,0, nullptr);
        Node3D goal(end_x,end_y,(float)(end_angle)*0.1f + 1.5708,0,0, nullptr);
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
                show.at<uchar>(x, y) = 120;
            }
        }
        while(ptr != nullptr)
        {
            int x = (int)ptr->getX()*inv_resolution;
            int y = (int)ptr->getY()*inv_resolution;
            show.at<uchar>(x, y) = 0;
            if(raw.at<uchar>(x, y) < 250)
            {
                //std::cerr<<" error !! "<<(int)raw.at<uchar>(y, x)<<" pos: x "<<x<<" y "<<y<<" angle "<<ptr->getT()<<std::endl;
            }
            //std::cout<<"pos: x "<<x<<" y "<<y<<" angle "<<ptr->getT()<<std::endl;
            ptr = ptr->getPred();
            if(ptr != nullptr)
            {
                int next_x = (int)ptr->getX()*inv_resolution;
                int next_y = (int)ptr->getY()*inv_resolution;
                cv::line(show,cv::Point(y,x),cv::Point(next_y,next_x),cv::Scalar(0,0,0));
            }
        }
        cv::circle(show,cv::Point(end_y,end_x),2,cv::Scalar(0,0,0));
        //cv::circle(show,cv::Point(),2,cv::Scalar(0,0,0));
        //cv::imwrite("../pic/hybridAStar.png",map);
        //cv::resize(map,map,cv::Size(map.cols*3,map.rows*3));
        cv::createTrackbar("start","result",&start_angle,100);
        cv::createTrackbar("end","result",&end_angle,100);
        cv::createTrackbar("end x","result",&end_x,800);
        cv::createTrackbar("end y","result",&end_y,800);
        cv::imshow("result",show);
        cv::waitKey(100);
    }
}



