#include "ReedsSheppPath.h"
#include <opencv2/opencv.hpp>
using namespace HybridAStar;
#define PI 3.14159
int main()
{
    int end_x = 50;
    int end_y = 30;
    int start_angle = 50;
    int end_angle = 50;
    while (true)
    {
        cv::Mat map = cv::imread("../test/write.png",cv::IMREAD_GRAYSCALE);
        ReedsShepp planner(10);
        ReedsShepp::pos start(10,1,(float)(start_angle-50)*0.1+1.5708);
        ReedsShepp::pos goal((float)end_x,(float)end_y,(float)(end_angle-50)*0.1+1.5708);
        auto path = planner.plan(start,goal);

        for(int i = 0;i < 500;i ++)
        {
            float t = (float)i / 500.f;
            ReedsShepp::pos p;
            planner.interpolate(&start,path,t,&p);
            ReedsShepp::pos p1;
            if(i > 1)
            {
                float t1 = (float)(i-1) / 500.f;
                planner.interpolate(&start,path,t1,&p1);
            }

            float angle = atan2(p.y - p1.y,p.x - p1.x);
            if(angle < -PI)
                angle += 2*PI;
            else if(angle > PI)
                angle -= 2*PI;

            float angle2 = p.angle;
            if(angle2 < -PI)
                angle2 += 2*PI;
            else if(angle2 > PI)
                angle2 -= 2*PI;

            //std::cout<<"pos: x "<<p.x<<" y "<<p.y<<" angle "<<angle<<" angle2 "<<angle2<<std::endl;
            int x = (int)(p.x * 10);
            int y = (int)(p.y * 10);
            float dx = 50*cos(p.angle);
            float dy = 50*sin(p.angle);
            for(int j = 0;j < 50;j ++)
            {
                int x1 = (int)((float)x + dx * (float)j / 50.f);
                int y1 = (int)((float)y + dy * (float)j / 50.f);
                //map.at<uchar>(y1, x1) = 125;
            }
            if(fabs(angle-angle2)>0.2)
                map.at<uchar>(x, y) = 0;
            else
                map.at<uchar>(x, y) = 125;
        }
        //cv::imwrite("../pic/RS.png",map);
        cv::createTrackbar("end x","test",&end_x,100);
        cv::createTrackbar("end y","test",&end_y,100);
        cv::createTrackbar("start angle","test",&start_angle,100);
        cv::createTrackbar("end angle","test",&end_angle,100);
        cv::imshow("test",map);
        cv::waitKey(10);
    }

}

