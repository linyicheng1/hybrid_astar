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
    int show_id = 0;
    while (true)
    {
        cv::Mat map = cv::imread("../test/write.png",cv::IMREAD_GRAYSCALE);
        ReedsShepp planner(50);
        ReedsShepp::pos start(20,20,(float)(start_angle-50)*0.1+1.5708);
        ReedsShepp::pos goal((float)end_x,(float)end_y,(float)(end_angle-50)*0.1+1.5708);
        auto path = planner.plan(start,goal);

        for(int i = 0;i < 500;i ++)
        {
            float t = (float)i / 500.f;
            ReedsShepp::pos p;
            planner.interpolate(&start,path,t,&p);
            ReedsShepp::pos p1;
            //if(i > 1)
            {
                float t1 = ((float)i+0.1) / 500.f;
                planner.interpolate(&start,path,t1,&p1);
            }
            float d_x = (-p1.x + p.x );
            float angle = atan(d_x / (p1.y - p.y));
            if(d_x > 0)
            {
                if(angle < 0)
                {
                    angle += 3.14159;
                }
            }
            else
            {
                if(angle > 0)
                {
                    angle -= 3.14159;
                }
            }
            float angle2 = p.angle;

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


            float a = angle2 + 1.5708;
            if(fabs(a-angle) <0.1)
            {
//                map.at<uchar>(x, y) = 0;
//                //angle2 = -angle2;
                angle += 3.14159;
            }
            map.at<uchar>(x, y) = 0;
            if(i == show_id)
            {
                float test = atan((-p1.x + p.x )/ (p1.y - p.y));
                cv::Point dir(y+50*cos(a),x-50*sinf(a));
                //cv::line(map,cv::Point(y,x),dir,cv::Scalar(0,0,0));
                dir = cv::Point(y+50*cos(angle),x-50*sinf(angle));
                cv::line(map,cv::Point(y,x),dir,cv::Scalar(0,0,0));
                cv::circle(map,cv::Point(y,x),i/10,cv::Scalar(0,0,0));
            }
        }
        //cv::imwrite("../pic/RS.png",map);
        cv::createTrackbar("end x","test",&end_x,100);
        cv::createTrackbar("end y","test",&end_y,100);
        cv::createTrackbar("start angle","test",&start_angle,100);
        cv::createTrackbar("end angle","test",&end_angle,100);
        cv::createTrackbar("id","test",&show_id,500);
        cv::imshow("test",map);
        cv::waitKey(10);
    }

}

