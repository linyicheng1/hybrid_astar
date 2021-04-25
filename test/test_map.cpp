#include "CollisionDetection.h"
#include <opencv2/opencv.hpp>
#include "node.h"

using namespace HybridAStar;

int main()
{
    cv::Mat map = cv::imread("../test/map.png",cv::IMREAD_GRAYSCALE);
    CollisionDetection map_data(map.data,map.cols / 100 ,map.rows / 100,100);

    for(int j = 0;j < map.rows;j += 50)
    {
        for(int i = 0;i < map.cols;i += 50)
        {
            auto node = new Node2D(i,j,0,0, nullptr);
            node->setIdx(map.cols);
            if(map_data.isTraversable(node))
            {
                std::cout<<" ";
            }
            else
            {
                std::cout<<"*";
            }
        }
        std::cout<<std::endl;
    }
}


