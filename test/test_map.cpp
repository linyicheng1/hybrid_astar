#include "CollisionDetection.h"
#include <opencv2/opencv.hpp>
#include "node.h"

using namespace HybridAStar;

int main()
{
    cv::Mat map = cv::imread("../test/map.png",cv::IMREAD_GRAYSCALE);
    CollisionDetection map_data(map.data,map.cols ,map.rows);

    for(int x = 0;x < map.rows;x += 50)
    {
        for(int y = 0;y < map.cols;y += 50)
        {
            auto node = new Node2D(x,y,0,0, nullptr);
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


