//#ifndef HYBRID_ASTAR_SMOOTH_H
//#define HYBRID_ASTAR_SMOOTH_H
//
//#include <cmath>
//#include <vector>
//#include <iostream>
//#include "dynamicVoronoi.h"
//#include "node.h"
//
//namespace HybridAStar
//{
//    /// A class describing a simple 2D vector
//    class Vector2D {
//    public:
//        /// default constructor
//        inline Vector2D(const float x = 0, const float y = 0) { this->x = x; this->y = y; }
//        /// a method to multiply a vector by a scalar
//        inline Vector2D operator * (const float k) const { return Vector2D(x * k, y * k); }
//        /// a method to divide a vector by a scalar
//        inline Vector2D operator / (const float k) const { return Vector2D(x / k, y / k); }
//        /// a method to add a vector to a vector
//        inline Vector2D operator + (const Vector2D& b) const { return Vector2D(x + b.x, y + b.y); }
//        /// a method to subtract a vector from a vector
//        inline Vector2D operator - (const Vector2D& b) const { return Vector2D(x - b.x, y - b.y); }
//        /// a method to negate a vector
//        inline Vector2D operator - () const  {return Vector2D(-x, -y);}
//        /// a convenience method to print a vector
//        friend std::ostream& operator<<(std::ostream& os, const Vector2D& b) {os << "(" << b.x << "|" << b.y << ")"; return os; }
//        /// a method to calculate the length of the vector
//        float length() const { return std::sqrt(std::pow(x, 2) + std::pow(y, 2)); }
//        /// a method to calculate the length of the vector
//        float sqlength() const { return x*x + y*y; }
//        /// a method to calculate the dot product of two vectors
//        float dot(Vector2D b) { return x * b.x + y * b.y; }
//        ///a method that returns the orthogonal complement of two vectors
//        inline Vector2D ort(Vector2D b) {
//            Vector2D a(this->x, this->y);
//            Vector2D c;
//            // multiply b by the dot product of this and b then divide it by b's length
//            c = a - b * a.dot(b) / b.sqlength();
//            return c;
//        }
//        inline float getX() { return x; }
//        inline float getY() { return y; }
//        //  void setT(float t) { this->t = t; }
//        //  float getT() { return t; }
//    private:
//        /// the x part of the vector
//        float x;
//        /// the y part of the vector
//        float y;
//        //  /// the theta part for plotting purposes
//        //  float t;
//    };
//    inline Vector2D operator * (double k, const Vector2D& b) {
//        return (b * k);
//    }
//
//    class Smoother
//    {
//    public:
//        Smoother() {}
//        /*!
//           \brief This function takes a path consisting of nodes and attempts to iteratively smooth the same using gradient descent.
//           核心函数，将由节点组成的路径采用梯度下降方法进行平滑
//           During the different interations the following cost are being calculated
//           不同的迭代阶段采用计算下面的代价值作为指标
//           obstacleCost：障碍物代价
//           curvatureCost：曲率代价
//           smoothnessCost：平滑代价
//           voronoiCost: Voronoi代价 (实际计算时没有这一项)
//        */
//        void smoothPath(DynamicVoronoi& voronoi);
//
//        /*!
//           \brief Given a node pointer the path to the root node will be traced recursively
//           \param node a 3D node, usually the goal node
//           \param i a parameter for counting the number of nodes
//           给定一个点的指针，递归地跟踪该点到根节点的路径，其中：
//           参数node：为一个3D节点类，通常是一个目标节点
//           参数i：对节点进行计数的参数
//        */
//        void tracePath(const Node3D* node, int i = 0, std::vector<Node3D> path = std::vector<Node3D>());
//
//        /// returns the path of the smoother object：返回平滑后的路径
//        std::vector<Node3D> getPath() {return path;}
//
//        /// obstacleCost - pushes the path away from obstacles
//        Vector2D obstacleTerm(Vector2D xi);//障碍物项，用于约束路径远离障碍物
//
//        /// curvatureCost - forces a maximum curvature of 1/R along the path ensuring drivability
//        Vector2D curvatureTerm(Vector2D xi0, Vector2D xi1, Vector2D xi2);//曲率项，用于保证可转弯性及通行性
//
//        /// smoothnessCost - attempts to spread nodes equidistantly and with the same orientation
//        //平滑项，用于将节点等距分布并尽量保持同一个方向
//        Vector2D smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2);
//
//        /// voronoiCost - trade off between path length and closeness to obstaclesg
//        //   Vector2D voronoiTerm(Vector2D xi);
//
//        /// a boolean test, whether vector is on the grid or not
//        bool isOnGrid(Vector2D vec) {
//            if (vec.getX() >= 0 && vec.getX() < width &&
//                vec.getY() >= 0 && vec.getY() < height) {
//                return true;
//            }
//            return false;
//        }
//
//    private:
//        /// maximum possible curvature of the non-holonomic vehicle
//        float kappaMax = 1.f / (param::r * 1.1);
//        /// maximum distance to obstacles that is penalized
//        float obsDMax = param::minRoadWidth;
//        /// maximum distance for obstacles to influence the voronoi field
//        float vorObsDMax = param::minRoadWidth;
//
//        //权重系数
//        /// falloff rate for the voronoi field
//        float alpha = 0.1;
//        /// weight for the obstacle term
//        float wObstacle = 0.2;
//        /// weight for the voronoi term
//        float wVoronoi = 0;
//        /// weight for the curvature term
//        float wCurvature = 0;
//        /// weight for the smoothness term
//        float wSmoothness = 0.2;
//
//        // 描述地图中拓扑结构的voronoi diagram
//        /// voronoi diagram describing the topology of the map
//        DynamicVoronoi voronoi;
//
//        //地图宽度与高度
//        /// width of the map
//        int width;
//        /// height of the map
//        int height;
//
//        //待平滑的路径
//        /// path to be smoothed
//        std::vector<Node3D> path;
//    };
//}
//#endif //HYBRID_ASTAR_SMOOTH_H
