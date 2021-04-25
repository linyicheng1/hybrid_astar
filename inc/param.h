#ifndef HYBRID_ASTAR_PARAM_H
#define HYBRID_ASTAR_PARAM_H
namespace param {
/// [#] --- Limits the maximum search depth of the algorithm, possibly terminating without the solution
    static const int iterations = 30000; //最大迭代次数
/// [m] --- Uniformly adds a padding around the vehicle
    static const double bloating = 0; //膨胀范围
/// [m] --- The width of the vehicle
    static const double width = 1.75 + 2 * bloating;//车的宽度
/// [m] --- The length of the vehicle
    static const double length = 2.65 + 2 * bloating;//车的长度
/// [m] --- The minimum turning radius of the vehicle
    static const float r = 6;//最小转弯半径
/// [m] --- The number of discretizations in heading
    static const int headings = 72;//车体朝向的离散数量
/// [°] --- The discretization value of the heading (goal condition)
    static const float deltaHeadingDeg = 360 / (float) headings; //朝向离散步长(以度表示)
/// [c*M_PI] --- The discretization value of heading (goal condition)
    static const float deltaHeadingRad = 2 * M_PI / (float) headings; //朝向离散步长(以弧度表示)
/// [c*M_PI] --- The heading part of the goal condition
    static const float deltaHeadingNegRad = 2 * M_PI - deltaHeadingRad;
/// [m] --- The cell size of the 2D grid of the world
    static const float cellSize = 1; //在2D网格中cell的大小

}
#endif //HYBRID_ASTAR_PARAM_H
