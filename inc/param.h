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
    static const float tieBreaker = 0.01;
    static const int positionResolution = 10;//每个cell里的离散位置数量的平方根
    static const int positions = positionResolution * positionResolution;//位置的数量
    struct relPos {//相对于中心的位置：即以中心为坐标原点
        /// the x position relative to the center
        int x;
        /// the y position relative to the center
        int y;
    };
    /// A structure capturing the lookup for each theta configuration
    struct config {//用以获取每个theta的查找表的结构体
        /// the number of cells occupied by this configuration of the vehicle
        int length;//长度，
        /*!
           \var relPos pos[64]
           \brief The maximum number of occupied cells
           \todo needs to be dynamic
        */
        relPos pos[64];//这里为什么是64有待考证
    };
    static const float factor2D = sqrt(5) / sqrt(2) + 1;
/// [#] --- A movement cost penalty for turning (choosing non straight motion primitives)
    static const float penaltyTurning = 1.05;
/// [#] --- A movement cost penalty for reversing (choosing motion primitives > 2)
    static const float penaltyReversing = 2.0;
/// [#] --- A movement cost penalty for change of direction (changing from primitives < 3 to primitives > 2)
    static const float penaltyCOD = 2.0;
    static const float dubinsShotDistance = 100;
    static const float minRoadWidth = 2;
}
#endif //HYBRID_ASTAR_PARAM_H
