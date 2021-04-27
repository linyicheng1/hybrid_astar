#include "node.h"
using namespace HybridAStar;

static inline float normalizeHeadingRad(float t) {
    if (t < 0) {
        t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
        return 2.f * M_PI + t;
    }
    return t - 2.f * M_PI * (int)(t / (2.f * M_PI));
}

// possible directions
const int Node2D::dir = 8;
// possible movements：8个方向相对中心(0, 0)的移动
const int Node2D::dx[] = { -1, -1, 0, 1, 1, 1, 0, -1 };
const int Node2D::dy[] = { 0, 1, 1, 1, 0, -1, -1, -1 };


//判断点是否在网络内
bool Node2D::isOnGrid(const int width, const int height) const
{
    return  x >= 0 && x < width && y >= 0 && y < height;
}

//创建successor
Node2D* Node2D::createSuccessor(const int i, float inv_scale)
{
    int xSucc = x + (int)((float)Node2D::dx[i] * inv_scale);
    int ySucc = y + (int)((float)Node2D::dy[i] * inv_scale);
    return new Node2D(xSucc, ySucc, g, 0, this);
}

//比较两个node是否相等
bool Node2D::operator == (const Node2D& rhs) const
{
    return x == rhs.x && y == rhs.y;
}

bool Node2D::equal(const Node2D& rhs,float scale) const
{
    float delta_x = std::abs(x - rhs.x);
    float delta_y = std::abs(y - rhs.y);
    return (delta_x < 1 / scale) && (delta_y < 1 / scale);
}

const int Node3D::dir = 6;
const float Node3D::dy[] = { 0,        -0.0415893,  0.0415893};
const float Node3D::dx[] = { 0.7068582,   0.705224,   0.705224};
const float Node3D::dt[] = { 0,         0.1178097,   -0.1178097};

bool Node3D::isOnGrid(const int width, const int height) const
{
    return x >= 0 && x < width && y >= 0 && y < height && (int)(t / param::deltaHeadingRad) >= 0 && (int)(t / param::deltaHeadingRad) < param::headings;
}

bool Node3D::isInRange(const Node3D& goal) const
{
    int random = rand() % 10 + 1;//产生位于[1, 10]的随机数
    float dx = std::abs(x - goal.x) / random;
    float dy = std::abs(y - goal.y) / random;
    return (dx * dx) + (dy * dy) < param::dubinsShotDistance;//距离的平方和在100以内则认为可达
}

Node3D* Node3D::createSuccessor(const int i,float inv_scale)
{
    float xSucc;
    float ySucc;
    float tSucc;

    // calculate successor p ositions forward
    if (i < 3) {//前向 Successor
        xSucc = x + dx[i] * cos(t) * inv_scale - dy[i] * sin(t) * inv_scale;
        ySucc = y + dx[i] * sin(t) * inv_scale + dy[i] * cos(t) * inv_scale;
        tSucc = normalizeHeadingRad(t + dt[i]);
    }
        // backwards
    else {//后向 Successor
        xSucc = x - dx[i - 3] * cos(t) * inv_scale - dy[i - 3] * sin(t) * inv_scale;
        ySucc = y - dx[i - 3] * sin(t) * inv_scale + dy[i - 3] * cos(t) * inv_scale;
        tSucc = normalizeHeadingRad(t - dt[i - 3]);
    }

    return new Node3D(xSucc, ySucc, tSucc, g, 0, this, i);
}

void Node3D::updateG()
{
    // forward driving
    if (prim < 3) {//前进情况
        // penalize turning
        if (pred->prim != prim) {//方向发生改变时
            // penalize change of direction
            if (pred->prim > 2) {
                g += dx[0] * param::penaltyTurning * param::penaltyCOD;//改变方向的惩罚
            } else {
                g += dx[0] * param::penaltyTurning;//没有改变方向
            }
        } else {//方向没有发生变化
            g += dx[0];
        }
    }
    // reverse driving
    else
    {//后退
        // penalize turning and reversing
        if (pred->prim != prim) {
            // penalize change of direction
            if (pred->prim < 3) {
                g += dx[0] * param::penaltyTurning * param::penaltyReversing * param::penaltyCOD;
            } else {
                g += dx[0] * param::penaltyTurning * param::penaltyReversing;
            }
        } else {
            g += dx[0] * param::penaltyReversing;
        }
    }
}

bool Node3D::operator == (const Node3D& rhs) const
{
    return (int)x == (int)rhs.x &&
           (int)y == (int)rhs.y &&
           (std::abs(t - rhs.t) <= param::deltaHeadingRad ||
            std::abs(t - rhs.t) >= param::deltaHeadingNegRad);
}
