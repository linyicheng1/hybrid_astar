#include "hybridAStar.h"
#include <boost/heap/binomial_heap.hpp>
#include <iostream>

using namespace  HybridAStar;

struct CompareNodes {
    /// Sorting 3D nodes by increasing C value - the total estimated cost
    bool operator()(const Node3D* lhs, const Node3D* rhs) const {
        return lhs->getC() > rhs->getC();
    }
    /// Sorting 2D nodes by increasing C value - the total estimated cost
    bool operator()(const Node2D* lhs, const Node2D* rhs) const {
        return lhs->getC() > rhs->getC();
    }
};
hybridAStar::hybridAStar(CollisionDetection *map):m_map(map)
{
    m_nodes2D = new Node2D[map->getSize()];
}

hybridAStar::~hybridAStar()
{
    delete m_nodes2D;
}

HybridAStar::Node3D *HybridAStar::hybridAStar::search_planer(bool reverse)
{
    return nullptr;
}

/**
 * @brief 原始A*算法，用来搜索计算 holonomic-with-obstacles heuristic
 * @param start
 * @param goal
 * @param nodes2D
 * @return
 */
float HybridAStar::hybridAStar::aStar(Node2D &start, Node2D &goal)
{
    int iPred, iSucc;
    float newG;
    for (int i = 0; i < m_map->getSize(); ++i)
    {
        m_nodes2D[i].reset();
    }
    boost::heap::binomial_heap<Node2D*,
            boost::heap::compare<CompareNodes>> open;//Open list
    start.updateH(goal);
    // mark start as open
    start.open();
    // push on priority queue
    open.push(&start);
    int width = m_map->getWidthSize();
    int height = m_map->getHeightSize();
    iPred = start.setIdx(width);
    m_nodes2D[iPred] = start;
    Node2D* nPred;
    Node2D* nSucc;
    while (!open.empty())
    {
        nPred = open.top();//从Open集合中找出代价最低的元素
        // set index
        iPred = nPred->setIdx(width);//相应的index
        if (m_nodes2D[iPred].isClosed())
        {//检查：如果已扩展，则从open set中移除，处理下一个
            // pop node from the open list and start with a fresh node
            open.pop();
            continue;
        }
        m_nodes2D[iPred].close();//标记为close
        m_nodes2D[iPred].discover();
        open.pop();

        // 目标点,返回G值
        if(*nPred == goal)
        {
            goal.setPred(nPred);
            return nPred->getG();
        }
        // 非目标点，则从可能的方向寻找
        for (int i = 0; i < Node2D::dir; i++)
        {//A*算法是8个方向：4个正方向和4个45度的方向
            nSucc = nPred->createSuccessor(i);
            iSucc = nSucc->setIdx(width);
            // 约束性检查：在有效网格范围内、且不是障碍、没有扩展过
            if (nSucc->isOnGrid(width, height) &&  m_map->isTraversable(nSucc) && !m_nodes2D[iSucc].isClosed())
            {
                // calculate new G value
                //更新G值
                nSucc->updateG();
                newG = nSucc->getG();

                // if successor not on open list or g value lower than before put it on open list
                // 如果子节点并在open集中，或者它的G值(cost-so-far)比之前要小，则为可行的方向
                if (!m_nodes2D[iSucc].isOpen() || newG < m_nodes2D[iSucc].getG())
                {
                    // calculate the H value
                    nSucc->updateH(goal);//计算H值
                    // put successor on open list
                    nSucc->open();//将该点移到open set中
                    m_nodes2D[iSucc] = *nSucc;
                    m_nodes2D[iSucc].setPred(nPred);
                    open.push(&m_nodes2D[iSucc]);
                    delete nSucc;
                }
                else
                {
                    delete nSucc;
                }
            }
            else
            {
                delete nSucc;
            }
        }
    }
    return 1000;
}



void hybridAStar::updateH(Node3D &start, const Node3D &goal, Node2D *nodes2D, float *dubinsLookup)
{

}

