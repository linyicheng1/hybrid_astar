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
    m_nodes3D = new Node3D[map->getSize()*param::headings];
}

hybridAStar::~hybridAStar()
{
    delete m_nodes2D;
}

/**
 * @brief hybrid A*
 * @param start
 * @param goal
 * @param reverse
 * @return
 */
HybridAStar::Node3D *HybridAStar::hybridAStar::search_planer(Node3D& start, Node3D& goal,float scale)
{
    int iPred, iSucc;
    float newG;
    for (int i = 0; i < m_map->getSize()*param::headings; ++i)
    {
        m_nodes3D[i].reset();
    }
    int dir = 6;
    int iterations = 0;//迭代计数
    m_shootSuccess = false;
    // OPEN LIST AS BOOST IMPLEMENTATION
    typedef boost::heap::binomial_heap<Node3D*,
            boost::heap::compare<CompareNodes>
    > priorityQueue;
    priorityQueue open;//open集
    updateH(start, goal);
    start.open();//将start加入open 集合: 1)将点标记为open
    open.push(&start);
    iPred = start.setIdx(m_map->getWidthSize(), m_map->getHeightSize()); //计算索引位置
    m_nodes3D[iPred] = start;
    float distance = (start.getX() - goal.getX()) * (start.getX() - goal.getX()) + (start.getY() - goal.getY()) * (start.getY() - goal.getY());
    // NODE POINTER
    Node3D* nPred;
    Node3D* nSucc;

    while (!open.empty())
    {
        nPred = open.top();
        //std::cout<<" point: x "<<nPred->getX()<<" y "<<nPred->getY()<<" T "<<nPred->getT()<<std::endl;
        // set index
        iPred = nPred->setIdx(m_map->getWidthSize(), m_map->getHeightSize());//获取该点在nodes3D的索引 (前缀i表示index, n表示node)
        iterations++;//记录迭代次数
        if (m_nodes3D[iPred].isClosed())
        {//检查该点是否closed状态
            // pop node from the open list and start with a fresh node
            open.pop();
            continue;//如果为closed，说明该点已经处理过，忽略(将它从open set中移除)
        }
        else if(m_nodes3D[iPred].isOpen())
        {
            m_nodes3D[iPred].close();
            open.pop();
            //检测当前节点是否是终点或者是否超出解算的最大时间(最大迭代次数)
            if (*nPred == goal || iterations > param::iterations)
            {
                goal.setPred(nPred);
                return nPred;
            }
            // shot
            float current_distance = (nPred->getX() - goal.getX()) * (nPred->getX() - goal.getX()) + (nPred->getY() - goal.getY()) * (nPred->getY() - goal.getY());
            if(current_distance < 0.1 * distance)
            {
                m_RS_path = m_RS.plan(ReedsShepp::pos(nPred->getX(),nPred->getY(),nPred->getT()),
                                      ReedsShepp::pos(goal.getX(),goal.getY(),goal.getT()));
                // is shot success
                if(m_RS.isTraversable(nPred,&m_RS_path,m_map))
                {
                    m_shootSuccess = true;
                    goal.setPred(nPred);
                    bool fix[4] = {false};
                    for(int i = 1;i < 5;i ++)
                    {
                        if((m_RS_path.length_[i-1] > 0 && m_RS_path.length_[i] < 0) ||
                           (m_RS_path.length_[i-1] < 0 && m_RS_path.length_[i] > 0))
                        {
                            fix[i-1] = true;
                        }
                    }
                    int seg_num = 0;
                    auto seg_length = (float)abs(m_RS_path.length_[0]);
                    bool last_fix = false;
                    for(int i = 0;i < m_RS_path.length()*2;i ++)
                    {
                        int prim = -1;
                        float t = (float)i / (m_RS_path.length()*2);
                        ReedsShepp::pos p;
                        ReedsShepp::pos st(goal.getPred()->getX(),goal.getPred()->getY(),goal.getPred()->getT());
                        interpolate(&st,t,&p);
                        if(t > seg_length / m_RS_path.length())
                        {
                            if(fix[seg_num])
                            {
                                prim = -2;
                                nPred->setPrim(-2);
                                last_fix = true;
                            }
                            seg_num ++;
                            seg_length += (float)abs(m_RS_path.length_[seg_num]);
                        }
                        if(last_fix && prim != -2)
                        {
                            prim = -2;
                            last_fix = false;
                        }
                        auto tmp_node = new Node3D(p.x,p.y,p.angle,0,0,nPred,prim);
                        nPred = tmp_node;
                    }
                    return nPred;
                }
            }
            for (int i = 0; i < 6; i++)
            {//每个方向都搜索
                nSucc = nPred->createSuccessor(i,1/scale);//找到下一个点
                iSucc = nSucc->setIdx(m_map->getWidthSize(), m_map->getHeightSize());//索引值

                if (nSucc->isOnGrid(m_map->getWidth(), m_map->getHeight())
                && m_map->isTraversable(nSucc))
                {
                    // 确定新扩展的节点不在close list中，或者没有在之前遍历过
                    if (!m_nodes3D[iSucc].isClosed() || iPred == iSucc)
                    {
                        // 更新合格点的G值
                        nSucc->updateG();
                        newG = nSucc->getG();
                        if (!m_nodes3D[iSucc].isOpen() || newG < m_nodes3D[iSucc].getG() || iPred == iSucc)
                        {
                            // calculate H value: 更新cost-to-go
                            updateH(*nSucc, goal);

                            // if the successor is in the same cell but the C value is larger
                            if (iPred == iSucc && nSucc->getC() > nPred->getC() + param::tieBreaker)
                            {
                                delete nSucc;//如果下一个点仍在相同的cell、并且cost变大，那继续找
                                continue;
                            }
                            // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                            // 如果下一节点仍在相同的cell, 但是代价值要小，则用当前successor替代前一个节点（这里仅更改指针，数据留在内存中）
                            else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + param::tieBreaker)
                            {
                                nSucc->setPred(nPred->getPred());//如果下一个点仍在相同的cell、并且cost变小，成功
                            }
                            if (nSucc->getPred() == nSucc)
                            {
                                std::cout << "looping";//给出原地踏步的提示
                            }
                            // put successor on open list
                            nSucc->open();
                            m_nodes3D[iSucc] = *nSucc;//将生成的子节点加入到open list中
                            open.push(&m_nodes3D[iSucc]);
                            delete nSucc;
                        }
                    }
                }
            }
        }
    }
    return nullptr;
}

/**
 * @brief 原始A*算法，用来搜索计算 holonomic-with-obstacles heuristic
 * @param start
 * @param goal
 * @param nodes2D
 * @return
 */
float HybridAStar::hybridAStar::aStar(Node2D &start, Node2D &goal, float scale)
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
        else if(m_nodes2D[iPred].isOpen())
        {
            m_nodes2D[iPred].close();//标记为close
            m_nodes2D[iPred].discover();
            open.pop();
            // 目标点,返回G值
            if (nPred->equal(goal,scale))
            {
                goal.setPred(nPred);
                return nPred->getG();
            }
            // 非目标点，则从可能的方向寻找
            for (int i = 0; i < Node2D::dir; i++)
            {//A*算法是8个方向：4个正方向和4个45度的方向
                nSucc = nPred->createSuccessor(i, 1 / scale);
                iSucc = nSucc->setIdx(width);
                // 约束性检查：在有效网格范围内、且不是障碍、没有扩展过
                if (nSucc->isOnGrid(width, height)
                && m_map->isTraversable(nSucc)
                && !m_nodes2D[iSucc].isClosed())
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
    }
    return 1000;
}

void hybridAStar::updateH(Node3D &start, const Node3D &goal)
{
    float reedsSheppCost = 0;
    float twoDCost = 0;
    float twoDoffset = 0;
    // 1、reedsSheppCost
    ReedsShepp::pos start_pt(start.getX(),start.getY(),start.getT());
    ReedsShepp::pos goal_pt(goal.getX(),goal.getY(),goal.getT());
    reedsSheppCost = (float)m_RS.distance(start_pt,goal_pt);
    float s_x = start.getX();
    float s_y = start.getY();
    float g_x = goal.getX();
    float g_y = goal.getY();
//    // 2、A *
//    Node2D start2d((int)s_x, (int)s_y, 0, 0, nullptr);
//    Node2D goal2d((int)g_x, (int)g_y, 0, 0, nullptr);
//    float G = aStar(start2d,goal2d,0.1);
//    // 3、
//    twoDoffset = sqrt( ((s_x - (float)((int)s_x)) - (g_x - (float)((int)g_x))) *
//                       ((s_x - (float)((int)s_x)) - (g_x - (float)((int)g_x))) +
//                       ((s_y - (float)((int)s_y)) - (g_y - (float)((int)g_y))) *
//                       ((s_y - (float)((int)s_y)) - (g_y - (float)((int)g_y)))
//    );
//    twoDCost = (G - twoDoffset);
    start.setH(std::max(reedsSheppCost, twoDCost));
}

void hybridAStar::interpolate(const ReedsShepp::pos *from, double t, ReedsShepp::pos *state) const
{
    m_RS.interpolate(from,m_RS_path,t,state);
}

