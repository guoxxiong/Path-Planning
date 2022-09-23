#include "dijkstra.h"
#include <iostream>

Dijkstra::Dijkstra()
{

}
Dijkstra::Dijkstra(Environment* env, QPointF start, QPointF destination, float radius)
{
    grideSize = env->gride_size;
    //构建全局栅格地图
    min_x = env->barrier[0].x();
    int max_x = env->barrier[0].x();
    min_y = env->barrier[0].y();
    int max_y = env->barrier[0].y();
    for (int i = 0; i < env->barrier.size(); i++) {
        if(env->barrier[i].x() < min_x)  min_x = env->barrier[i].x();
        if(env->barrier[i].x() > max_x)  max_x = env->barrier[i].x();
        if(env->barrier[i].y() < min_y)  min_y = env->barrier[i].y();
        if(env->barrier[i].y() > max_y)  max_y = env->barrier[i].y();
    }
    width = (max_x - min_x) / env->gride_size;
    high = (max_y - min_y) / env->gride_size;

    //设置有障碍的点
    for (auto i : env->barrier) {
        Point point;
        point.x = (i.x() - min_x) / env->gride_size;
        point.y = (i.y() - min_y) / env->gride_size;
        point.value = 10000;
        point.parent = nullptr;
        global_map.insert(unordered_map<int, Point>::value_type(100 * point.y + point.x, point));
    }
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < high; j++) {
             int index = 100 * j + i;
             if(global_map.find(index) == global_map.end())
             {
                 Point point;
                 point.x = i;
                 point.y = j;
                 point.value = 0;
                 point.parent = nullptr;
                 global_map.insert(unordered_map<int, Point>::value_type(index, point));
             }
        }
    }
    startPoint.x = (start.x() - min_x) / env->gride_size;
    startPoint.y = (start.y() - min_y) / env->gride_size;
    destinationPoint.x = (destination.x() - min_x) / env->gride_size;
    destinationPoint.y = (destination.y() - min_y) / env->gride_size;
}
//路径规划
void Dijkstra::planning()
{
    //把起点放入open并另父节点为Null
    int currentKey;
    int nextKey;
    int num = 0;
    currentKey = calcKeyFromPoint(startPoint);
    global_map[currentKey].parent = nullptr;
    open.push_back(currentKey);
    while(1)
    {
         //判断open是否为空  搜索失败放回
        if(open.empty())
        {
            std::cout << "search Falied!!!" << std::endl;
            break;
        }
        //取open中value最小的节点放入closed
        currentKey = getMinfromOpen();
        closed.push_back(currentKey);
        visitedPoint.push_back(currentKey);
        //cout << num++ << "current x:" << global_map[currentKey].x << " y:" << global_map[currentKey].y  << "value:" << global_map[currentKey].value << endl;
        //如果节点为终点    结束搜索
        if(global_map[currentKey].x == destinationPoint.x && global_map[currentKey].y == destinationPoint.y)
        {
            cout << "search successful!!!" << endl;
            while(global_map[currentKey].parent != nullptr)
            {
                path.push_back(currentKey);
                currentKey = calcKeyFromPoint(*(global_map[currentKey].parent));
            }
            path.push_back(currentKey);

            calcPlanningPath();
            calcVisitedPath();
            cout << "path length:" << path.size() << endl;
            return;
        }
        //遍历当前节点的未在closed中的节点
        for (uint i = 0; i < 8; i ++) {
            nextKey = calcNextKey(currentKey, i);
            //障碍
            if(global_map[nextKey].value >= 10000) continue;
            if(inClosed(nextKey)) continue;
            //如果节点在open中    更新节点value
            if(inOpen(nextKey))
            {
                if(global_map[nextKey].value > global_map[currentKey].value + Move[i].value)
                {
                    global_map[nextKey].value = global_map[currentKey].value + Move[i].value;
                    global_map[nextKey].parent = &(global_map[currentKey]);
                }
            }
            else//否则计算节点的value, 放入open
            {
                global_map[nextKey].value = global_map[currentKey].value + Move[i].value;
                global_map[nextKey].parent = &(global_map[currentKey]);
                open.push_back(nextKey);
            }
        }
    }
}

int Dijkstra::calcKeyFromPoint(const Point &point)
{
    return point.y * 100 + point.x;
}
//获取open中最小值并删除
int Dijkstra::getMinfromOpen()
{
    int minKey;
    list<int>::iterator minIt = open.begin();
    for (list<int>::iterator it = open.begin();it != open.end(); it++) {
        if(global_map[(*it)].value < global_map[(*minIt)].value)
            minIt = it;
    }
    minKey = *minIt;
    open.erase(minIt);
    return minKey;
}
//计算下一步的KEY
int Dijkstra::calcNextKey(const int &currentKey, int index)
{
    return  100 * (global_map[currentKey].y + Move[index].y) + global_map[currentKey].x + Move[index].x;
}

bool Dijkstra::inClosed(int key)
{
    for(auto i : closed)
        if(key == i) return true;
    return false;
}

bool Dijkstra::inOpen(int key)
{
    for(auto i : open)
        if(key == i) return true;
    return false;
}

void Dijkstra::calcPlanningPath()
{
    for (int i = 0; i < path.size(); i ++) {
        planningPath.append(QPointF(global_map[path[i]].x * grideSize + min_x,
                            global_map[path[i]].y * grideSize + min_y));
    }
}
void Dijkstra::calcVisitedPath()
{
    for (int i = 0; i < visitedPoint.size(); i ++)
    {
        visitedPath.append(QPointF(global_map[visitedPoint[i]].x * grideSize + min_x,
                            global_map[visitedPoint[i]].y * grideSize + min_y));
    }
}
