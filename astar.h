#ifndef ASTAR_H
#define ASTAR_H

#include <QPointF>
#include <unordered_map>
#include "environment.h"
#include <vector>
#include <list>
#include <QList>
#include "dijkstra.h"

class Astar
{
public:
    Astar();
    Astar(Environment* env, QPointF start, QPointF destination, float radius);
    void planning();
    int calcKeyFromPoint(const Point &point);
    int calcNextKey(const int &currentKey, int index);
    int getMinfromOpen();
    bool inClosed(int key);
    bool inOpen(int key);
    void calcPlanningPath();
    void calcVisitedPath();
    float calcH(const Point &point);
    Point startPoint;
    Point destinationPoint;
    unordered_map<int, Point> global_map;    //全局网格地图
    //算法的辅助队列
    list<int> open;
    list<int> closed;
    //存储访问过的节点
    vector<int> visitedPoint;
    vector<int> path;
    QList<QPointF> planningPath;
    QList<QPointF> visitedPath;

    int width;
    int high;
    int min_x;
    int min_y;
    int grideSize;

};

#endif // ASTAR_H
