#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <QPointF>
#include <unordered_map>
#include "environment.h"
#include <vector>
#include <list>
#include <QList>

using namespace std;
struct Point{
    int x;
    int y;
    float value;
    Point* parent;
};
struct Step{
    int x;
    int y;
    float value;
    Step(int x_, int y_, float value_):x(x_), y(y_), value(value_){};
};
static vector<Step> Move = {Step(-1, 1, 1.4142),
                            Step(0, 1, 1.0),
                            Step(1, 1, 1.4142),
                            Step(1, 0, 1.0),
                            Step(1, -1, 1.4142),
                            Step(0, -1, 1.0),
                            Step(-1, -1, 1.4142),
                            Step(-1, 0, 1.0)};
class Dijkstra
{
public:
    Dijkstra();
    Dijkstra(Environment* env, QPointF start, QPointF destination, float radius);
    void planning();
    int calcKeyFromPoint(const Point &point);
    int calcNextKey(const int &currentKey, int index);
    int getMinfromOpen();
    bool inClosed(int key);
    bool inOpen(int key);
    void calcPlanningPath();
    void calcVisitedPath();
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
private:


};

#endif // DIJKSTRA_H
