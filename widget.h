#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QtCharts>
#include <QtCharts/QChartView>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QValueAxis>
#include "environment.h"
#include "dijkstra.h"
#include "astar.h"
#include "dwa.h"
#include <QTimer>


using namespace QtCharts;

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = nullptr);
    ~Widget();
    QChart *chart;
    Environment env;
    QScatterSeries *barrierScatter;
    QScatterSeries *pathScatter;
    QScatterSeries *processScatter;
    QScatterSeries *startAndDestination;
    QPointF start;         //起点
    QPointF destination;   // 终点
    float radius;    //机器人半径
    QTimer *mytimer;
    Dijkstra *dijkstra;
    Astar *aStar;
    DWA *dwa;

private slots:
    void on_startButton_clicked();
    void dealTimer();

    void on_resetButton_clicked();

private:
    Ui::Widget *ui;
};

#endif // WIDGET_H
