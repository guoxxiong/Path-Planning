#include "widget.h"
#include "ui_widget.h"

#include <iostream>
#include <time.h>


Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
    //初始化图
    chart = new QChart();
    barrierScatter = new QScatterSeries(chart);
    barrierScatter->setColor(QColor("black"));
    barrierScatter->setMarkerSize(12);
    barrierScatter->append(env.barrier);
    chart->addSeries(barrierScatter);

    processScatter = new QScatterSeries(chart);
    processScatter->setColor(QColor("blue"));
    processScatter->setMarkerSize(12);
    chart->addSeries(processScatter);

    pathScatter = new QScatterSeries(chart);
    pathScatter->setColor(QColor("red"));
    pathScatter->setMarkerSize(3);
    chart->addSeries(pathScatter);

    startAndDestination = new QScatterSeries(chart);
    startAndDestination->setColor(QColor("yellow"));
    startAndDestination->setMarkerSize(12);
    chart->addSeries(startAndDestination);

    chart->createDefaultAxes();

    QChartView *chartView = new QChartView(chart);
    chartView->setFixedSize(500, 500);
    chartView->setParent(ui->map);
    //初始化开始和结束点
    start = QPointF(20, 20);
    destination = QPointF(60,60);
    radius = 1.0;

    startAndDestination->append(start);
    startAndDestination->append(destination);

    mytimer = new QTimer(this);
    connect(mytimer, &QTimer::timeout, this, &Widget::dealTimer);


}

Widget::~Widget()
{
    delete ui;
}

void Widget::on_startButton_clicked()
{
    if(ui->algorithmList->currentIndex() == 0)
    {
        std::cout << "start Dijkstra" << std::endl;
        dijkstra = new Dijkstra(&env, start, destination, radius);
        clock_t nowTime = clock();
        dijkstra->planning();
        QString str = to_string((clock() - nowTime) * 1.0 / CLOCKS_PER_SEC * 1000).c_str();
        str += "ms";
        ui->castTime->setText(str);
        mytimer->start(3);
    }
    else if(ui->algorithmList->currentIndex() == 1)
    {
        std::cout << "start Astar" << std::endl;
        aStar = new Astar(&env, start, destination, radius);
        clock_t nowTime = clock();
        aStar->planning();
        QString str = to_string((clock() - nowTime) * 1.0 / CLOCKS_PER_SEC * 1000).c_str();
        str += "ms";
        ui->castTime->setText(str);
        mytimer->start(3);
    }
    else if(ui->algorithmList->currentIndex() == 2)   //DWA
    {
        std::cout << "start Dynamic Window Approach" << std::endl;
        dwa = new DWA(&env, start, destination);
        clock_t nowTime = clock();
        dwa->planning();
        QString str = to_string((clock() - nowTime) * 1.0 / CLOCKS_PER_SEC * 1000).c_str();
        str += "ms";
        ui->castTime->setText(str);
        mytimer->start(30);
    }
}
void Widget::dealTimer()
{
    static int num = 0;
    if(ui->algorithmList->currentIndex() == 0)
    {
        if(num < dijkstra->visitedPath.size())
            processScatter->append(dijkstra->visitedPath[num]);
        else {
            pathScatter->append(dijkstra->planningPath);
            mytimer->stop();
            num = 0;
        }
    }
    else if (ui->algorithmList->currentIndex() == 1) {
        if(num < aStar->visitedPath.size())
            processScatter->append(aStar->visitedPath[num]);
        else {
            pathScatter->append(aStar->planningPath);
            mytimer->stop();
            num = 0;
        }
    }
    else if(ui->algorithmList->currentIndex() == 2)
    {
        if(num < dwa->trajectory.size())
        {
            QPointF a(dwa->trajectory[num].back().x, dwa->trajectory[num].back().y);
            pathScatter->append(a);
        }
        else {
            mytimer->stop();
            num = 0;
        }

    }
    num ++;

}

void Widget::on_resetButton_clicked()
{
    processScatter->clear();
    pathScatter->clear();
    ui->castTime->setText("");

}
