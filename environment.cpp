#include "environment.h"
#include <iostream>

Environment::Environment()
{
    //barrier初始化
    for (float i = 10.0; i < 70; i += gride_size) {
        barrier.append(QPointF(10.0, i));
        barrier.append(QPointF(70.0, i + gride_size));
        barrier.append(QPointF(i + gride_size, 10.0));
        barrier.append(QPointF(i, 70.0));
    }

    for (int i = 10.0 + gride_size; i < 50; i += gride_size) {
        barrier.append(QPointF(30, i));
    }
    for (int i = 70 - gride_size; i > 30; i -= gride_size) {
        barrier.append(QPointF(50, i));
    }
    barrier.append(QPointF(20, 30));
    barrier.append(QPointF(20, 35));
    barrier.append(QPointF(25, 35));
    barrier.append(QPointF(25, 30));
    barrier.append(QPointF(27, 30));
    barrier.append(QPointF(25, 25));
    barrier.append(QPointF(22, 23));
    barrier.append(QPointF(21, 23));
    barrier.append(QPointF(23, 23));


}
