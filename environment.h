#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <QList>
#include <QPointF>
class Environment
{
public:
    Environment();
    QList<QPointF> barrier;
    float gride_size = 2.0;

};

#endif // ENVIRONMENT_H
