#include "pose.h"

Pose::Pose(QObject *parent) :
    QObject(parent)
{
    this->heading = 0.0;
}

Pose::Pose(QPointF position, qreal heading, QObject *parent) : QObject(parent)
{
    this->position = position;
    this->heading = heading;
}
