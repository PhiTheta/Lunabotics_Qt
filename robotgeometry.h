#ifndef ROBOTGEOMETRY_H
#define ROBOTGEOMETRY_H

#include <QObject>
#include <QPointF>

class RobotGeometry : public QObject
{
    Q_OBJECT
public:
    explicit RobotGeometry(QObject *parent = 0);

    QPointF leftFrontJoint;
    QPointF rightFrontJoint;
    QPointF leftRearJoint;
    QPointF rightRearJoint;
    bool jointPositionsAcquired;
    qreal wheelOffset;
    qreal wheelRadius;

    void setGeometry(const RobotGeometry *geometry);
};

#endif // ROBOTGEOMETRY_H
