#include "robotgeometry.h"

RobotGeometry::RobotGeometry(QObject *parent) :
    QObject(parent)
{
    this->jointPositionsAcquired = false;
}


void RobotGeometry::setGeometry(const RobotGeometry *geometry)
{
    this->jointPositionsAcquired = geometry->jointPositionsAcquired;
    this->leftFrontJoint = geometry->leftFrontJoint;
    this->rightFrontJoint = geometry->rightFrontJoint;
    this->leftRearJoint = geometry->leftRearJoint;
    this->rightRearJoint = geometry->rightRearJoint;
}
