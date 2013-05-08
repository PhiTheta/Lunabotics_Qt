#include "robotgeometry.h"

RobotGeometry::RobotGeometry(QObject *parent) :
    QObject(parent)
{
    this->jointPositionsAcquired = false;
    this->wheelOffset = 0;
    this->wheelRadius = 0;
}


void RobotGeometry::setGeometry(const RobotGeometry *geometry)
{
    this->jointPositionsAcquired = geometry->jointPositionsAcquired;
    this->leftFrontJoint = geometry->leftFrontJoint;
    this->rightFrontJoint = geometry->rightFrontJoint;
    this->leftRearJoint = geometry->leftRearJoint;
    this->rightRearJoint = geometry->rightRearJoint;
    this->wheelOffset = geometry->wheelOffset;
    this->wheelRadius = geometry->wheelRadius;
}
