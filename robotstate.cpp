#include "robotstate.h"

RobotState::RobotState(QObject *parent) :
    QObject(parent)
{
    this->autonomous = false;
    this->pose = new Pose();
    this->drivingMotors = new AllWheelState();
    this->steeringMotors = new AllWheelState();
    this->geometry = new RobotGeometry();
    this->ICRVelocity = 0.0;
    this->drivingMask = 0;
    this->steeringMode = lunabotics::proto::ACKERMANN;
}

RobotState::~RobotState()
{
    delete this->pose;
    delete this->drivingMotors;
    delete this->steeringMotors;
    delete this->geometry;
}
