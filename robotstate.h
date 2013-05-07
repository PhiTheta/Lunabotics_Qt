#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include <QObject>
#include "pose.h"
#include "allwheelstate.h"
#include "robotgeometry.h"
#include "Telecommand.pb.h"

enum DIRECTION_MASK
{
    FORWARD   = 1 << 0,
    BACKWARD  = 1 << 1,
    RIGHT     = 1 << 2,
    LEFT      = 1 << 3
};

Q_DECLARE_FLAGS(BASIC_TELEOP_CMDS, DIRECTION_MASK);
Q_DECLARE_OPERATORS_FOR_FLAGS(BASIC_TELEOP_CMDS);

class RobotState : public QObject
{
    Q_OBJECT
public:
    explicit RobotState(QObject *parent = 0);
    ~RobotState();

    Pose *pose;
    bool autonomous;
    lunabotics::SteeringModeType controlType;
    AllWheelState *drivingMotors;
    AllWheelState *steeringMotors;
    RobotGeometry *geometry;
    lunabotics::SteeringModeType steeringMode;
    lunabotics::AllWheelControl::AllWheelControlType allWheelControlType;
    lunabotics::AllWheelControl::PredefinedControlType predefinedControlType;

    QPointF ICR;
    qreal ICRVelocity;

    BASIC_TELEOP_CMDS drivingMask;
};

#endif // ROBOTSTATE_H
