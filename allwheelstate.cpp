#include "allwheelstate.h"

AllWheelState::AllWheelState(QObject *parent) :
    QObject(parent)
{
    this->leftFront = 0;
    this->leftRear = 0;
    this->rightFront = 0;
    this->rightRear = 0;
}

AllWheelState::AllWheelState(qreal leftFront, qreal rightFront, qreal leftRear, qreal rightRear, QObject *parent) :
    QObject(parent)
{
    this->leftFront = leftFront;
    this->leftRear = leftRear;
    this->rightFront = rightFront;
    this->rightRear = rightRear;
}

void AllWheelState::setState(const AllWheelState *state)
{
    this->leftFront = state->leftFront;
    this->leftRear = state->leftRear;
    this->rightFront = state->rightFront;
    this->rightRear = state->rightRear;
}
