#ifndef ALLWHEELSTATE_H
#define ALLWHEELSTATE_H

#include <QObject>

class AllWheelState : public QObject
{
    Q_OBJECT
public:
    explicit AllWheelState(QObject *parent = 0);
    explicit AllWheelState(qreal leftFront, qreal rightFront, qreal leftRear, qreal rightRear, QObject *parent = 0);

    //All wheel steering control cache
    qreal leftFront;
    qreal rightFront;
    qreal leftRear;
    qreal rightRear;

    void setState(const AllWheelState *state);
};

#endif // ALLWHEELSTATE_H
