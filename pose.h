#ifndef POSE_H
#define POSE_H

#include <QObject>
#include <QPointF>

class Pose : public QObject
{
    Q_OBJECT
public:
    explicit Pose(QObject *parent = 0);
    explicit Pose(QPointF position, qreal heading, QObject *parent = 0);

    QPointF position;
    qreal heading;
    
};

#endif // POSE_H
