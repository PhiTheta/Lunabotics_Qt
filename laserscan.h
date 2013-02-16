#ifndef LASERSCAN_H
#define LASERSCAN_H

#include <QObject>
#include <QVector>

class LaserScan : public QObject
{
    Q_OBJECT

private:
    float angleMin;
    float angleMax;
    float angleIncrement;
    QVector<float> ranges;
public:
    explicit LaserScan(QObject *parent = 0);
    void setAngleMin(float newAngleMin);
    void setAngleMax(float newAngleMax);
    void setAngleIncrement(float newAngleIncrement);
    void setRanges(QVector<float> newRanges);
    float getAngleMin();
    float getAngleMax();
    float getAngleIncrement();
    QVector<float> getRanges();


signals:
    
public slots:
    
};

#endif // LASERSCAN_H
