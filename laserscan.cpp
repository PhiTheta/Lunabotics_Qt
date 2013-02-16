#include "laserscan.h"

LaserScan::LaserScan(QObject *parent) :
    QObject(parent)
{
}

void LaserScan::setAngleMin(float newAngleMin)
{
    this->angleMin = newAngleMin;
}

void LaserScan::setAngleMax(float newAngleMax)
{
    this->angleMax = newAngleMax;
}

void LaserScan::setAngleIncrement(float newAngleIncrement)
{
    this->angleIncrement = newAngleIncrement;
}

void LaserScan::setRanges(QVector<float> newRanges)
{
    this->ranges = newRanges;
}

float LaserScan::getAngleMin()
{
    return this->angleMin;
}

float LaserScan::getAngleMax()
{
    return this->angleMax;
}

float LaserScan::getAngleIncrement()
{
    return this->angleIncrement;
}

QVector<float> LaserScan::getRanges()
{
    return this->ranges;
}
