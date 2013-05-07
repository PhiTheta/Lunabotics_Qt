#include "mapviewmetainfo.h"

MapViewMetaInfo::MapViewMetaInfo(QObject *parent) :
    QObject(parent)
{
    this->viewportHeight = this->viewportWidth = this->cellHeight = this->cellWidth = 0;
}

MapViewMetaInfo::MapViewMetaInfo(qint32 w, qint32 h, QObject *parent) :
    QObject(parent)
{
    this->viewportWidth = w;
    this->viewportHeight = h;
    this->cellHeight = this->cellWidth = 0;
}

QRect MapViewMetaInfo::cellRectAt(qint32 x, qint32 y)
{
    return QRect(this->viewportWidth-(x+1)*this->cellWidth, y*this->cellHeight-this->viewportHeight/2, this->cellWidth, this->cellHeight);
}

QRect MapViewMetaInfo::cellRectAt(QPoint coordinate)
{
    return this->cellRectAt(coordinate.x(), coordinate.y());
}

QPointF MapViewMetaInfo::cellCenterAt(qint32 x, qint32 y)
{
    QRect rect = this->cellRectAt(x, y);
    return QPointF(rect.x()+this->cellWidth/2, rect.y()+this->cellHeight/2);
}

QPointF MapViewMetaInfo::cellCenterAt(QPoint coordinate)
{

    return this->cellCenterAt(coordinate.x(), coordinate.y());
}

QPointF MapViewMetaInfo::pointFromWorld(QPointF worldPoint, qreal mapResolution)
{
    qreal x = this->viewportWidth-(worldPoint.x()/mapResolution+0.5)*this->cellWidth;
    qreal y = (worldPoint.y()/mapResolution+0.5)*this->cellHeight-this->viewportHeight/2;
    return QPointF(x, y);
}
