#include "mapviewmetainfo.h"

MapViewMetaInfo::MapViewMetaInfo(QObject *parent) :
    QObject(parent)
{
    this->viewportHeight = this->viewportWidth = this->cellEdge = 0;
}

MapViewMetaInfo::MapViewMetaInfo(qint32 w, qint32 h, QObject *parent) :
    QObject(parent)
{
    this->viewportWidth = w;
    this->viewportHeight = h;
    this->cellEdge = 0;
}

QRect MapViewMetaInfo::cellRectAt(qint32 x, qint32 y)
{
    return QRect(this->viewportWidth-(x+1)*this->cellEdge, y*this->cellEdge-this->viewportHeight/2, this->cellEdge, this->cellEdge);
}

QRect MapViewMetaInfo::cellRectAt(QPoint coordinate)
{
    return this->cellRectAt(coordinate.x(), coordinate.y());
}

QPointF MapViewMetaInfo::cellCenterAt(qint32 x, qint32 y)
{
    QRect rect = this->cellRectAt(x, y);
    return QPointF(rect.x()+this->cellEdge/2, rect.y()+this->cellEdge/2);
}

QPointF MapViewMetaInfo::cellCenterAt(QPoint coordinate)
{

    return this->cellCenterAt(coordinate.x(), coordinate.y());
}

QPointF MapViewMetaInfo::pointFromWorld(QPointF worldPoint, qreal mapResolution)
{
    qreal x = this->viewportWidth-(worldPoint.x()/mapResolution+0.5)*this->cellEdge;
    qreal y = (worldPoint.y()/mapResolution+0.5)*this->cellEdge-this->viewportHeight/2;
    return QPointF(x, y);
}
