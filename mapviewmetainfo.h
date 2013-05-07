#ifndef MAPVIEWMETAINFO_H
#define MAPVIEWMETAINFO_H

#include <QObject>
#include <QRect>
#include <QPointF>
#include <QPoint>

class MapViewMetaInfo : public QObject
{
    Q_OBJECT
public:
    explicit MapViewMetaInfo(QObject *parent = 0);
    explicit MapViewMetaInfo(qint32 w, qint32 h, QObject *parent = 0);

    qint32 viewportWidth;
    qint32 viewportHeight;
    qint32 cellWidth;
    qint32 cellHeight;


    QRect cellRectAt(qint32 x, qint32 y);
    QRect cellRectAt(QPoint coordinate);
    QPointF cellCenterAt(qint32 x, qint32 y);
    QPointF cellCenterAt(QPoint coordinate);
    QPointF pointFromWorld(QPointF worldPoint, qreal mapResolution);
};

#endif // MAPVIEWMETAINFO_H
