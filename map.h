#ifndef MAP_H
#define MAP_H

#include <QObject>
#include <QPoint>
#include <QPointF>
#include <QVector>

class Map : public QObject
{
    Q_OBJECT
public:
    explicit Map(QObject *parent = 0);
    explicit Map(qint32 width, qint32 height, qreal resolution, QVector<quint8> *cells, QObject *parent = 0);
    ~Map();

    qint32 width;
    qint32 height;
    qreal resolution;

    bool isValid();
    quint8 at(qint32 x, qint32 y);
    QPoint coordinateOf(QPointF point);
    QPointF positionOf(QPoint coordinate);
    QVector<quint8> *cells();
    void setCells(QVector<quint8> *cells);
    void appendCells(QVector<quint8> *cells);
private:
    QVector<quint8> *_cells;
};

#endif // MAP_H
