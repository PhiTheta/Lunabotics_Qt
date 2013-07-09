#include "map.h"
#include <QtCore>

Map::Map(QObject *parent) :
    QObject(parent)
{
    this->width = 0;
    this->height = 0;
    this->resolution = 0.0;
    this->_cells = NULL;
}

Map::Map(qint32 width, qint32 height, qreal resolution, QVector<quint8> *cells, QObject *parent) : QObject(parent)
{
    this->width = width;
    this->height = height;
    this->resolution = resolution;
    this->_cells = cells;
}

Map::~Map()
{
    delete this->_cells;
}

bool Map::isValid()
{
    return this->width > 0 && this->height > 0 && this->resolution > 0.0 && !this->_cells->isEmpty() && this->width*this->height == this->_cells->size();
}


quint8 Map::at(qint32 x, qint32 y)
{
    if (x >= 0 && x < this->width && y >= 0 && y < this->height) {
        qint32 idx = y*this->width+x;
        if (idx < this->_cells->size()) {
            return this->_cells->at(idx);
        }
    }
    return 0;
}

QPoint Map::coordinateOf(QPointF point)
{
    qint32 x = round(point.x()/this->resolution);
    qint32 y = round(point.y()/this->resolution);
    if (x < 0) { x = 0; }
    else if (x >= this->width) { x = this->width-1; }

    if (y < 0) { y = 0; }
    else if (y >= this->height) {y = this->height-1; }

    return QPoint(x, y);
}

QPointF Map::positionOf(QPoint coordinate)
{
    return QPointF(coordinate.x()*this->resolution, coordinate.y()*this->resolution);
}

QVector<quint8> *Map::cells()
{
    return this->_cells;
}

void Map::setCells(QVector<quint8> *cells)
{
    if (this->_cells) {
        delete this->_cells;
    }
    this->_cells = cells;
}


void Map::appendCells(QVector<quint8> *cells)
{
    if (!this->_cells) {
        this->_cells = cells;
    }
    else {
        for (int i = 0; i < cells->size(); i++) {
            this->_cells->append(cells->at(i));
        }
    }
}
