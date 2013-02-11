#include "occupancygraphicsitem.h"
#include <QDebug>

OccupancyGraphicsItem::OccupancyGraphicsItem(QPoint coordinate, QRect rect, QGraphicsItem* parent) :
    QGraphicsRectItem(rect.x(), rect.y(), rect.width(), rect.height(), parent)
{
    this->setAcceptedMouseButtons(Qt::LeftButton);
    this->preservedBrush.setColor(Qt::white);
    this->coordinate = coordinate;
}

void OccupancyGraphicsItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    this->preservedBrush = this->brush();
    QBrush greenBrush(Qt::green);
    this->setBrush(greenBrush);
    QGraphicsItem::mousePressEvent(event);
    emit clicked(this->coordinate);
}

void OccupancyGraphicsItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    qDebug() << "Mouse released";
    this->setBrush(this->preservedBrush);
    QGraphicsItem::mouseReleaseEvent(event);
}
