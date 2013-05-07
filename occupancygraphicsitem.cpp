#include "occupancygraphicsitem.h"
#include <QDebug>
#include <QGraphicsSceneHoverEvent>

OccupancyGraphicsItem::OccupancyGraphicsItem(QPoint coordinate, QRect rect, QGraphicsItem* parent) :
    QGraphicsRectItem(rect.x(), rect.y(), rect.width(), rect.height(), parent)
{
    this->setAcceptedMouseButtons(Qt::LeftButton);
    this->setAcceptHoverEvents(true);
    this->coordinate = coordinate;
}

void OccupancyGraphicsItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsItem::mousePressEvent(event);
    emit clicked(this->coordinate);
}

void OccupancyGraphicsItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsItem::mouseReleaseEvent(event);
}

void OccupancyGraphicsItem::hoverEnterEvent(QGraphicsSceneHoverEvent *event)
{
    QGraphicsItem::hoverEnterEvent(event);
    emit hovered(this->coordinate);
}
