#include "occupancygraphicsitem.h"
#include <QDebug>
#include <QGraphicsSceneHoverEvent>
#include "Graphics.h"
#include "constants.h"

OccupancyGraphicsItem::OccupancyGraphicsItem(QPoint coordinate, QRect rect, qint32 occupancy, QGraphicsItem* parent) :
    QGraphicsRectItem(rect.x(), rect.y(), rect.width(), rect.height(), parent)
{
    this->setAcceptedMouseButtons(Qt::LeftButton);
    this->setAcceptHoverEvents(true);
    this->coordinate = coordinate;
    this->occupancy = occupancy;
    int val = 255-occupancy*2.55;
    QBrush varBrush(QColor(val, val, val));
    this->setBrush(varBrush);
    this->setPen(PEN_CLEAR);
}

void OccupancyGraphicsItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsItem::mousePressEvent(event);
    if (this->occupancy <= OCCUPANCY_THRESHOLD) {
        emit clicked(this->coordinate);
    }
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
