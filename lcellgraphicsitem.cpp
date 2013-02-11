#include "lcellgraphicsitem.h"
#include <QtGui>
#include <QtCore>

void LCellGraphicsItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    this->setBrush(QBrush(Qt::green));
}
