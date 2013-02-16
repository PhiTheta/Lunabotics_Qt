#ifndef OCCUPANCYGRAPHICSITEM_H
#define OCCUPANCYGRAPHICSITEM_H

#include <QGraphicsRectItem>
#include <QBrush>

class OccupancyGraphicsItem : public QObject, public QGraphicsRectItem
{
    Q_OBJECT
public:
    OccupancyGraphicsItem(QPoint coordinate, QRect rect, QGraphicsItem* parent);
    QPoint coordinate;

protected:
    virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);
    virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    virtual void hoverEnterEvent(QGraphicsSceneHoverEvent *event);

private:
    QBrush preservedBrush;

signals:
    void clicked(QPoint coordinate);
    void hovered(QPoint coordinate);
    
public slots:
    
};

#endif // OCCUPANCYGRAPHICSITEM_H
