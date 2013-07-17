#ifndef TRAJECTORYFOLLOWINGFORM_H
#define TRAJECTORYFOLLOWINGFORM_H

#include <QWidget>
#include <QGraphicsScene>
#include <QGraphicsLineItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsItemGroup>
#include "mapviewmetainfo.h"

namespace Ui {
class TrajectoryFollowingForm;
}

enum LocalFrameMode {
    LocalFrameAckermann,
    LocalFramePointTurn
};

class TrajectoryFollowingForm : public QWidget
{
    Q_OBJECT
signals:
    void closing();
    void sendPID();

public slots:
    void updateLocalFrame(QPointF feedbackPoint, QPointF feedbackPathPoint, QVector<QPointF> feedforwardPoints, QPointF feedforwardCenter);
    void updateLocalFrame(QPointF deviationPathPoint);
    void clearLocalFrame();

public:
    explicit TrajectoryFollowingForm(QWidget *parent = 0);
    ~TrajectoryFollowingForm();
    void closeEvent(QCloseEvent *event);
    
private slots:
    void on_resetButton_clicked();

    void on_sendToRobotButton_clicked();

private:
    Ui::TrajectoryFollowingForm *ui;
    QGraphicsLineItem *feedbackLookAheadLineItem;
    QGraphicsLineItem *feedbackErrorLineItem;
    QGraphicsEllipseItem *feedbackPathPointEllipseItem;
    QGraphicsEllipseItem *feedbackPointEllipseItem;
    QGraphicsLineItem *deviationLineItem;
    QGraphicsEllipseItem *deviationPathPointEllipseItem;
    QGraphicsItemGroup *feedforwardItems;
    QGraphicsEllipseItem *feedforwardCenterItem;


    LocalFrameMode mode;

    QGraphicsScene *localFrameScene;
    MapViewMetaInfo *localFrameInfo;

    void saveSettings();
};

#endif // TRAJECTORYFOLLOWINGFORM_H
