#ifndef TRAJECTORYFOLLOWINGFORM_H
#define TRAJECTORYFOLLOWINGFORM_H

#include <QWidget>
#include <QGraphicsScene>
#include <QGraphicsLineItem>
#include <QGraphicsEllipseItem>
#include "mapviewmetainfo.h"

namespace Ui {
class TrajectoryFollowingForm;
}

class TrajectoryFollowingForm : public QWidget
{
    Q_OBJECT
signals:
    void closing();
    void sendPID();

public slots:
    void updateLocalFrame(QPointF velocityPoint, QPointF trajectoryPoint);
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
    QGraphicsLineItem *localVectorLineItem;
    QGraphicsLineItem *localDistanceLineItem;
    QGraphicsEllipseItem *localTrajectoryEllipseItem;
    QGraphicsEllipseItem *localVectorEllipseItem;


    QGraphicsScene *localFrameScene;
    MapViewMetaInfo *localFrameInfo;

    void saveSettings();
};

#endif // TRAJECTORYFOLLOWINGFORM_H
