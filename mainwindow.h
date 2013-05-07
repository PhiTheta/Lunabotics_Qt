#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QKeyEvent>
#include <QTcpSocket>
#include <QTcpServer>
#include <QtGui>
#include <QtCore>
#include <QMutex>
#include <QGraphicsItemGroup>
#include <QGraphicsRectItem>
#include <QGraphicsLineItem>
#include "laserscan.h"
#include "Telecommand.pb.h"
#include "allwheelform.h"
#include "trajectoryfollowingform.h"
#include "map.h"
#include "robotstate.h"
#include "mapviewmetainfo.h"

namespace Ui {
class MainWindow;
}




class MainWindow : public QMainWindow
{
    Q_OBJECT
    Q_ENUMS(QAbstractSocket::SocketState);
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

signals:
    void allWheelStateUpdated(AllWheelState *steering, AllWheelState *driving);
    void ICRUpdated(QPointF ICR);
    void jointPositionsUpdated(RobotGeometry *geometry);
    void updateLocalFrame(QPointF velocityPoint, QPointF trajectoryPoint);
    void clearLocalFrame();

protected:
    void keyPressEvent(QKeyEvent* event);
    void keyReleaseEvent(QKeyEvent *event);

private slots:

    void on_useLateralButton_clicked();

    void on_useSpotButton_clicked();

    void on_useAckermannButton_clicked();
    void acceptConnection();
    void receiveTelemetry();

    void on_actionPreferences_triggered();

    void predefinedControlSelected(lunabotics::AllWheelControl::PredefinedControlType controlType);
    void explicitControlSelected(AllWheelState *steering, AllWheelState *driving);
    void ICRControlSelected(QPointF ICR, float velocity);
    void nullifyAllWheelPanel();

    void nullifyFollowingPanel();
    void sendPID();

    void mapCell_clicked(QPoint coordinate);
    void mapCell_hovered(QPoint coordinate);

    void on_autonomyButton_clicked();

    void on_actionExit_triggered();

    void on_refreshMapButton_clicked();

    void on_actionAll_wheel_control_triggered();

    void on_actionTrajectory_following_triggered();

private:

    //UI
    Ui::MainWindow *ui;
    QGraphicsScene *mapScene;
    QStandardItemModel *pathTableModel;
    QStandardItemModel *telemetryTableModel;
    AllWheelForm *allWheelPanel;
    TrajectoryFollowingForm *followingPanel;
    QGraphicsItemGroup *pathGraphicsItem;
    QGraphicsItemGroup *robotPointerItem;
    QGraphicsRectItem *robotCellItem;
    QGraphicsLineItem *velocityVectorItem;
    QGraphicsLineItem *closestDistanceItem;
    MapViewMetaInfo *mapViewInfo;

    //Network
    QMutex socketMutex;
    QTcpSocket *outgoingSocket;
    QTcpSocket *incomingSocket;
    QTcpServer *incomingServer;


    //Data objects
    LaserScan laserScan;
    Map *map;
    QVector<QPointF> *path;
    RobotState *robotState;

    //Trajectory following data
    QPointF closestTrajectoryPoint;
    QPointF velocityPoint;
    QPointF transformedVelocityPoint;
    QPointF transformedClosestTrajectoryPoint;
    QPoint goal;
    int nextWaypointIdx;

    void leftAction();
    void rightAction();
    void forwardAction();
    void backAction();
    void sendTelecommand(lunabotics::Telecommand::Type contentType);
    void connectRobot();
    void disconnectRobot();
    void removeAndDeleteAllMapItems();
    void setRow(int &rowNumber, const QString &label, const QString &value);

    //Redraw map grid
    void redrawMap();

    //Update poses of the robot
    void updateMapPoses();

    //Update path points
    void updateMapPath();

    //Autonomy managment
    void toggleAutonomy();
    void setAutonomy(bool enabled);
    void setAutonomyLabel(bool enabled);


};

#endif // MAINWINDOW_H
