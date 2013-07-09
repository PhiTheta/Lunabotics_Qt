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
#include "analysisform.h"

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
    void updateLocalFrame(QPointF feedbackPoint, QPointF feedbackPathPoint, QVector<QPointF> feedforwardPoints, QPointF feedforwardCenter);
    void clearLocalFrame();
    void updateCurves(QVector<lunabotics::proto::Telemetry::Path::Curve> curves);
    void updateRadius(float minRadius);


protected:
    void keyPressEvent(QKeyEvent* event);
    void keyReleaseEvent(QKeyEvent *event);

private slots:

    void on_useSpotButton_clicked();

    void on_useAckermannButton_clicked();
    void acceptConnection();
    void receiveTelemetry();

    void on_actionPreferences_triggered();

    void predefinedControlSelected(lunabotics::proto::AllWheelControl::PredefinedControlType controlType);
    void explicitControlSelected(AllWheelState *steering, AllWheelState *driving);
    void ICRControlSelected(QPointF ICR, float velocity);
    void crabControlSelected(qreal head, qreal vel);
    void nullifyAllWheelPanel();

    void nullifyFollowingPanel();
    void nullifyAnalysisPanel();
    void sendPID();

    void mapCell_clicked(QPoint coordinate);
    void mapCell_hovered(QPoint coordinate);

    void on_autonomyButton_clicked();

    void on_actionExit_triggered();

    void on_refreshMapButton_clicked();

    void on_actionAll_wheel_control_triggered();

    void on_actionTrajectory_following_triggered();

    void on_actionTrajectory_analysis_triggered();

    void on_multiWaypointsButton_clicked();

    void on_waypointsResetButton_clicked();

    void on_waypointsSendButton_clicked();

    void on_useAutoButton_clicked();

private:

    //UI
    Ui::MainWindow *ui;
    QGraphicsScene *mapScene;
    QStandardItemModel *pathTableModel;
    QStandardItemModel *telemetryTableModel;
    AllWheelForm *allWheelPanel;
    AnalysisForm *analysisPanel;
    TrajectoryFollowingForm *followingPanel;
    QGraphicsItemGroup *pathGraphicsItem;
    QGraphicsItemGroup *robotPointerItem;
    QGraphicsItemGroup *multiWaypointsItem;
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

    //Map chunks data
    int chunksTotal;
    int currentChunk;


    //Trajectory following data
    bool hasAckermannData;
    QPointF feedbackPathPoint;
    QPointF feedbackPoint;
    QPointF feedbackPointLocal;
    QPointF feedbackPathPointLocal;
    int nextWaypointIdx;
    int segmentIdx;

    //Trajectory analysis data
    QVector<lunabotics::proto::Telemetry::Path::Curve> trajectoryCurves;
    float minICRRadius;

    void leftAction();
    void rightAction();
    void forwardAction();
    void backAction();
    void sendTelecommand(lunabotics::proto::Telecommand::Type contentType);
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

    void resetTelemetryModel();
    void resetPathModel();

    bool isMapValid();

    qreal crabVelocity;
    qreal crabHeading;


    //Multiple waypoints selection
    bool multiWaypoints;
    QVector<QPoint> *waypoints;
    void removeMultiWaypointsPrint();

};

#endif // MAINWINDOW_H
