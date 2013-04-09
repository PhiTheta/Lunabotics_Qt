#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QKeyEvent>
#include <QTcpSocket>
#include <QTcpServer>
#include <QtGui>
#include <QtCore>
#include <QMutex>
#include "laserscan.h"
#include "Telecommand.pb.h"

#define BUFFER_SIZE 256

namespace Ui {
class MainWindow;
}


enum STEERING_MASK
{
    FORWARD   = 1 << 0,
    BACKWARD  = 1 << 1,
    RIGHT     = 1 << 2,
    LEFT      = 1 << 3
};

Q_DECLARE_FLAGS(STEERING_CMDS, STEERING_MASK);
Q_DECLARE_OPERATORS_FOR_FLAGS(STEERING_CMDS);

class MainWindow : public QMainWindow
{
    Q_OBJECT
    Q_ENUMS(QAbstractSocket::SocketState);
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    void keyPressEvent(QKeyEvent* event);
    void keyReleaseEvent(QKeyEvent *event);

private slots:

    void on_useLateralButton_clicked();

    void on_useSpotButton_clicked();

    void on_useAckermannButton_clicked();
    void serverAcceptConnection();
    void serverStartRead();

    void on_actionPreferences_triggered();

    void mapCell_clicked(QPoint coordinate);
    void mapCell_hovered(QPoint coordinate);
    void outSocketConnected();
    void outSocketDisconnected();

    void on_autonomyButton_clicked();

    void on_actionExit_triggered();

    void on_refreshMapButton_clicked();

    void on_resendParamsButton_clicked();

    void on_removePathButton_clicked();

    void on_allWheelButton_clicked();

    void on_forwardButton_clicked();

    void on_backwardButton_clicked();

    void on_stopButton_clicked();

private:
    QMutex socketMutex;
    Ui::MainWindow *ui;
    QTcpSocket *outgoingSocket;
    QTcpSocket *incomingSocket;
    QTcpServer *incomingServer;
    QGraphicsScene *mapScene;
    QGraphicsScene *localFrameScene;
    QVector<int> *occupancyGrid;
    QVector<QPointF> *path;
    QPointF robotPosition;
    QPointF closestTrajectoryPoint;
    QPointF velocityPoint;
    QPointF transformedVelocityPoint;
    QPointF transformedClosestTrajectoryPoint;
    double robotAngle;
    QPoint goal;
    int mapWidth;
    int mapHeight;
    double mapResolution;
    int nextWaypointIdx;
    QPointF mapPoint(QPointF pointInMeters);
    lunabotics::SteeringModeType robotControlType;
    QStandardItemModel *pathTableModel;
    LaserScan laserScan;

    int mapViewportWidth;
    int mapViewportHeight;
    int mapCellWidth;
    int mapCellHeight;

    int localFrameViewportWidth;
    int localFrameViewportHeight;

    void leftAction();
    void rightAction();
    void forwardAction();
    void backAction();
    void postData(lunabotics::Telecommand::Type contentType);
    void connectRobot();
    void disconnectRobot();
    void redrawMap();
    void toggleAutonomy();
    void setAutonomy(bool enabled);
    void setAutonomyLabel(bool enabled);

    bool autonomyEnabled;
    lunabotics::SteeringModeType controlType;
    STEERING_CMDS drivingMask;
};

#endif // MAINWINDOW_H
