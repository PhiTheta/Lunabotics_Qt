#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QKeyEvent>
#include <QTcpSocket>
#include <QTcpServer>
#include <QtGui>
#include <QtCore>

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

enum TX_CONTENT_TYPE
{
    STEERING         = 0,
    AUTONOMY         = 1,
    CTRL_MODE        = 2,
    ROUTE            = 3
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

    void on_ackermannLinearSpeedCheckBox_clicked(bool checked);

    void on_ackermannDependentValueCheckBox_clicked(bool checked);

    void on_spoeLinearSpeedCheckBox_clicked(bool checked);

    void on_spotDependentValueCheckBox_clicked(bool checked);

    void on_lateralLinearSpeedCheckBox_clicked(bool checked);

    void on_lateralDependentValueCheckBox_clicked(bool checked);

    void mapCell_clicked(QPoint coordinate);
    void outSocketConnected();
    void outSocketDisconnected();

    void on_autonomyButton_clicked();

    void on_actionExit_triggered();

private:
    Ui::MainWindow *ui;
    QTcpSocket *outgoingSocket;
    QTcpSocket *incomingSocket;
    QTcpServer *incomingServer;
    QGraphicsScene *mapScene;
    QVector<uint8_t> *occupancyGrid;
    QVector<QPointF> *path;
    QPointF robotPosition;
    double robotAngle;
    QPoint goal;
    uint8_t mapWidth;
    uint8_t mapHeight;
    double mapResolution;

    void leftAction();
    void rightAction();
    void forwardAction();
    void backAction();
    void postData(TX_CONTENT_TYPE contentType);
    void connectRobot();
    void disconnectRobot();
    void redrawMap();
    void toggleAutonomy();

    double decodeDouble(char buffer[], int &pointer);
    uint8_t decodeByte(char buffer[], int &pointer);
    int decodeInt(char buffer[], int &pointer);

    bool autonomyEnabled;
    uint8_t controlType;
    STEERING_CMDS drivingMask;
};

#endif // MAINWINDOW_H
