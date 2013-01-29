#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QKeyEvent>
#include <QTcpSocket>
#include <QTcpServer>

#define BUFFER_SIZE 256

namespace Ui {
class MainWindow;
}


enum DrivingCommandsMask
{
    DrivingForward   = 1 << 0,
    DrivingBackgward = 1 << 1,
    DrivingRight     = 1 << 2,
    DrivingLeft      = 1 << 3
};

Q_DECLARE_FLAGS(DrivingCommands, DrivingCommandsMask);
Q_DECLARE_OPERATORS_FOR_FLAGS(DrivingCommands);

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    void keyPressEvent(QKeyEvent* event);
    void keyReleaseEvent(QKeyEvent *event);

private slots:
    void on_autonomyCheckbox_clicked(bool checked);

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

private:
    Ui::MainWindow *ui;
    QTcpSocket *tcpSocket;
    QTcpServer *tcpServer;
    QTcpSocket *tcpClient;
    void leftAction();
    void rightAction();
    void forwardAction();
    void backAction();
    void postData();
    bool autonomyEnabled;
    uint8_t controlType;
    DrivingCommands drivingMask;
};

#endif // MAINWINDOW_H
