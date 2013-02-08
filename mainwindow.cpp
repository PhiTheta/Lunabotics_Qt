#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "preferencedialog.h"
#include "constants.h"
#include <QDebug>
#include <QByteArray>
#include <QtNetwork>
#include <QHostAddress>
#include <QSettings>
#include <QMetaEnum>

#define DEFAULT_LINEAR_SPEED_LIMIT  5.0
#define DEFAULT_WHEEL_ROTATION_ANGLE_LIMIT  45
#define DEFAULT_SPOT_ROTATIONAL_SPEED_LIMIT 1.0
#define DEFAULT_LATERAL_SPEED_LIMIT 2.0

union BytesToFloat {
    char    bytes[4];
    float   floatValue;
};

union BytesToUint8 {
    char bytes[1];
    uint8_t uint8Value;
};

union BytesToDouble {
    char bytes[8];
    double doubleValue;
};

enum CTRL_MODE_TYPE {
    ACKERMANN 		 = 0,
    TURN_IN_SPOT     = 1,
    LATERAL   		 = 2
};

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    this->outgoingSocket = NULL;
    this->incomingServer = NULL;
    this->incomingSocket = NULL;

    QString text;
    ui->ackermannLinearSpeedEdit->setText(text.setNum(DEFAULT_LINEAR_SPEED_LIMIT));
    ui->spotLinearSpeedEdit->setText(text.setNum(DEFAULT_LINEAR_SPEED_LIMIT));
    ui->lateralLinearSpeedEdit->setText(text.setNum(DEFAULT_LINEAR_SPEED_LIMIT));
    ui->ackermanDependentValueEdit->setText(text.setNum(DEFAULT_WHEEL_ROTATION_ANGLE_LIMIT));
    ui->spotDependentValueEdit->setText(text.setNum(DEFAULT_SPOT_ROTATIONAL_SPEED_LIMIT));
    ui->lateralDependentValueEdit->setText(text.setNum(DEFAULT_LATERAL_SPEED_LIMIT));

    this->connectRobot();

    this->autonomyEnabled = false;
    this->controlType = ACKERMANN;
    ui->driveForwardLabel->setStyleSheet("QLabel { background-color : blue; color : white; }");
    ui->driveLeftLabel->setStyleSheet("QLabel { background-color : blue; color : white; }");
    ui->driveRightLabel->setStyleSheet("QLabel { background-color : blue; color : white; }");
    ui->driveBackLabel->setStyleSheet("QLabel { background-color : blue; color : white; }");
}

MainWindow::~MainWindow()
{
    delete ui;
    this->disconnectRobot();
    delete this->incomingServer;
    delete this->outgoingSocket;
}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    if (event->text().compare("w", Qt::CaseSensitive) == 0) {
        this->drivingMask |= FORWARD;
        ui->driveForwardLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
    }
    else if (event->text().compare("a", Qt::CaseSensitive) == 0) {
        this->drivingMask |= LEFT;
        ui->driveLeftLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
    }
    else if (event->text().compare("s", Qt::CaseSensitive) == 0) {
        this->drivingMask |= BACKWARD;
        ui->driveBackLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
    }
    else if (event->text().compare("d", Qt::CaseSensitive) == 0) {
        this->drivingMask |= RIGHT;
        ui->driveRightLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
    }
    else {
        return;
    }
    this->postData(STEERING);
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    if (event->text().compare("w", Qt::CaseSensitive) == 0) {
        this->drivingMask &= ~FORWARD;
        ui->driveForwardLabel->setStyleSheet("QLabel { background-color : blue; color : white; }");
    }
    else if (event->text().compare("a", Qt::CaseSensitive) == 0) {
        this->drivingMask &= ~LEFT;
        ui->driveLeftLabel->setStyleSheet("QLabel { background-color : blue; color : white; }");
    }
    else if (event->text().compare("s", Qt::CaseSensitive) == 0) {
        this->drivingMask &= ~BACKWARD;
        ui->driveBackLabel->setStyleSheet("QLabel { background-color : blue; color : white; }");
    }
    else if (event->text().compare("d", Qt::CaseSensitive) == 0) {
        this->drivingMask &= ~RIGHT;
        ui->driveRightLabel->setStyleSheet("QLabel { background-color : blue; color : white; }");
    }
    else {
        return;
    }
    this->postData(STEERING);
}

void MainWindow::postData(TX_CONTENT_TYPE contentType)
{
    this->outgoingSocket->abort();
    QSettings settings("ivany4", "lunabotics");
    settings.beginGroup("connection");
    this->outgoingSocket->connectToHost(settings.value("out_ip", CONN_OUTGOING_ADDR).toString(), settings.value("out_port", CONN_OUTGOING_PORT).toInt());
    settings.endGroup();

    union BytesToFloat floatConverter;

    QByteArray *bytes = new QByteArray();
    bytes->append(contentType);


    switch (contentType) {
    case AUTONOMY:
        bytes->append(this->autonomyEnabled);
        break;
       case CTRL_MODE:
        bytes->append(this->controlType);
        break;
    case STEERING: {
        float linearSpeedLimit = DEFAULT_LATERAL_SPEED_LIMIT;
        float controlDependentLimit = 0;
        switch (this->controlType) {
        case ACKERMANN:
            controlDependentLimit = DEFAULT_WHEEL_ROTATION_ANGLE_LIMIT;
            if (ui->ackermannLinearSpeedCheckBox->checkState() == Qt::Checked) {
                linearSpeedLimit = ui->ackermannLinearSpeedEdit->text().toFloat();
            }
            if (ui->ackermannDependentValueCheckBox->checkState() == Qt::Checked) {
                controlDependentLimit = ui->ackermanDependentValueEdit->text().toFloat();
            }
            break;
        case TURN_IN_SPOT:
            controlDependentLimit = DEFAULT_SPOT_ROTATIONAL_SPEED_LIMIT;
            if (ui->spoeLinearSpeedCheckBox->checkState() == Qt::Checked) {
                linearSpeedLimit = ui->spotLinearSpeedEdit->text().toFloat();
            }
            if (ui->spotDependentValueCheckBox->checkState() == Qt::Checked) {
                controlDependentLimit = ui->spotDependentValueEdit->text().toFloat();
            }
            break;
        case LATERAL:
            controlDependentLimit = DEFAULT_LATERAL_SPEED_LIMIT;
            if (ui->lateralLinearSpeedCheckBox->checkState() == Qt::Checked) {
                linearSpeedLimit = ui->lateralLinearSpeedEdit->text().toFloat();
            }
            if (ui->lateralDependentValueCheckBox->checkState() == Qt::Checked) {
                controlDependentLimit = ui->lateralDependentValueEdit->text().toFloat();
            }
            break;
        default:
            break;
        }
        qDebug() << "Settign linear speed limit as " << linearSpeedLimit;
        qDebug() << "Settign dependent limit as " << controlDependentLimit;
        floatConverter.floatValue = linearSpeedLimit;
        bytes->append(floatConverter.bytes, sizeof(float));
        floatConverter.floatValue = controlDependentLimit;
        bytes->append(floatConverter.bytes, sizeof(float));
        qDebug() << "Driving:";
        if (this->drivingMask & FORWARD) {
            qDebug() << "   Forward";
        }
        if (this->drivingMask & BACKWARD) {
            qDebug() << "   Backward";
        }
        if (this->drivingMask & LEFT) {
            qDebug() << "   Left";
        }
        if (this->drivingMask & RIGHT) {
            qDebug() << "   Right";
        }

        bytes->append(this->drivingMask & FORWARD);
        bytes->append(this->drivingMask & BACKWARD);
        bytes->append(this->drivingMask & LEFT);
        bytes->append(this->drivingMask & RIGHT);
    }
        break;
    case ROUTE:
        break;
    default:
        break;
    }

    if (this->outgoingSocket->state() != QTcpSocket::UnconnectedState) {
        this->outgoingSocket->write(bytes->data(), bytes->size());
    }
    else {
        qDebug() << "Socket is not connected!";
    }

    // READ THE RESPONSE

    //this->outgoingSocket->read(bytes->data(), bytes->size());

    delete bytes;
}

void MainWindow::on_autonomyCheckbox_clicked(bool checked)
{
    if (checked) {
        qDebug() << "Disabling autonomy";
    }
    else {
        qDebug() << "Enabling autonomy";
    }
    this->autonomyEnabled = !checked;
    this->postData(AUTONOMY);
}

void MainWindow::on_useLateralButton_clicked()
{
    qDebug() << "Switching to lateral driving mode";
    this->controlType = LATERAL;
    this->postData(CTRL_MODE);
}

void MainWindow::on_useSpotButton_clicked()
{
    qDebug() << "Switching to spot driving mode";
    this->controlType = TURN_IN_SPOT;
    this->postData(CTRL_MODE);
}

void MainWindow::on_useAckermannButton_clicked()
{
    qDebug() << "Switching to Ackermann driving mode";
    this->controlType = ACKERMANN;
    this->postData(CTRL_MODE);
}


//TCP Server methods


void MainWindow::serverAcceptConnection()
{
    this->incomingSocket = this->incomingServer->nextPendingConnection();

    if (this->incomingSocket) {
        qDebug() << "New connection";
        connect(this->incomingSocket, SIGNAL(readyRead()), this, SLOT(serverStartRead()));
    }
}

void MainWindow::serverStartRead()
{
    qDebug() << "Receiving telemetry";
    char buffer[sizeof(double)*6] = {0};
    this->incomingSocket->read(buffer, this->incomingSocket->bytesAvailable());

    BytesToDouble doubleConverter;
    int pointer = 0;
    doubleConverter.bytes[0] = buffer[pointer++];
    doubleConverter.bytes[1] = buffer[pointer++];
    doubleConverter.bytes[2] = buffer[pointer++];
    doubleConverter.bytes[3] = buffer[pointer++];
    doubleConverter.bytes[4] = buffer[pointer++];
    doubleConverter.bytes[5] = buffer[pointer++];
    doubleConverter.bytes[6] = buffer[pointer++];
    doubleConverter.bytes[7] = buffer[pointer++];
    double posXValue = doubleConverter.doubleValue;
    doubleConverter.bytes[0] = buffer[pointer++];
    doubleConverter.bytes[1] = buffer[pointer++];
    doubleConverter.bytes[2] = buffer[pointer++];
    doubleConverter.bytes[3] = buffer[pointer++];
    doubleConverter.bytes[4] = buffer[pointer++];
    doubleConverter.bytes[5] = buffer[pointer++];
    doubleConverter.bytes[6] = buffer[pointer++];
    doubleConverter.bytes[7] = buffer[pointer++];
    double posYValue = doubleConverter.doubleValue;
    doubleConverter.bytes[0] = buffer[pointer++];
    doubleConverter.bytes[1] = buffer[pointer++];
    doubleConverter.bytes[2] = buffer[pointer++];
    doubleConverter.bytes[3] = buffer[pointer++];
    doubleConverter.bytes[4] = buffer[pointer++];
    doubleConverter.bytes[5] = buffer[pointer++];
    doubleConverter.bytes[6] = buffer[pointer++];
    doubleConverter.bytes[7] = buffer[pointer++];
    double orValue = doubleConverter.doubleValue;
    doubleConverter.bytes[0] = buffer[pointer++];
    doubleConverter.bytes[1] = buffer[pointer++];
    doubleConverter.bytes[2] = buffer[pointer++];
    doubleConverter.bytes[3] = buffer[pointer++];
    doubleConverter.bytes[4] = buffer[pointer++];
    doubleConverter.bytes[5] = buffer[pointer++];
    doubleConverter.bytes[6] = buffer[pointer++];
    doubleConverter.bytes[7] = buffer[pointer++];
    double vXValue = doubleConverter.doubleValue;
    doubleConverter.bytes[0] = buffer[pointer++];
    doubleConverter.bytes[1] = buffer[pointer++];
    doubleConverter.bytes[2] = buffer[pointer++];
    doubleConverter.bytes[3] = buffer[pointer++];
    doubleConverter.bytes[4] = buffer[pointer++];
    doubleConverter.bytes[5] = buffer[pointer++];
    doubleConverter.bytes[6] = buffer[pointer++];
    doubleConverter.bytes[7] = buffer[pointer++];
    double vYValue = doubleConverter.doubleValue;
    doubleConverter.bytes[0] = buffer[pointer++];
    doubleConverter.bytes[1] = buffer[pointer++];
    doubleConverter.bytes[2] = buffer[pointer++];
    doubleConverter.bytes[3] = buffer[pointer++];
    doubleConverter.bytes[4] = buffer[pointer++];
    doubleConverter.bytes[5] = buffer[pointer++];
    doubleConverter.bytes[6] = buffer[pointer++];
    doubleConverter.bytes[7] = buffer[pointer++];
    double vZValue = doubleConverter.doubleValue;
//    doubleConverter.bytes[0] = buffer[pointer++];
//    doubleConverter.bytes[1] = buffer[pointer++];
//    doubleConverter.bytes[2] = buffer[pointer++];
//    doubleConverter.bytes[3] = buffer[pointer++];
//    doubleConverter.bytes[4] = buffer[pointer++];
//    doubleConverter.bytes[5] = buffer[pointer++];
//    doubleConverter.bytes[6] = buffer[pointer++];
//    doubleConverter.bytes[7] = buffer[pointer++];
//    double wXValue = doubleConverter.doubleValue;
//    doubleConverter.bytes[0] = buffer[pointer++];
//    doubleConverter.bytes[1] = buffer[pointer++];
//    doubleConverter.bytes[2] = buffer[pointer++];
//    doubleConverter.bytes[3] = buffer[pointer++];
//    doubleConverter.bytes[4] = buffer[pointer++];
//    doubleConverter.bytes[5] = buffer[pointer++];
//    doubleConverter.bytes[6] = buffer[pointer++];
//    doubleConverter.bytes[7] = buffer[pointer++];
//    double wYValue = doubleConverter.doubleValue;
//    doubleConverter.bytes[0] = buffer[pointer++];
//    doubleConverter.bytes[1] = buffer[pointer++];
//    doubleConverter.bytes[2] = buffer[pointer++];
//    doubleConverter.bytes[3] = buffer[pointer++];
//    doubleConverter.bytes[4] = buffer[pointer++];
//    doubleConverter.bytes[5] = buffer[pointer++];
//    doubleConverter.bytes[6] = buffer[pointer++];
//    doubleConverter.bytes[7] = buffer[pointer++];
//    double wZValue = doubleConverter.doubleValue;

    ui->posXLabel->setText(QString::number(posXValue));
    ui->posYLabel->setText(QString::number(posYValue));
    ui->orLabel->setText(QString::number(orValue));
    ui->vXLabel->setText(QString::number(vXValue));
    ui->vYLabel->setText(QString::number(vYValue));
    ui->vZLabel->setText(QString::number(vZValue));
//    ui->wXLabel->setText(QString::number(wXValue));
//    ui->wYLabel->setText(QString::number(wYValue));
//    ui->wZLabel->setText(QString::number(wZValue));
}

void MainWindow::on_actionPreferences_triggered()
{
    PreferenceDialog *preferenceDialog = new PreferenceDialog(this);
    preferenceDialog->setWindowModality(Qt::WindowModal);
    if (preferenceDialog->exec() == QDialog::Accepted) {
        this->connectRobot();
    }
}

void MainWindow::disconnectRobot()
{
    qDebug() << "Closing connection";
    if (this->outgoingSocket) {
        this->outgoingSocket->close();
    }
    if (this->incomingServer) {
        this->incomingServer->close();
    }
    if (this->incomingSocket) {
        this->incomingSocket->close();
    }
}

void MainWindow::connectRobot()
{
    this->disconnectRobot();

    qDebug() << "Creating new connection";


    QSettings settings("ivany4", "lunabotics");
    settings.beginGroup("connection");

    if (!this->outgoingSocket) {
        this->outgoingSocket = new QTcpSocket(this);
    }
    this->outgoingSocket->connectToHost(settings.value("out_ip", CONN_OUTGOING_ADDR).toString(), settings.value("out_port", CONN_OUTGOING_PORT).toInt());

    if (!this->incomingServer) {
        this->incomingServer = new QTcpServer(this);
        connect(this->incomingServer, SIGNAL(newConnection()), this, SLOT(serverAcceptConnection()));
    }

    if (!this->incomingServer->listen(QHostAddress(settings.value("in_ip", CONN_INCOMING_ADDR).toString()), settings.value("in_port", CONN_INCOMING_PORT).toInt())) {
        qDebug() << "Failed to start listening for " << settings.value("in_ip", CONN_INCOMING_ADDR).toString();
    }

    settings.endGroup();
}

void MainWindow::on_ackermannLinearSpeedCheckBox_clicked(bool checked)
{
    ui->ackermannLinearSpeedEdit->setEnabled(checked);
}

void MainWindow::on_ackermannDependentValueCheckBox_clicked(bool checked)
{
    ui->ackermanDependentValueEdit->setEnabled(checked);
}

void MainWindow::on_spoeLinearSpeedCheckBox_clicked(bool checked)
{
    ui->spotLinearSpeedEdit->setEnabled(checked);
}

void MainWindow::on_spotDependentValueCheckBox_clicked(bool checked)
{
    ui->spotDependentValueEdit->setEnabled(checked);
}

void MainWindow::on_lateralLinearSpeedCheckBox_clicked(bool checked)
{
    ui->lateralLinearSpeedEdit->setEnabled(checked);
}

void MainWindow::on_lateralDependentValueCheckBox_clicked(bool checked)
{
    ui->lateralDependentValueEdit->setEnabled(checked);
}
