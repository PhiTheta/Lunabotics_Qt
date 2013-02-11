#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "preferencedialog.h"
#include "constants.h"
#include "occupancygraphicsitem.h"
#include <QDebug>
#include <QByteArray>
#include <QtNetwork>
#include <QHostAddress>
#include <QSettings>
#include <QMetaEnum>
#include <QtAlgorithms>

#define DEFAULT_LINEAR_SPEED_LIMIT  5.0
#define DEFAULT_WHEEL_ROTATION_ANGLE_LIMIT  45
#define DEFAULT_SPOT_ROTATIONAL_SPEED_LIMIT 1.0
#define DEFAULT_LATERAL_SPEED_LIMIT 2.0

#define OCCUPANCY_THRESHOLD     80

union BytesToFloatInt {
    char    bytes[4];
    float   floatValue;
    float   intValue;
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

enum RX_CONTENT_TYPE {
    TELEMETRY       = 0,
    MAP             = 1,
    PATH            = 2
};

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    this->outgoingSocket = NULL;
    this->incomingServer = NULL;
    this->incomingSocket = NULL;
    this->mapWidth = this->mapHeight = 0;

    this->mapScene = new QGraphicsScene(this);
    ui->mapView->setScene(this->mapScene);
    this->occupancyGrid = new QVector<uint8_t>();
    this->path = new QVector<QPointF>();

    QString text;
    ui->ackermannLinearSpeedEdit->setText(text.setNum(DEFAULT_LINEAR_SPEED_LIMIT));
    ui->spotLinearSpeedEdit->setText(text.setNum(DEFAULT_LINEAR_SPEED_LIMIT));
    ui->lateralLinearSpeedEdit->setText(text.setNum(DEFAULT_LINEAR_SPEED_LIMIT));
    ui->ackermanDependentValueEdit->setText(text.setNum(DEFAULT_WHEEL_ROTATION_ANGLE_LIMIT));
    ui->spotDependentValueEdit->setText(text.setNum(DEFAULT_SPOT_ROTATIONAL_SPEED_LIMIT));
    ui->lateralDependentValueEdit->setText(text.setNum(DEFAULT_LATERAL_SPEED_LIMIT));

    this->redrawMap();

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
    delete this->mapScene;
    delete this->occupancyGrid;
    delete this->path;
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
    /*
    this->outgoingSocket->abort();
    QSettings settings("ivany4", "lunabotics");
    settings.beginGroup("connection");
    this->outgoingSocket->connectToHost(settings.value("out_ip", CONN_OUTGOING_ADDR).toString(), settings.value("out_port", CONN_OUTGOING_PORT).toInt());
    settings.endGroup();
*/
    union BytesToFloatInt floatConverter;

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
        floatConverter.floatValue = this->goal.x()*this->mapResolution;
        bytes->append(floatConverter.bytes, sizeof(float));
        floatConverter.floatValue = this->goal.y()*this->mapResolution;
        bytes->append(floatConverter.bytes, sizeof(float));
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

double MainWindow::decodeDouble(char buffer[], int &pointer)
{
    BytesToDouble doubleConverter;
    for (unsigned int i = 0; i < sizeof(double); i++) {
        doubleConverter.bytes[i] = buffer[pointer++];
    }
    return doubleConverter.doubleValue;
}

uint8_t MainWindow::decodeByte(char buffer[], int &pointer)
{
    BytesToUint8 uint8Converter;
    uint8Converter.bytes[0] = buffer[pointer++];
    return uint8Converter.uint8Value;
}

int MainWindow::decodeInt(char buffer[], int &pointer)
{
    BytesToFloatInt intConverter;
    for (unsigned int i = 0; i < sizeof(int); i++) {
        intConverter.bytes[i] = buffer[pointer++];
    }
    return intConverter.intValue;
}

void MainWindow::serverStartRead()
{
    int pointer = 0;
    qint64 bytesAvailable = this->incomingSocket->bytesAvailable();
    char *buffer = new char[bytesAvailable];
    this->incomingSocket->read(buffer, bytesAvailable);

    RX_CONTENT_TYPE contentType = (RX_CONTENT_TYPE)this->decodeByte(buffer, pointer);

    switch (contentType) {
    case TELEMETRY: {
     //   qDebug() << "Receiving telemetry";

        double posXValue = this->decodeDouble(buffer, pointer);
        double posYValue = this->decodeDouble(buffer, pointer);
        double orValue = this->decodeDouble(buffer, pointer);
        double vXValue = this->decodeDouble(buffer, pointer);
        double vYValue = this->decodeDouble(buffer, pointer);
        double vZValue = this->decodeDouble(buffer, pointer);

        //    double wXValue = this->decodeDouble(buffer, pointer);
        //    double wYValue = this->decodeDouble(buffer, pointer);
        //    double wZValue = this->decodeDouble(buffer, pointer);

        ui->posXLabel->setText(QString("%1 m").arg(QString::number(posXValue, 'f', 2)));
        ui->posYLabel->setText(QString("%1 m").arg(QString::number(posYValue, 'f', 2)));
        ui->orLabel->setText(QString("%1 rad").arg(QString::number(orValue, 'f', 2)));
        ui->vXLabel->setText(QString("%1 m/s").arg(QString::number(vXValue, 'f', 2)));
        ui->vYLabel->setText(QString("%1 m/s").arg(QString::number(vYValue, 'f', 2)));
        ui->vZLabel->setText(QString("%1 m/s").arg(QString::number(vZValue, 'f', 2)));


        this->robotPosition.setX(posXValue);
        this->robotPosition.setY(posYValue);
        //    ui->wXLabel->setText(QString::number(wXValue));
        //    ui->wYLabel->setText(QString::number(wYValue));
        //    ui->wZLabel->setText(QString::number(wZValue));

        this->redrawMap();
    }
        break;

    case MAP: {
        this->mapWidth = this->decodeByte(buffer, pointer);
        this->mapHeight = this->decodeByte(buffer, pointer);
        this->mapResolution = this->decodeDouble(buffer, pointer);
        qDebug() << "Receiving map (" << this->mapWidth << "x" << this->mapHeight << ") res is " << this->mapResolution;
        this->occupancyGrid->clear();

        for (int i = 0; i < this->mapWidth*this->mapHeight; i++) {
            uint8_t cell = this->decodeByte(buffer, pointer);
            this->occupancyGrid->push_back(cell);
        }

        this->redrawMap();
    }
        break;

    case PATH: {
        qDebug() << "Receiving waypoints";
        int numOfPoses = this->decodeByte(buffer, pointer);
        this->path->clear();

        for (int i = 0; i < numOfPoses; i++) {
            double x = this->decodeDouble(buffer, pointer);
            double y = this->decodeDouble(buffer, pointer);
            QPointF point;
            point.setX(x);
            point.setY(y);
            qDebug() << x << "  ,  " << y;
            this->path->push_back(point);
        }

        this->redrawMap();
    }

    default:
        break;
    }

    delete buffer;
}

void MainWindow::on_actionPreferences_triggered()
{
    PreferenceDialog *preferenceDialog = new PreferenceDialog(this);
    preferenceDialog->setWindowModality(Qt::WindowModal);
    if (preferenceDialog->exec() == QDialog::Accepted) {
        this->connectRobot();
    }
}

void MainWindow::redrawMap()
{
    qDeleteAll(this->mapScene->items());


    //////////////TEST///////////////
/*
    this->mapWidth = 10;
    this->mapHeight = 10;
    this->mapResolution = 1;
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            uint8_t occ = 0;
            if (i > 6 && i < 8 && j > 4 && j < 7) occ = 100;
            this->occupancyGrid->push_back(occ);
        }
    }

    qDebug() << "TEsting Map";


*/



    //////////////////////////////////

    if (this->mapWidth > 0 && this->mapHeight > 0) {
        QBrush redBrush(Qt::red);
        QBrush whiteBrush(Qt::white);
        QBrush blueBrush(Qt::blue);
        QBrush yellowBrush(Qt::yellow);
        QPen pen(Qt::black);
        int viewportWidth = ui->mapView->width();
        int viewportHeight = ui->mapView->height();
        int cellWidth = floor(viewportWidth/this->mapWidth);
        int cellHeight = floor(viewportHeight/this->mapHeight);

        QVector<QPoint> pathCells;
        for (int i = 0; i < this->path->size(); i++) {
            QPointF pointF = this->path->at(i);
            QPoint point;
            point.setX(round(pointF.x()/this->mapResolution));
            point.setY(round(pointF.y()/this->mapResolution));
            pathCells.push_back(point);
        }

        int robotX = round(this->robotPosition.x()/this->mapResolution);
        int robotY = round(this->robotPosition.y()/this->mapResolution);
        robotX = std::max(0, robotX);
        robotY = std::max(0, robotY);

        for (int i = 0; i < this->mapHeight; i++) {
            for (int j = 0; j < this->mapWidth; j++) {
                uint8_t occupancy = this->occupancyGrid->at(i*this->mapWidth+j);
                QBrush brush;
                QPoint point;
                point.setX(j);
                point.setY(i);
                if (occupancy > OCCUPANCY_THRESHOLD) {
                    brush = redBrush;
                }
                else if (j == robotX && i == robotY) {
                    brush = yellowBrush;
                }
                else if (pathCells.contains(point)) {
                    brush = blueBrush;
                }
                else {
                    brush = whiteBrush;
                }
                OccupancyGraphicsItem *rect = new OccupancyGraphicsItem(point, QRect(j*cellWidth-viewportWidth/2, i*cellHeight-viewportHeight/2, cellWidth, cellHeight), 0);
                rect->setBrush(brush);
                rect->setPen(pen);

                QObject::connect(rect, SIGNAL(clicked(QPoint)), this, SLOT(mapCell_clicked(QPoint)));

                this->mapScene->addItem(rect);
            }
        }


//        QGraphicsPixmapItem *pixmapItem = new QGraphicsPixmapItem(QPixmap::fromImage(QImage("robot.png")));
////        pixmapItem->setPos(robotX*cellWidth-viewportWidth/2, robotY*cellHeight-viewportHeight/2);
//        this->mapScene->addItem(pixmapItem);

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

    connect(this->outgoingSocket, SIGNAL(connected()), this, SLOT(socketConnected()));
    connect(this->outgoingSocket, SIGNAL(disconnected()), this, SLOT(socketDisconnected()));
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

void MainWindow::mapCell_clicked(QPoint coordinate)
{
    qDebug() << "Coordinate " << coordinate.x() << "," << coordinate.y() << " clicked";
    this->goal = coordinate;
    this->postData(ROUTE);
    //TODO: Request a path here
}

void MainWindow::socketConnected()
{
    ui->connectionLabel->setText("Robot connected");
}

void MainWindow::socketDisconnected()
{
    ui->connectionLabel->setText("Robot disconnected");
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


