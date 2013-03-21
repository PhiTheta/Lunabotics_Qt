#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "preferencedialog.h"
#include "constants.h"
#include "occupancygraphicsitem.h"
#include "SleepSimulator.h"
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

#define OCCUPANCY_THRESHOLD     80

union BytesToFloatInt {
    char    bytes[4];
    float   floatValue;
    int   intValue;
};

union BytesToUint8 {
    char bytes[1];
    uint8_t uint8Value;
};

union BytesToDouble {
    char bytes[8];
    double doubleValue;
};

enum RX_CONTENT_TYPE {
    TELEMETRY       = 0,
    MAP             = 1,
    PATH            = 2,
    LASER           = 3
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

    this->nextWaypointIdx = -1;

    QString text;
    ui->ackermannLinearSpeedEdit->setText(text.setNum(DEFAULT_LINEAR_SPEED_LIMIT));
    ui->lateralLinearSpeedEdit->setText(text.setNum(DEFAULT_LINEAR_SPEED_LIMIT));
    ui->ackermanDependentValueEdit->setText(text.setNum(DEFAULT_WHEEL_ROTATION_ANGLE_LIMIT));
    ui->lateralDependentValueEdit->setText(text.setNum(DEFAULT_LATERAL_SPEED_LIMIT));

    this->pathTableModel = new QStandardItemModel(0, 2, this); //2 Rows and 3 Columns
    this->pathTableModel->setHorizontalHeaderItem(0, new QStandardItem(QString("x")));
    this->pathTableModel->setHorizontalHeaderItem(1, new QStandardItem(QString("y")));
    ui->pathTableView->setModel(this->pathTableModel);


    this->mapViewportWidth = ui->mapView->width()-30;
    this->mapViewportHeight = ui->mapView->height()-15;
    this->mapCellWidth = 0;
    this->mapCellHeight = 0;


    this->redrawMap();

    this->connectRobot();

    this->autonomyEnabled = false;
    this->controlType = ACKERMANN;
    ui->driveForwardLabel->setStyleSheet("background-color : blue; color : white;");
    ui->driveLeftLabel->setStyleSheet("background-color : blue; color : white;");
    ui->driveRightLabel->setStyleSheet("background-color : blue; color : white;");
    ui->driveBackLabel->setStyleSheet("background-color : blue; color : white;");

    ui->autonomyLabel->setStyleSheet("background-color : yellow; color : black;");
    ui->collisionLabel->setStyleSheet("background-color : red; color : black;");
    ui->autonomyLabel->setVisible(false);
    ui->collisionLabel->setVisible(false);
    ui->mapView->setStyleSheet("background: transparent");
    ui->mechanicsGraphicsView->setStyleSheet("background: transparent");
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


void MainWindow::toggleAutonomy()
{
    if (this->autonomyEnabled) {
        qDebug() << "Disabling autonomy";
        ui->autonomyButton->setText("Enable autonomy");
        this->autonomyEnabled = false;
    }
    else {
        qDebug() << "Enabling autonomy";
        ui->autonomyButton->setText("Disable autonomy");
        this->autonomyEnabled = true;
    }
    ui->autonomyLabel->setVisible(this->autonomyEnabled && this->isDriving);
    this->postData(AUTONOMY);
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

    this->path->clear();

    QPointF point;
    point.setX(2);
    point.setY(2);
    this->path->push_back(point);
    point.setX(4);
    point.setY(6);
    this->path->push_back(point);
    point.setX(8);
    point.setY(7);
    this->path->push_back(point);
    point.setX(1);
    point.setY(1);
    this->path->push_back(point);

    this->pathTableModel->clear();


    qDebug() << "TEsting Map";




*/

    //////////////////////////////////

    if (this->mapWidth > 0 && this->mapHeight > 0) {


        QBrush whiteBrush(Qt::white);
        QBrush blueBrush(Qt::blue);
        QBrush clearBrush(Qt::transparent);
        QPen blackPen(Qt::black);
        QPen clearPen(Qt::transparent);
        int robotX = round(this->robotPosition.x()/this->mapResolution);
        int robotY = round(this->robotPosition.y()/this->mapResolution);
        robotX = std::max(0, robotX);
        robotY = std::max(0, robotY);
        ui->gridPositionLabel->setText(QString("%1,%2").arg(QString::number(robotX)).arg(QString::number(robotY)));

        for (int i = 0; i < this->mapHeight; i++) {
            for (int j = 0; j < this->mapWidth; j++) {
                uint8_t occupancy = this->occupancyGrid->at(i*this->mapWidth+j);
                QBrush brush;
                QPoint point;
                point.setX(j);
                point.setY(i);
                if (j == robotX && i == robotY) {
                    brush = blueBrush;
                }
                else {
                    int red = occupancy * 2.55;
                    int green = (100-occupancy) * 2.55;
                    int blue = std::min(0, (100-abs(occupancy-50))) * 50;
                    QBrush varBrush(QColor(red, green, blue));
                    brush = varBrush;
                }
                OccupancyGraphicsItem *rect = new OccupancyGraphicsItem(point, QRect(this->mapViewportWidth-(j+1)*this->mapCellWidth, i*this->mapCellHeight-this->mapViewportHeight/2, this->mapCellWidth, this->mapCellHeight), 0);
                rect->setBrush(brush);
                rect->setPen(clearPen);

                QObject::connect(rect, SIGNAL(clicked(QPoint)), this, SLOT(mapCell_clicked(QPoint)));
                QObject::connect(rect, SIGNAL(hovered(QPoint)), this, SLOT(mapCell_hovered(QPoint)));

                this->mapScene->addItem(rect);
            }
        }

        QBrush redBrush(Qt::red);
        QBrush yellowBrush(Qt::yellow);
        QBrush purpleBrush(QColor(255,0,255));
        QPen yellowPen(Qt::yellow);
        QPen whitePen(Qt::white);
        QPen purplePen(QColor(255,0,255));
        yellowPen.setWidth(2);
        whitePen.setWidth(2);

        this->pathTableModel->clear();
        if (this->controlType == TURN_IN_SPOT) {
            QVector<QPoint> pathCells;
            for (int i = 0; i < this->path->size(); i++) {
                QPointF pointF = this->path->at(i);
                QPoint point;
                point.setX(round(pointF.x()/this->mapResolution));
                point.setY(round(pointF.y()/this->mapResolution));
                pathCells.push_back(point);
            }
            for (int i = 1; i < this->path->size(); i++) {
                QPointF pathPoint1 = this->mapPoint(this->path->at(i-1));
                QPointF pathPoint2 = this->mapPoint(this->path->at(i));
                this->mapScene->addLine(pathPoint1.x(), pathPoint1.y(), pathPoint2.x(), pathPoint2.y(), whitePen);
            }
            for (int i = 0; i < pathCells.size(); i++) {
                this->mapScene->addEllipse(this->mapViewportWidth-(pathCells.at(i).x()+1)*this->mapCellWidth, pathCells.at(i).y()*this->mapCellHeight-this->mapViewportHeight/2, this->mapCellWidth, this->mapCellHeight, yellowPen, i == this->nextWaypointIdx ? purpleBrush : redBrush);
                QStandardItem *row = new QStandardItem(QString::number(pathCells.at(i).x()));
                this->pathTableModel->setItem(i,0,row);
                row = new QStandardItem(QString::number(pathCells.at(i).y()));
                this->pathTableModel->setItem(i,1,row);
            }
        }
        else {
            if (this->path->size() > 0) {
                QPointF p = this->mapPoint(this->path->at(0));
                this->mapScene->addEllipse(p.x()-2, p.y()-2, 4, 4, yellowPen, yellowBrush);
            }
            for (int i = 1; i < this->path->size(); i++) {
                QPointF pathPoint1 = this->mapPoint(this->path->at(i-1));
                QPointF pathPoint2 = this->mapPoint(this->path->at(i));
                this->mapScene->addLine(pathPoint1.x(), pathPoint1.y(), pathPoint2.x(), pathPoint2.y(), whitePen);
                this->mapScene->addEllipse(pathPoint2.x()-2, pathPoint2.y()-2, 4, 4, yellowPen, yellowBrush);
            }
            if (this->nextWaypointIdx >= 0 && this->nextWaypointIdx < this->path->size()) {
                QPointF p = this->mapPoint(this->path->at(this->nextWaypointIdx));
                this->mapScene->addEllipse(p.x()-3, p.y()-3, 6, 6, purplePen, purpleBrush);
            }
        }

        QPointF robotCenter = this->mapPoint(this->robotPosition);

        if (this->controlType == ACKERMANN && this->isDriving) {
            QPointF closestPoint = this->mapPoint(this->closestTrajectoryPoint);
            qDebug() << "CLosest point " << closestPoint.x() << ", " << closestPoint.y();
            if (closestPoint.x() <= this->mapViewportWidth && closestPoint.y() <= this->mapViewportHeight) {
                this->mapScene->addLine(robotCenter.x(), robotCenter.y(), closestPoint.x(), closestPoint.y(), purplePen);
            }
        }

        qreal robotRadius = 10;
        qreal pointerX = robotCenter.x()-robotRadius*cos(this->robotAngle);
        qreal pointerY = robotCenter.y()+robotRadius*sin(this->robotAngle);

        this->mapScene->addEllipse(robotCenter.x()-robotRadius, robotCenter.y()-robotRadius, robotRadius*2, robotRadius*2, blackPen, clearBrush);
        this->mapScene->addLine(robotCenter.x(), robotCenter.y(), pointerX, pointerY);

//        QGraphicsPixmapItem *pixmapItem = new QGraphicsPixmapItem(QPixmap::fromImage(QImage("robot.png")));
////        pixmapItem->setPos(robotX*cellWidth-viewportWidth/2, robotY*cellHeight-viewportHeight/2);
//        this->mapScene->addItem(pixmapItem);




        //Draw bezier curves

//        QPainter painter(this);
//        painter.setRenderHint(QPainter::Antialiasing, true);

//        QPainterPath path;
//        path.moveTo(80, 320);
//        path.cubicTo(200, 80, 320, 80, 480, 320);

//        painter.setPen(QPen(Qt::black, 8));
//        painter.drawPath(path);


    }
}

QPointF MainWindow::mapPoint(QPointF pointInMeters)
{
    qreal x = this->mapViewportWidth-(pointInMeters.x()/this->mapResolution+0.5)*this->mapCellWidth;
    qreal y = (pointInMeters.y()/this->mapResolution+0.5)*this->mapCellHeight-this->mapViewportHeight/2;
    return QPointF(x, y);
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



    connect(this->outgoingSocket, SIGNAL(connected()), this, SLOT(outSocketConnected()));
    connect(this->outgoingSocket, SIGNAL(disconnected()), this, SLOT(outSocketDisconnected()));
    this->outgoingSocket->connectToHost(settings.value("out_ip", CONN_OUTGOING_ADDR).toString(), settings.value("out_port", CONN_OUTGOING_PORT).toInt());

    if (!this->incomingServer) {
        this->incomingServer = new QTcpServer(this);
        connect(this->incomingServer, SIGNAL(newConnection()), this, SLOT(serverAcceptConnection()));
    }


    QHostAddress inAddr = QHostAddress::Any;
    QString inIPString = settings.value("in_ip", CONN_INCOMING_ADDR).toString();
    if (inIPString.compare("any") != 0) {
        inAddr = QHostAddress(inIPString);
    }


    if (!this->incomingServer->listen(inAddr, settings.value("in_port", CONN_INCOMING_PORT).toInt())) {
        qDebug() << "Failed to start listening for " << settings.value("in_ip", CONN_INCOMING_ADDR).toString();
    }

    settings.endGroup();
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
        double wXValue = this->decodeDouble(buffer, pointer);
        double wYValue = this->decodeDouble(buffer, pointer);
        double wZValue = this->decodeDouble(buffer, pointer);
        uint8_t controlMode = this->decodeByte(buffer, pointer);
        this->isDriving = this->decodeByte(buffer, pointer);

        this->robotControlType = (CTRL_MODE_TYPE)controlMode;

        if (this->isDriving) {
            this->nextWaypointIdx = this->decodeInt(buffer, pointer)-1;

            if (this->robotControlType == ACKERMANN) {
                double yErrValue = this->decodeDouble(buffer, pointer);
                double closestTrajectoryXValue = this->decodeDouble(buffer, pointer);
                double closestTrajectoryYValue = this->decodeDouble(buffer, pointer);
                this->closestTrajectoryPoint.setX(closestTrajectoryXValue);
                this->closestTrajectoryPoint.setY(closestTrajectoryYValue);
                ui->yErrLabel->setText(QString("%1").arg(QString::number(yErrValue, 'f', 3)));
                ui->closestTrajectoryPointLabel->setText(QString("%1, %2 (%3, %4 on the map)").arg(QString::number(closestTrajectoryXValue, 'f', 2)).arg(QString::number(closestTrajectoryYValue, 'f', 2)).arg(QString::number(round(closestTrajectoryXValue/this->mapResolution), 'f', 0)).arg(QString::number(round(closestTrajectoryYValue/this->mapResolution), 'f', 0)));
            }
            else {
                ui->closestTrajectoryPointLabel->setText("N/A");
                ui->yErrLabel->setText("N/A");
            }
        }
        else {
            ui->closestTrajectoryPointLabel->setText("N/A");
            ui->yErrLabel->setText("N/A");
        }

        ui->autonomyLabel->setVisible(this->autonomyEnabled && this->isDriving);

        ui->posXLabel->setText(QString("%1 m").arg(QString::number(posXValue, 'f', 2)));
        ui->posYLabel->setText(QString("%1 m").arg(QString::number(posYValue, 'f', 2)));
        ui->orLabel->setText(QString("%1 rad").arg(QString::number(orValue, 'f', 2)));
        ui->vXLabel->setText(QString("%1 m/s").arg(QString::number(vXValue, 'f', 2)));
        ui->vYLabel->setText(QString("%1 m/s").arg(QString::number(vYValue, 'f', 2)));
        ui->vZLabel->setText(QString("%1 m/s").arg(QString::number(vZValue, 'f', 2)));
        ui->wXLabel->setText(QString("%1 rad/s").arg(QString::number(wXValue, 'f', 2)));
        ui->wYLabel->setText(QString("%1 rad/s").arg(QString::number(wYValue, 'f', 2)));
        ui->wZLabel->setText(QString("%1 rad/s").arg(QString::number(wZValue, 'f', 2)));

        switch (this->robotControlType) {
        case ACKERMANN: ui->controlModeLabel->setText("Ackermann"); break;
        case TURN_IN_SPOT: ui->controlModeLabel->setText("'Turn in spot'"); break;
        case LATERAL: ui->controlModeLabel->setText("Lateral"); break;
        default: ui->controlModeLabel->setText("Undefined"); break;
        }




        this->robotPosition.setX(posXValue);
        this->robotPosition.setY(posYValue);
        this->robotAngle = orValue;

        this->redrawMap();
    }
        break;

    case MAP: {
        this->mapWidth = this->decodeByte(buffer, pointer);
        this->mapHeight = this->decodeByte(buffer, pointer);
        this->mapResolution = this->decodeDouble(buffer, pointer);
        ui->mapResolutionLabel->setText(QString("1 cell = %1x%2m").arg(QString::number(this->mapResolution, 'f', 2)).arg(QString::number(this->mapResolution, 'f', 2)));
        this->occupancyGrid->clear();

        this->mapCellWidth = floor(this->mapViewportWidth/this->mapWidth);
        this->mapCellHeight = floor(this->mapViewportHeight/this->mapHeight);


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
        break;

    case LASER: {
/*
        float angleMin = this->decodeFloat(buffer, pointer);
        float angleMax = this->decodeFloat(buffer, pointer);
        float angleIncrement = this->decodeFloat(buffer, pointer);
        int numOfRanges = this->decodeInt(buffer, pointer);

        QVector<float> ranges;
        for (int i = 0; i < numOfRanges; i++) {
            float range = this->decodeFloat(buffer, pointer);
            ranges.push_back(range);
        }

        this->laserScan.setAngleMin(angleMin);
        this->laserScan.setAngleMax(angleMax);
        this->laserScan.setAngleIncrement(angleIncrement);
        this->laserScan.setRanges(ranges);

        qDebug() << "Getting laser between angles " << this->laserScan.getAngleMin() << " and " << this->laserScan.getAngleMax();

        this->redrawMap();
        */
    }
        break;

    default:
        break;
    }

    delete buffer;
}
void MainWindow::postData(TX_CONTENT_TYPE contentType)
{

    this->outgoingSocket->abort();
    QSettings settings("ivany4", "lunabotics");
    settings.beginGroup("connection");
    this->outgoingSocket->connectToHost(settings.value("out_ip", CONN_OUTGOING_ADDR).toString(), settings.value("out_port", CONN_OUTGOING_PORT).toInt());
    settings.endGroup();

    union BytesToFloatInt floatConverter;

    QByteArray *bytes = new QByteArray();
    bytes->append(contentType);


    switch (contentType) {
    case AUTONOMY:
        bytes->append(this->autonomyEnabled);
        break;
    case CTRL_MODE:
        bytes->append(this->controlType);
        if (this->controlType == ACKERMANN) {
            float linearSpeedLimit = DEFAULT_LATERAL_SPEED_LIMIT;
            float controlDependentLimit = DEFAULT_WHEEL_ROTATION_ANGLE_LIMIT;
            if (ui->ackermannLinearSpeedCheckBox->checkState() == Qt::Checked) {
                linearSpeedLimit = ui->ackermannLinearSpeedEdit->text().toFloat();
            }
            if (ui->ackermannDependentValueCheckBox->checkState() == Qt::Checked) {
                controlDependentLimit = ui->ackermanDependentValueEdit->text().toFloat();
            }
            floatConverter.floatValue = linearSpeedLimit;
            bytes->append(floatConverter.bytes, sizeof(float));
            floatConverter.floatValue = controlDependentLimit;
            bytes->append(floatConverter.bytes, sizeof(float));
        }
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
        floatConverter.floatValue = ui->spotAngleTolerance->text().toFloat();
        bytes->append(floatConverter.bytes, sizeof(float));
        floatConverter.floatValue = ui->spotDistanceTolerance->text().toFloat();
        bytes->append(floatConverter.bytes, sizeof(float));
        break;

    case MAP_REQUEST:
        //Nothing to include
        break;

    case PID:
        settings.beginGroup("pid");
        floatConverter.floatValue = settings.value("p", PID_KP).toFloat();
        bytes->append(floatConverter.bytes, sizeof(float));
        floatConverter.floatValue = settings.value("i", PID_KI).toFloat();
        bytes->append(floatConverter.bytes, sizeof(float));
        floatConverter.floatValue = settings.value("d", PID_KD).toFloat();
        bytes->append(floatConverter.bytes, sizeof(float));
        settings.endGroup();
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


void MainWindow::mapCell_clicked(QPoint coordinate)
{
    this->goal = coordinate;
    if (!this->autonomyEnabled) {
        this->toggleAutonomy();
    }
    this->postData(ROUTE);
}

void MainWindow::mapCell_hovered(QPoint coordinate)
{
    ui->cursorPositionLabel->setText(QString("%1,%2").arg(QString::number(coordinate.x())).arg(QString::number(coordinate.y())));
}

void MainWindow::outSocketConnected()
{
    ui->outConnectionLabel->setText("Sending socket: connected");
}

void MainWindow::outSocketDisconnected()
{
    ui->outConnectionLabel->setText("Sending socket: disconnected");
}

void MainWindow::on_ackermannLinearSpeedCheckBox_clicked(bool checked)
{
    ui->ackermannLinearSpeedEdit->setEnabled(checked);
}

void MainWindow::on_ackermannDependentValueCheckBox_clicked(bool checked)
{
    ui->ackermanDependentValueEdit->setEnabled(checked);
}

void MainWindow::on_lateralLinearSpeedCheckBox_clicked(bool checked)
{
    ui->lateralLinearSpeedEdit->setEnabled(checked);
}

void MainWindow::on_lateralDependentValueCheckBox_clicked(bool checked)
{
    ui->lateralDependentValueEdit->setEnabled(checked);
}


void MainWindow::on_actionPreferences_triggered()
{
    PreferenceDialog *preferenceDialog = new PreferenceDialog(this);
    preferenceDialog->setWindowModality(Qt::WindowModal);
    if (preferenceDialog->exec() == QDialog::Accepted) {
        this->connectRobot();
        this->postData(PID);
    }
}

void MainWindow::on_autonomyButton_clicked()
{
    this->toggleAutonomy();
}

void MainWindow::on_actionExit_triggered()
{
    QApplication::quit();
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

void MainWindow::on_refreshMapButton_clicked()
{
    this->postData(MAP_REQUEST);
}

void MainWindow::on_resendParamsButton_clicked()
{
    SleepSimulator sim;
    this->postData(CTRL_MODE);
    sim.sleep(1000);
    this->postData(PID);
    sim.sleep(1000);
    this->postData(AUTONOMY);
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
    if (this->autonomyEnabled) {
        this->toggleAutonomy();
    }
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
    if (this->autonomyEnabled) {
        this->toggleAutonomy();
    }
}

double MainWindow::decodeDouble(char buffer[], int &pointer)
{
    BytesToDouble converter;
    for (unsigned int i = 0; i < sizeof(double); i++) {
        converter.bytes[i] = buffer[pointer++];
    }
    return converter.doubleValue;
}

uint8_t MainWindow::decodeByte(char buffer[], int &pointer)
{
    BytesToUint8 converter;
    converter.bytes[0] = buffer[pointer++];
    return converter.uint8Value;
}

int MainWindow::decodeInt(char buffer[], int &pointer)
{
    BytesToFloatInt converter;
    for (unsigned int i = 0; i < sizeof(int); i++) {
        converter.bytes[i] = buffer[pointer++];
    }
    return converter.intValue;
}

float MainWindow::decodeFloat(char buffer[], int &pointer)
{
    BytesToFloatInt converter;
    for (unsigned int i = 0; i < sizeof(float); i++) {
        converter.bytes[i] = buffer[pointer++];
    }
    return converter.floatValue;
}
