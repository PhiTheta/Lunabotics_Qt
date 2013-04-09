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
#include "Telemetry.pb.h"

#define DEFAULT_LINEAR_SPEED_LIMIT  0.33
#define DEFAULT_WHEEL_ROTATION_ANGLE_LIMIT  45
#define DEFAULT_SPOT_ROTATIONAL_SPEED_LIMIT 1.0
#define DEFAULT_LATERAL_SPEED_LIMIT 2.0
#define DEFAULT_BEZIER_SEGMENTS 20

#define OCCUPANCY_THRESHOLD     80

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    GOOGLE_PROTOBUF_VERIFY_VERSION;


    ui->setupUi(this);

    this->outgoingSocket = NULL;
    this->incomingServer = NULL;
    this->incomingSocket = NULL;
    this->mapWidth = this->mapHeight = 0;

    this->mapScene = new QGraphicsScene(this);
    ui->mapView->setScene(this->mapScene);
    this->occupancyGrid = new QVector<int>();
    this->path = new QVector<QPointF>();

    this->localFrameScene = new QGraphicsScene(this);
    ui->localFrameView->setScene(this->localFrameScene);

    this->nextWaypointIdx = -1;

    QString text;

    QSettings settings("ivany4", "lunabotics");
    settings.beginGroup("control");
    ui->ackermannLinearSpeedEdit->setText(settings.value("ackermann.v", DEFAULT_LINEAR_SPEED_LIMIT).toString());
    ui->bezierSegmentsEdit->setText(settings.value("ackermann.bezier", DEFAULT_BEZIER_SEGMENTS).toString());
    settings.endGroup();


    ui->ackermannLinearSpeedEdit->setText(text.setNum(DEFAULT_LINEAR_SPEED_LIMIT));

    this->pathTableModel = new QStandardItemModel(0, 2, this); //2 Rows and 3 Columns
    this->pathTableModel->setHorizontalHeaderItem(0, new QStandardItem(QString("x")));
    this->pathTableModel->setHorizontalHeaderItem(1, new QStandardItem(QString("y")));
    ui->pathTableView->setModel(this->pathTableModel);


    this->mapViewportWidth = ui->mapView->width()-30;
    this->mapViewportHeight = ui->mapView->height()-15;
    this->mapCellWidth = 0;
    this->mapCellHeight = 0;
    this->localFrameViewportWidth = ui->localFrameView->width();
    this->localFrameViewportHeight = ui->localFrameView->height();


    this->redrawMap();

    this->connectRobot();

    this->autonomyEnabled = false;
    this->controlType = lunabotics::ACKERMANN;
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
    google::protobuf::ShutdownProtobufLibrary();

    QSettings settings("ivany4", "lunabotics");
    settings.beginGroup("control");
    settings.setValue("ackermann.v", ui->ackermannLinearSpeedEdit->text());
    settings.setValue("ackermann.bezier", ui->bezierSegmentsEdit->text());
    settings.endGroup();

    delete ui;
    this->disconnectRobot();

    delete this->incomingServer;
    delete this->outgoingSocket;
    delete this->mapScene;
    delete this->localFrameScene;
    delete this->occupancyGrid;
    delete this->path;
}


void MainWindow::toggleAutonomy()
{
    this->setAutonomy(!this->autonomyEnabled);
}

void MainWindow::setAutonomy(bool enabled)
{
    if (enabled != this->autonomyEnabled) {
        this->autonomyEnabled = enabled;
        if (enabled) {
            this->setAutonomy(true);
            qDebug() << "Enabling autonomy";
        }
        else {
            this->setAutonomy(false);
            qDebug() << "Disabling autonomy";
        }
        this->postData(lunabotics::Telecommand::SET_AUTONOMY);
    }
}

void MainWindow::setAutonomyLabel(bool enabled)
{
    ui->autonomyLabel->setVisible(enabled);
    QRect currentGeometry = ui->mainWidget->geometry();
    if (enabled) {
        currentGeometry.setY(31);
        ui->autonomyButton->setText("Disable autonomy");
    }
    else {
        currentGeometry.setY(0);
        ui->autonomyButton->setText("Enable autonomy");
    }
    ui->mainWidget->setGeometry(currentGeometry);
}


void MainWindow::redrawMap()
{
    qDeleteAll(this->mapScene->items());
    qDeleteAll(this->localFrameScene->items());

    if (!this->autonomyEnabled) {
        this->path->clear();
    }

    //// DRAW LOCAL FRAME ////////

    qreal originX = ui->localFrameView->width()/2;
    qreal originY = ui->localFrameView->height()/2;
    qreal x = originX-this->transformedVelocityPoint.x()/this->mapResolution*10;
    qreal y = originY-this->transformedVelocityPoint.y()/this->mapResolution*10;

    this->localFrameScene->addEllipse(originX-2, originY-2, 4, 4, QPen(Qt::red), QBrush(Qt::red));
    this->localFrameScene->addEllipse(x-1, y-1, 2, 2, QPen(Qt::blue), QBrush(Qt::blue));
    this->localFrameScene->addLine(x, y, originX, originY, QPen(Qt::red));
    qreal x1 = originX-this->transformedClosestTrajectoryPoint.x()/this->mapResolution*10;
    qreal y1 = originY-this->transformedClosestTrajectoryPoint.y()/this->mapResolution*10;
    this->localFrameScene->addEllipse(x1-1, y1-1, 2, 2, QPen(Qt::black), QBrush(Qt::black));
    this->localFrameScene->addLine(x, y, x1, y1, QPen(Qt::blue));

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
        QPen redPen(Qt::red);
        QPen yellowPen(Qt::yellow);
        QPen whitePen(Qt::white);
        QPen purplePen(QColor(255,0,255));
        yellowPen.setWidth(2);
        whitePen.setWidth(2);

        this->pathTableModel->clear();
        if (this->controlType == lunabotics::TURN_IN_SPOT) {
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

        if (this->controlType == lunabotics::ACKERMANN && this->autonomyEnabled) {
            QPointF closestPoint = this->mapPoint(this->closestTrajectoryPoint);
            QPointF velocityPoint = this->mapPoint(this->velocityPoint);
            qDebug() << "CLosest point " << closestPoint.x() << ", " << closestPoint.y();
            if (velocityPoint.x() <= this->mapViewportWidth && velocityPoint.y() <= this->mapViewportHeight) {
                this->mapScene->addLine(robotCenter.x(), robotCenter.y(), velocityPoint.x(), velocityPoint.y(), redPen);
            }
            if (closestPoint.x() <= this->mapViewportWidth && closestPoint.y() <= this->mapViewportHeight) {
                this->mapScene->addLine(velocityPoint.x(), velocityPoint.y(), closestPoint.x(), closestPoint.y(), purplePen);
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
    qint64 bytesAvailable = this->incomingSocket->bytesAvailable();
    char *buffer = new char[bytesAvailable];
    this->incomingSocket->read(buffer, bytesAvailable);

//    qDebug() << "Getting " << bytesAvailable << "bytes";

    lunabotics::Telemetry tm;
    if (!tm.ParseFromArray(buffer, bytesAvailable) || !tm.IsInitialized()) {
        qDebug() << "Failed to parse telemetry object";
    }
    else {
        bool renderMap = false;
        if (tm.has_state_data()) {
            const lunabotics::Telemetry::State state = tm.state_data();

            this->robotPosition.setX(state.position().x());
            this->robotPosition.setY(state.position().y());
            this->robotAngle = state.heading();


            ui->posXLabel->setText(QString("%1 m").arg(QString::number(this->robotPosition.x(), 'f', 2)));
            ui->posYLabel->setText(QString("%1 m").arg(QString::number(this->robotPosition.y(), 'f', 2)));
            ui->orLabel->setText(QString("%1 rad").arg(QString::number(this->robotAngle, 'f', 2)));
            ui->vXLabel->setText(QString("%1 m/s").arg(QString::number(state.velocities().linear(), 'f', 2)));
            ui->wZLabel->setText(QString("%1 rad/s").arg(QString::number(state.velocities().angular(), 'f', 2)));

            this->robotControlType = state.steering_mode();
            this->autonomyEnabled = state.autonomy_enabled();
            this->setAutonomyLabel(this->autonomyEnabled);

            if (state.has_next_waypoint_idx()) {
                this->nextWaypointIdx = state.next_waypoint_idx()-1;
            }

            if (state.has_ackermann_telemetry()) {

                const lunabotics::Telemetry::State::AckermannTelemetry params = state.ackermann_telemetry();

                ui->yErrLabel->setText(QString("%1 m").arg(QString::number(params.pid_error(), 'f', 3)));
                this->closestTrajectoryPoint.setX(params.closest_trajectory_point().x());
                this->closestTrajectoryPoint.setY(params.closest_trajectory_point().y());
                this->velocityPoint.setX(params.velocity_vector_point().x());
                this->velocityPoint.setY(params.velocity_vector_point().y());
                this->transformedClosestTrajectoryPoint.setX(params.closest_trajectory_local_point().x());
                this->transformedClosestTrajectoryPoint.setY(params.closest_trajectory_local_point().y());
                this->transformedVelocityPoint.setX(params.velocity_vector_local_point().x());
                this->transformedVelocityPoint.setY(params.velocity_vector_local_point().y());

                ui->pidGroupBox->setVisible(true);
                ui->transformedTrajectoryXLabel->setText(QString("x: %1 m").arg(QString::number(this->transformedClosestTrajectoryPoint.x(), 'f', 2)));
                ui->transformedTrajectoryYLabel->setText(QString("y: %1 m").arg(QString::number(this->transformedClosestTrajectoryPoint.y(), 'f', 2)));
                ui->transformedReferenceXLabel->setText(QString("x: %1 m").arg(QString::number(this->transformedVelocityPoint.x(), 'f', 2)));
                ui->transformedReferenceYLabel->setText(QString("y: %1 m").arg(QString::number(this->transformedVelocityPoint.y(), 'f', 2)));
                ui->closestTrajectoryXLabel->setText(QString("x: %1 m").arg(QString::number(this->closestTrajectoryPoint.x(), 'f', 2)));
                ui->closestTrajectoryYLabel->setText(QString("y: %1 m").arg(QString::number(this->closestTrajectoryPoint.y(), 'f', 2)));
            }
            else {
                ui->pidGroupBox->setVisible(false);
            }

            switch (this->robotControlType) {
            case lunabotics::ACKERMANN: ui->controlModeLabel->setText("Ackermann"); break;
            case lunabotics::TURN_IN_SPOT: ui->controlModeLabel->setText("'Turn in spot'"); break;
            case lunabotics::CRAB: ui->controlModeLabel->setText("Crab"); break;
            default: ui->controlModeLabel->setText("Undefined"); break;
            }

            renderMap = true;
        }

        if (tm.has_world_data()) {
            const lunabotics::Telemetry::World world = tm.world_data();
            int width = world.width();
            int height = world.height();
            if (world.cell_size() != width*height) {
                qDebug() << "ERROR: World dimensions mismatch. Declared width and height don't correspond to number of cells in the message";
            }
            else if (width == 0 || height == 0) {
                qDebug() << "ERROR: Receiving zero dimensions for map";
            }
            else {
                this->mapWidth = width;
                this->mapHeight = height;
                this->mapResolution = world.resolution();
                ui->mapResolutionLabel->setText(QString("1 cell = %1x%2m").arg(QString::number(this->mapResolution, 'f', 2)).arg(QString::number(this->mapResolution, 'f', 2)));
                this->occupancyGrid->clear();

                this->mapCellWidth = floor(this->mapViewportWidth/this->mapWidth);
                this->mapCellHeight = floor(this->mapViewportHeight/this->mapHeight);

                for (int i = 0; i < world.cell_size(); i++) {
                    this->occupancyGrid->push_back(world.cell(i));
                }
            }
            renderMap = true;
        }

        if (tm.has_path_data()) {
            this->path->clear();

            const lunabotics::Telemetry::Path path = tm.path_data();

            for (int i = 0; i < path.position_size(); i++) {
                const lunabotics::Point position = path.position(i);
                QPointF point;
                point.setX(position.x());
                point.setY(position.y());
                this->path->push_back(point);
            }

            renderMap = true;
        }

        if (tm.has_laser_scan_data()) {
        }

        if (renderMap) {
            this->redrawMap();
        }
    }

    delete buffer;

}
void MainWindow::postData(lunabotics::Telecommand::Type contentType)
{
    this->socketMutex.lock();
    qDebug() << "=======ENTERING=========";
    this->outgoingSocket->abort();
    QSettings settings("ivany4", "lunabotics");
    settings.beginGroup("connection");
    this->outgoingSocket->connectToHost(settings.value("out_ip", CONN_OUTGOING_ADDR).toString(), settings.value("out_port", CONN_OUTGOING_PORT).toInt());
    settings.endGroup();

    lunabotics::Telecommand tc;
    tc.set_type(contentType);

    switch (contentType) {
    case lunabotics::Telecommand::SET_AUTONOMY:
        tc.mutable_autonomy_data()->set_enabled(this->autonomyEnabled);
        break;
    case lunabotics::Telecommand::STEERING_MODE: {
        lunabotics::Telecommand::SteeringMode *steeringMode = tc.mutable_steering_mode_data();
        steeringMode->set_type(this->controlType);
        if (this->controlType == lunabotics::ACKERMANN) {
            lunabotics::Telecommand::SteeringMode::AckermannSteeringData *steeringData = steeringMode->mutable_ackermann_steering_data();
            steeringData->set_max_linear_velocity(ui->ackermannLinearSpeedEdit->text().toFloat());
            steeringData->set_bezier_curve_segments(ui->bezierSegmentsEdit->text().toInt());
        }
    }
        break;
    case lunabotics::Telecommand::TELEOPERATION: {
        lunabotics::Telecommand::Teleoperation *teleoperation = tc.mutable_teleoperation_data();
        teleoperation->set_forward(this->drivingMask & FORWARD);
        teleoperation->set_backward(this->drivingMask & BACKWARD);
        teleoperation->set_left(this->drivingMask & LEFT);
        teleoperation->set_right(this->drivingMask & RIGHT);
    }
        break;
    case lunabotics::Telecommand::DEFINE_ROUTE: {
        lunabotics::Telecommand::DefineRoute *route = tc.mutable_define_route_data();
        route->mutable_goal()->set_x(this->goal.x()*this->mapResolution);
        route->mutable_goal()->set_y(this->goal.y()*this->mapResolution);
        route->set_heading_accuracy(ui->spotAngleTolerance->text().toFloat());
        route->set_position_accuracy(ui->spotDistanceTolerance->text().toFloat());
    }
        break;

    case lunabotics::Telecommand::REQUEST_MAP:
        //Nothing to include
        break;

    case lunabotics::Telecommand::ADJUST_PID: {
        lunabotics::Telecommand::AdjustPID *pid = tc.mutable_adjust_pid_data();
        settings.beginGroup("pid");
        pid->set_p(settings.value("p", PID_KP).toFloat());
        pid->set_i(settings.value("i", PID_KI).toFloat());
        pid->set_d(settings.value("d", PID_KD).toFloat());
        pid->set_velocity_offset(settings.value("offset", PID_OFFSET).toFloat());
        pid->set_velocity_multiplier(settings.value("v", PID_VEL_M).toFloat());
        settings.endGroup();
    }
        break;

    case lunabotics::Telecommand::ADJUST_WHEELS: {
        lunabotics::AllWheelControl::Wheels *steering = tc.mutable_all_wheel_control_data()->mutable_steering();
        lunabotics::AllWheelControl::Wheels *driving = tc.mutable_all_wheel_control_data()->mutable_driving();

        steering->set_left_front(ui->lfSteeringEdit->text().toFloat());
        steering->set_right_front(ui->rfSteeringEdit->text().toFloat());
        steering->set_left_rear(ui->lrSteeringEdit->text().toFloat());
        steering->set_right_rear(ui->rrSteeringEdit->text().toFloat());

        driving->set_left_front(ui->lfDrivingEdit->text().toFloat());
        driving->set_right_front(ui->rfDrivingEdit->text().toFloat());
        driving->set_left_rear(ui->lrDrivingEdit->text().toFloat());
        driving->set_right_rear(ui->rrDrivingEdit->text().toFloat());

    }
        break;

    default:
        break;
    }

    if (!tc.IsInitialized()) {
        qDebug() << "Telecommand not initialized";
        qDebug() << QString(tc.InitializationErrorString().c_str());
        return;
    }


    if (this->outgoingSocket->state() != QTcpSocket::UnconnectedState) {
        QByteArray byteArray(tc.SerializeAsString().c_str(), tc.ByteSize());
        qDebug() << "Sending " << byteArray.size() << " bytes";
        this->outgoingSocket->write(byteArray);
        this->outgoingSocket->waitForDisconnected(1);
    }
    else {
        qDebug() << "Socket is not connected!";
    }


    qDebug() << "=======EXITING=========";
    this->socketMutex.unlock();
}


void MainWindow::mapCell_clicked(QPoint coordinate)
{
    this->goal = coordinate;
    this->setAutonomy(true);
    this->postData(lunabotics::Telecommand::DEFINE_ROUTE);
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

void MainWindow::on_actionPreferences_triggered()
{
    PreferenceDialog *preferenceDialog = new PreferenceDialog(this);
    preferenceDialog->setWindowModality(Qt::WindowModal);
    if (preferenceDialog->exec() == QDialog::Accepted) {
        this->connectRobot();
        this->postData(lunabotics::Telecommand::ADJUST_PID);
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
    this->controlType = lunabotics::CRAB;
    this->postData(lunabotics::Telecommand::STEERING_MODE);
}

void MainWindow::on_useSpotButton_clicked()
{
    qDebug() << "Switching to spot driving mode";
    this->controlType = lunabotics::TURN_IN_SPOT;
    this->postData(lunabotics::Telecommand::STEERING_MODE);
}

void MainWindow::on_useAckermannButton_clicked()
{
    qDebug() << "Switching to Ackermann driving mode";
    this->controlType = lunabotics::ACKERMANN;
    this->postData(lunabotics::Telecommand::STEERING_MODE);
}

void MainWindow::on_refreshMapButton_clicked()
{
    qDeleteAll(this->mapScene->items());
    this->postData(lunabotics::Telecommand::REQUEST_MAP);
}

void MainWindow::on_resendParamsButton_clicked()
{
    SleepSimulator sim;
    this->postData(lunabotics::Telecommand::STEERING_MODE);
    sim.sleep(1000);
    this->postData(lunabotics::Telecommand::ADJUST_PID);
    sim.sleep(1000);
    this->postData(lunabotics::Telecommand::SET_AUTONOMY);
}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    this->setAutonomy(false);
    if (event->key() == Qt::Key_F5 || event->key() == Qt::Key_W) {
        this->drivingMask |= FORWARD;
        ui->driveForwardLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
    }
    if (event->key() == Qt::Key_F6 || event->key() == Qt::Key_A) {
        this->drivingMask |= LEFT;
        ui->driveLeftLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
    }
    if (event->key() == Qt::Key_F7 || event->key() == Qt::Key_S) {
        this->drivingMask |= BACKWARD;
        ui->driveBackLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
    }
    if (event->key() == Qt::Key_F8 || event->key() == Qt::Key_D) {
        this->drivingMask |= RIGHT;
        ui->driveRightLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
    }
    this->postData(lunabotics::Telecommand::TELEOPERATION);
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    this->setAutonomy(false);
    if (event->key() == Qt::Key_F5 || event->key() == Qt::Key_W) {
        this->drivingMask &= ~FORWARD;
        ui->driveForwardLabel->setStyleSheet("QLabel { background-color : blue; color : white; }");
    }
    if (event->key() == Qt::Key_F6 || event->key() == Qt::Key_A) {
        this->drivingMask &= ~LEFT;
        ui->driveLeftLabel->setStyleSheet("QLabel { background-color : blue; color : white; }");
    }
    if (event->key() == Qt::Key_F7 || event->key() == Qt::Key_S) {
        this->drivingMask &= ~BACKWARD;
        ui->driveBackLabel->setStyleSheet("QLabel { background-color : blue; color : white; }");
    }
    if (event->key() == Qt::Key_F8 || event->key() == Qt::Key_D) {
        this->drivingMask &= ~RIGHT;
        ui->driveRightLabel->setStyleSheet("QLabel { background-color : blue; color : white; }");
    }
    this->postData(lunabotics::Telecommand::TELEOPERATION);
}

void MainWindow::on_removePathButton_clicked()
{
    this->path->clear();
    this->redrawMap();
}

void MainWindow::on_allWheelButton_clicked()
{
    this->postData(lunabotics::Telecommand::ADJUST_WHEELS);
}

void MainWindow::on_forwardButton_clicked()
{
    ui->lfDrivingEdit->setText("1");
    ui->rfDrivingEdit->setText("1");
    ui->lrDrivingEdit->setText("1");
    ui->rrDrivingEdit->setText("1");
    this->postData(lunabotics::Telecommand::ADJUST_WHEELS);
}

void MainWindow::on_backwardButton_clicked()
{
    ui->lfDrivingEdit->setText("-1");
    ui->rfDrivingEdit->setText("-1");
    ui->lrDrivingEdit->setText("-1");
    ui->rrDrivingEdit->setText("-1");
    this->postData(lunabotics::Telecommand::ADJUST_WHEELS);
}

void MainWindow::on_stopButton_clicked()
{
    ui->lfDrivingEdit->setText("0");
    ui->rfDrivingEdit->setText("0");
    ui->lrDrivingEdit->setText("0");
    ui->rrDrivingEdit->setText("0");
    ui->lfSteeringEdit->setText("0");
    ui->rfSteeringEdit->setText("0");
    ui->lrSteeringEdit->setText("0");
    ui->rrSteeringEdit->setText("0");
    this->postData(lunabotics::Telecommand::ADJUST_WHEELS);
}
