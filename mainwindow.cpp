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
#include "Telemetry.pb.h"
#include "Graphics.h"
#include "Common.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    ui->setupUi(this);

    //Setup data models

    this->map = new Map();
    this->robotState = new RobotState();
    this->nextWaypointIdx = -1;
    this->mapViewInfo = new MapViewMetaInfo(ui->mapView->width()-30, ui->mapView->height()-15);


    //Setup UI
    statusBar()->setVisible(false);


    this->allWheelPanel = NULL;
    this->followingPanel = NULL;

    this->mapScene = new QGraphicsScene(this);
    ui->mapView->setScene(this->mapScene);
    this->path = new QVector<QPointF>();
    this->pathGraphicsItem = NULL;



    QString text;

    QSettings settings("ivany4", "lunabotics");
    settings.beginGroup("control");
    ui->ackermannLinearSpeedEdit->setText(settings.value("ackermann.v", DEFAULT_LINEAR_SPEED_LIMIT).toString());
    ui->bezierSegmentsEdit->setText(settings.value("ackermann.bezier", DEFAULT_BEZIER_SEGMENTS).toString());
    settings.endGroup();

    ui->ackermannLinearSpeedEdit->setText(text.setNum(DEFAULT_LINEAR_SPEED_LIMIT));

    this->pathTableModel = new QStandardItemModel(0, 2, this); //0 Rows and 2 Columns
    this->pathTableModel->setHorizontalHeaderItem(0, new QStandardItem(QString("x")));
    this->pathTableModel->setHorizontalHeaderItem(1, new QStandardItem(QString("y")));
    ui->pathTableView->setModel(this->pathTableModel);

    this->telemetryTableModel = new QStandardItemModel(0, 2, this); //0 Rows and 2 Columns
    this->telemetryTableModel->setHorizontalHeaderItem(0, new QStandardItem(QString("Param")));
    this->telemetryTableModel->setHorizontalHeaderItem(1, new QStandardItem(QString("Value")));
    ui->telemetryTableView->setModel(this->telemetryTableModel);

    this->redrawMap();
    ui->driveForwardLabel->setStyleSheet("background-color : blue; color : white;");
    ui->driveLeftLabel->setStyleSheet("background-color : blue; color : white;");
    ui->driveRightLabel->setStyleSheet("background-color : blue; color : white;");
    ui->driveBackLabel->setStyleSheet("background-color : blue; color : white;");

    ui->autonomyLabelWidget->setVisible(false);
    ui->autonomyButton->setVisible(false);
    ui->ctrlDownPixmap->setVisible(false);
    ui->ctrlLeftPixmap->setVisible(false);
    ui->ctrlRightPixmap->setVisible(false);
    ui->ctrlUpPixmap->setVisible(false);

    //Controller representation is used instead
    ui->ctrlButtonsWidget->setVisible(false);



    //Setup network
    this->outgoingSocket = NULL;
    this->incomingServer = NULL;
    this->incomingSocket = NULL;
    this->connectRobot();


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
    delete this->map;
    delete this->path;
    delete this->allWheelPanel;
    delete this->followingPanel;
    delete this->pathTableModel;
    delete this->telemetryTableModel;
}


void MainWindow::toggleAutonomy()
{
    this->setAutonomy(!this->robotState->autonomous);
}

void MainWindow::setAutonomy(bool enabled)
{
    if (enabled != this->robotState->autonomous) {
        this->robotState->autonomous = enabled;
        if (enabled) {
            qDebug() << "Enabling autonomy";
        }
        else {
            qDebug() << "Disabling autonomy";
            if (!this->path->isEmpty()) {
                this->path->clear();
                this->updateMapPath();
            }
        }
        this->sendTelecommand(lunabotics::Telecommand::SET_AUTONOMY);
    }
}

void MainWindow::setAutonomyLabel(bool enabled)
{
    ui->autonomyLabelWidget->setVisible(enabled);
    ui->autonomyButton->setVisible(enabled);
}

void MainWindow::updateMapPath()
{
    qDebug() << "Updating path";
    if (this->map->isValid()) {
        this->pathTableModel->clear();
        if (this->pathGraphicsItem) {
            this->mapScene->removeItem(this->pathGraphicsItem);
            delete this->pathGraphicsItem, this->pathGraphicsItem = NULL;
        }
        if (!this->pathGraphicsItem) {
            this->pathGraphicsItem = new QGraphicsItemGroup();
            this->mapScene->addItem(this->pathGraphicsItem);
        }
        if (this->robotState->steeringMode == lunabotics::TURN_IN_SPOT) {
            for (int i = 1; i < this->path->size(); i++) {
                QPoint previousCoordinate = this->map->coordinateOf(this->path->at(i-1));
                QPoint coordinate = this->map->coordinateOf(this->path->at(i));

                if (i == 1) {
                    QGraphicsEllipseItem *ellipse = new QGraphicsEllipseItem(this->mapViewInfo->cellRectAt(previousCoordinate));
                    ellipse->setPen(PEN_BLUE);
                    ellipse->setBrush(BRUSH_CLEAR);
                    this->pathGraphicsItem->addToGroup(ellipse);
                }

                QGraphicsEllipseItem *ellipse = new QGraphicsEllipseItem(this->mapViewInfo->cellRectAt(coordinate));
                ellipse->setPen(PEN_BLUE);
                ellipse->setBrush(BRUSH_CLEAR);
                this->pathGraphicsItem->addToGroup(ellipse);
                QPointF pathPoint1 = this->mapViewInfo->cellCenterAt(previousCoordinate);
                QPointF pathPoint2 = this->mapViewInfo->cellCenterAt(coordinate);
                QGraphicsLineItem *line = new QGraphicsLineItem(pathPoint1.x(), pathPoint1.y(), pathPoint2.x(), pathPoint2.y());
                line->setPen(PEN_BLUE);
                this->pathGraphicsItem->addToGroup(line);

                //Set table rows
                QStandardItem *row = new QStandardItem(QString::number(previousCoordinate.x()));
                this->pathTableModel->setItem(i-1,0,row);
                row = new QStandardItem(QString::number(previousCoordinate.y()));
                this->pathTableModel->setItem(i-1,1,row);
            }
            if (!this->path->isEmpty()) {
                QPoint lastCoordinate = this->map->coordinateOf(this->path->last());
                QStandardItem *row = new QStandardItem(QString::number(lastCoordinate.x()));
                this->pathTableModel->setItem(this->path->size()-1,0,row);
                row = new QStandardItem(QString::number(lastCoordinate.y()));
                this->pathTableModel->setItem(this->path->size()-1,1,row);
            }
        }
        else {
            if (this->path->size() > 0) {
                QPointF p = this->mapViewInfo->pointFromWorld(this->path->at(0), this->map->resolution);
                QGraphicsEllipseItem *ellipse = new QGraphicsEllipseItem(p.x()-2, p.y()-2, 4, 4);
                ellipse->setPen(PEN_BLUE);
                ellipse->setBrush(BRUSH_CLEAR);
                this->pathGraphicsItem->addToGroup(ellipse);
            }
            for (int i = 1; i < this->path->size(); i++) {
                QPointF pathPoint1 = this->mapViewInfo->pointFromWorld(this->path->at(i-1), this->map->resolution);
                QPointF pathPoint2 = this->mapViewInfo->pointFromWorld(this->path->at(i), this->map->resolution);

                if (i == 0) {
                    QGraphicsEllipseItem *ellipse = new QGraphicsEllipseItem(pathPoint1.x()-2, pathPoint1.y()-2, 4, 4);
                    ellipse->setPen(PEN_BLUE);
                    ellipse->setBrush(BRUSH_CLEAR);
                    this->pathGraphicsItem->addToGroup(ellipse);
                }
                QGraphicsLineItem *line = new QGraphicsLineItem(pathPoint1.x(), pathPoint1.y(), pathPoint2.x(), pathPoint2.y());
                line->setPen(PEN_BLUE);
                this->pathGraphicsItem->addToGroup(line);
                QGraphicsEllipseItem *ellipse = new QGraphicsEllipseItem(pathPoint2.x()-2, pathPoint2.y()-2, 4, 4);
                ellipse->setPen(PEN_BLUE);
                ellipse->setBrush(BRUSH_CLEAR);
                this->pathGraphicsItem->addToGroup(ellipse);

                //Set table rows
                QStandardItem *row = new QStandardItem(QString::number(pathPoint1.x()));
                this->pathTableModel->setItem(i-1,0,row);
                row = new QStandardItem(QString::number(pathPoint1.y()));
                this->pathTableModel->setItem(i-1,1,row);
            }
            if (!this->path->isEmpty()) {
                QPointF lastCoordinate = this->mapViewInfo->pointFromWorld(this->path->last(), this->map->resolution);
                QStandardItem *row = new QStandardItem(QString::number(round(lastCoordinate.x(),1),'f',1));
                this->pathTableModel->setItem(this->path->size()-1,0,row);
                row = new QStandardItem(QString::number(round(lastCoordinate.y(),1),'f',1));
                this->pathTableModel->setItem(this->path->size()-1,1,row);
            }
        }
    }
    qDebug() << "Finished updating path";
}

void MainWindow::updateMapPoses()
{
    if (this->map->isValid()) {
        QPoint coordinate = this->map->coordinateOf(this->robotState->pose->position);
        ui->gridPositionLabel->setText(QString("%1,%2").arg(QString::number(coordinate.x())).arg(QString::number(coordinate.y())));

        QPointF robotCenter = this->mapViewInfo->pointFromWorld(this->robotState->pose->position, this->map->resolution);
        qreal robotRadius = 10;

        if (!this->robotCellItem) {
            this->robotCellItem = new QGraphicsRectItem();
            this->robotCellItem->setBrush(BRUSH_CLEAR);
            this->robotCellItem->setPen(PEN_BLUE);
            this->mapScene->addItem(this->robotCellItem);
        }
        this->robotCellItem->setRect(this->mapViewInfo->cellRectAt(coordinate));

        if (!this->robotPointerItem) {
            QGraphicsEllipseItem *ellipse = new QGraphicsEllipseItem(-robotRadius, -robotRadius, robotRadius*2, robotRadius*2);
            ellipse->setPen(PEN_RED);
            ellipse->setBrush(BRUSH_CLEAR);
            QGraphicsLineItem *line = new QGraphicsLineItem(0, 0, -robotRadius, 0);
            line->setPen(PEN_RED);
            this->robotPointerItem = new QGraphicsItemGroup();
            this->robotPointerItem->addToGroup(ellipse);
            this->robotPointerItem->addToGroup(line);
            this->mapScene->addItem(this->robotPointerItem);
        }
        this->robotPointerItem->setPos(robotCenter);
        this->robotPointerItem->setRotation(-this->robotState->pose->heading*180.0/M_PI);


        if (this->robotState->steeringMode == lunabotics::ACKERMANN && this->robotState->autonomous) {
            QPointF closestPoint = this->mapViewInfo->pointFromWorld(this->closestTrajectoryPoint, this->map->resolution);
            QPointF velocityPoint = this->mapViewInfo->pointFromWorld(this->velocityPoint, this->map->resolution);
            qDebug() << "CLosest point " << closestPoint.x() << ", " << closestPoint.y();
            if (!this->velocityVectorItem) {
                this->velocityVectorItem = new QGraphicsLineItem();
                this->velocityVectorItem->setPen(PEN_PURPLE);
            }
            if (!this->closestDistanceItem) {
                this->closestDistanceItem = new QGraphicsLineItem();
                this->closestDistanceItem->setPen(PEN_PURPLE);
            }

            this->velocityVectorItem->setLine(robotCenter.x(), robotCenter.y(), velocityPoint.x(), velocityPoint.y());
            this->closestDistanceItem->setLine(velocityPoint.x(), velocityPoint.y(), closestPoint.x(), closestPoint.y());
        }
        else {
            if (this->closestDistanceItem) {
                this->mapScene->removeItem(this->closestDistanceItem);
                delete this->closestDistanceItem, this->closestDistanceItem = NULL;
            }
            if (this->velocityVectorItem) {
                this->mapScene->removeItem(this->velocityVectorItem);
                delete this->velocityVectorItem, this->velocityVectorItem = NULL;
            }
        }

        this->robotCellItem->setZValue(1000);
        this->robotPointerItem->setZValue(1000);
    }
}

void MainWindow::redrawMap()
{
    this->removeAndDeleteAllMapItems();

    if (this->map->isValid()) {
        qDebug() << "Updating map";
        this->mapViewInfo->viewportWidth = ui->mapView->width();
        this->mapViewInfo->viewportHeight = ui->mapView->height();
        this->mapViewInfo->cellWidth = floor(this->mapViewInfo->viewportWidth/this->map->width);
        this->mapViewInfo->cellHeight = floor(this->mapViewInfo->viewportHeight/this->map->height);

        for (int x = 0; x < this->map->width; x++) {
            for (int y = 0; y < this->map->height; y++) {
                quint8 occupancy = this->map->at(x, y);
                QPoint point(x,y);

                //Set color according to occupancy value
                int val = 255-occupancy*2.55;
                QBrush varBrush(QColor(val, val, val));
                OccupancyGraphicsItem *rect = new OccupancyGraphicsItem(point, this->mapViewInfo->cellRectAt(x, y), 0);
                rect->setBrush(varBrush);
                rect->setPen(PEN_CLEAR);

                QObject::connect(rect, SIGNAL(clicked(QPoint)), this, SLOT(mapCell_clicked(QPoint)));
                QObject::connect(rect, SIGNAL(hovered(QPoint)), this, SLOT(mapCell_hovered(QPoint)));
                this->mapScene->addItem(rect);
            }
        }
    }
    else {
        qDebug() << "ERROR: Invalid map";
    }

    this->updateMapPath();
    this->updateMapPoses();

    qDebug() << "Finished updating map";
}

void MainWindow::removeAndDeleteAllMapItems()
{
    qDeleteAll(this->mapScene->items());
    this->mapScene->items().clear();
    this->robotCellItem = NULL;
    this->robotPointerItem = NULL;
    this->velocityVectorItem = NULL;
    this->closestDistanceItem = NULL;
    this->pathGraphicsItem = NULL;
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

    this->outgoingSocket->connectToHost(settings.value("ip", CONN_OUTGOING_ADDR).toString(), REMOTE_PORT);

    if (!this->incomingServer) {
        this->incomingServer = new QTcpServer(this);
        connect(this->incomingServer, SIGNAL(newConnection()), this, SLOT(acceptConnection()));
    }


    QHostAddress addr = QHostAddress::Any;
    if (!this->incomingServer->listen(addr, LOCAL_PORT)) {
        qDebug() << "Failed to start listening";
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

void MainWindow::acceptConnection()
{
    this->incomingSocket = this->incomingServer->nextPendingConnection();

    if (this->incomingSocket) {
        qDebug() << "New connection";
        connect(this->incomingSocket, SIGNAL(readyRead()), this, SLOT(receiveTelemetry()));
    }
}

void MainWindow::setRow(int &rowNumber, const QString &label, const QString &value)
{
    QStandardItem *item = new QStandardItem(label);
    item->setToolTip(label);
    this->telemetryTableModel->setItem(rowNumber, 0, item);
    item = new QStandardItem(value);
    item->setToolTip(value);
    this->telemetryTableModel->setItem(rowNumber++, 1, item);
}

void MainWindow::receiveTelemetry()
{
    qint64 bytesAvailable = this->incomingSocket->bytesAvailable();
    char *buffer = new char[bytesAvailable];
    this->incomingSocket->read(buffer, bytesAvailable);

    lunabotics::Telemetry tm;
    if (!tm.ParseFromArray(buffer, bytesAvailable) || !tm.IsInitialized()) {
        qDebug() << "Failed to parse telemetry object";
    }
    else {
        if (tm.has_world_data()) {
            qDebug() << "Receiving world data";
            const lunabotics::Telemetry::World world = tm.world_data();
            int width = world.width();
            int height = world.height();
            if (world.cell_size() != width*height) {
                qDebug() << "ERROR: World dimensions mismatch. Declared width and height don't correspond to number of cells in the message (" << world.cell_size() << "against " << width << "*" << height << ")";
            }
            else if (width == 0 || height == 0) {
                qDebug() << "ERROR: Receiving zero dimensions for map";
            }
            else {
                this->map->width = width;
                this->map->height = height;
                this->map->resolution = world.resolution();
                QVector<quint8> *cells = new QVector<quint8>();

                ui->mapResolutionLabel->setText(QString("1 cell = %1x%2m").arg(QString::number(this->map->resolution, 'f', 2)).arg(QString::number(this->map->resolution, 'f', 2)));

                qDebug() << "Receiving map";

                for (int i = 0; i < world.cell_size(); i++) {
                    cells->push_back(world.cell(i));
                }
                this->map->setCells(cells);
            }
            this->redrawMap();
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
            this->updateMapPath();
        }

        this->telemetryTableModel->clear();

        if (tm.has_state_data()) {
            const lunabotics::Telemetry::State state = tm.state_data();

            qint32 row = 0;

            this->robotState->pose->position.setX(state.position().x());
            this->robotState->pose->position.setY(state.position().y());
            this->robotState->pose->heading = state.heading();

            this->setRow(row, "pos.x", QString("%1 m").arg(QString::number(this->robotState->pose->position.x(), 'f', 2)));
            this->setRow(row, "pos.y", QString("%1 m").arg(QString::number(this->robotState->pose->position.y(), 'f', 2)));
            this->setRow(row, "rot", QString("%1 rad").arg(QString::number(this->robotState->pose->heading, 'f', 2)));
            this->setRow(row, "lin.vel", QString("%1 m/s").arg(QString::number(round(state.velocities().linear(), 2), 'f', 2)));
            this->setRow(row, "ang.vel", QString("%1 rad/s").arg(QString::number(round(state.velocities().angular(), 2), 'f', 2)));

            this->robotState->steeringMode = state.steering_mode();
            this->robotState->autonomous = state.autonomy_enabled();
            this->setAutonomyLabel(this->robotState->autonomous);


            QString str;
            switch (this->robotState->steeringMode) {
            case lunabotics::ACKERMANN: str = "Ackermann"; break;
            case lunabotics::TURN_IN_SPOT: str = "Turn in spot"; break;
            case lunabotics::CRAB: str = "Crab"; break;
            default: str = "Undefined"; break;
            }

            this->setRow(row, "steering.mode", str);

            if (state.has_next_waypoint_idx()) {
                this->nextWaypointIdx = state.next_waypoint_idx()-1;
            }

            if (state.has_icr()) {
                emit ICRUpdated(QPointF(state.icr().x(), state.icr().y()));
            }

            bool hasTrajectoryFollowingInfo = this->robotState->steeringMode == lunabotics::ACKERMANN && state.has_ackermann_telemetry();

            if (hasTrajectoryFollowingInfo) {

                const lunabotics::Telemetry::State::AckermannTelemetry params = state.ackermann_telemetry();


                this->closestTrajectoryPoint.setX(params.closest_trajectory_point().x());
                this->closestTrajectoryPoint.setY(params.closest_trajectory_point().y());
                this->velocityPoint.setX(params.velocity_vector_point().x());
                this->velocityPoint.setY(params.velocity_vector_point().y());
                this->transformedClosestTrajectoryPoint.setX(params.closest_trajectory_local_point().x());
                this->transformedClosestTrajectoryPoint.setY(params.closest_trajectory_local_point().y());
                this->transformedVelocityPoint.setX(params.velocity_vector_local_point().x());
                this->transformedVelocityPoint.setY(params.velocity_vector_local_point().y());

                emit updateLocalFrame(this->transformedVelocityPoint, this->transformedClosestTrajectoryPoint);

                this->setRow(row, "PID.err", QString("%1 m").arg(QString::number(params.pid_error(), 'f', 3)));
                this->setRow(row, "local.traj.pt.x", QString("%1 m").arg(QString::number(this->transformedClosestTrajectoryPoint.x(), 'f', 2)));
                this->setRow(row, "local.traj.pt.y", QString("%1 m").arg(QString::number(this->transformedClosestTrajectoryPoint.y(), 'f', 2)));
                this->setRow(row, "local.vec.pt.x", QString("%1 m").arg(QString::number(this->transformedVelocityPoint.x(), 'f', 2)));
                this->setRow(row, "local.vec.pt.y", QString("%1 m").arg(QString::number(this->transformedVelocityPoint.y(), 'f', 2)));
                this->setRow(row, "global.traj.pt.x", QString("%1 m").arg(QString::number(this->closestTrajectoryPoint.x(), 'f', 2)));
                this->setRow(row, "global.traj.pt.y", QString("%1 m").arg(QString::number(this->closestTrajectoryPoint.y(), 'f', 2)));
            }
            else {
                emit clearLocalFrame();
                if (this->robotState->steeringMode == lunabotics::TURN_IN_SPOT) { //state.has_point_turn_telemetry()) {
                    switch(state.point_turn_telemetry().state()) {
                    case lunabotics::Telemetry::DRIVING: str = "Driving"; break;
                    case lunabotics::Telemetry::TURNING: str = "Turning"; break;
                    case lunabotics::Telemetry::STOPPED: str = "Stopped"; break;
                    default: str = "Unknown"; break;
                    }
                    this->setRow(row, "spot-turn.state", str);
                }
            }
            this->updateMapPoses();
        }


        if (tm.has_laser_scan_data()) {
        }

        if (tm.has_all_wheel_state()) {
            const lunabotics::AllWheelState::Wheels steering = tm.all_wheel_state().steering();
            const lunabotics::AllWheelState::Wheels driving = tm.all_wheel_state().driving();
            AllWheelState *steeringMotors = new AllWheelState(steering.left_front(), steering.right_front(), steering.left_rear(), steering.right_rear());
            AllWheelState *drivingMotors = new AllWheelState(driving.left_front(), driving.right_front(), driving.left_rear(), driving.right_rear());
            emit allWheelStateUpdated(steeringMotors, drivingMotors);
            delete steeringMotors;
            delete drivingMotors;
        }

        if (tm.has_joints_data()) {
            if (!this->robotState->geometry->jointPositionsAcquired) {
                const lunabotics::Point p1 = tm.joints_data().left_front();
                this->robotState->geometry->leftFrontJoint.setX(p1.x());
                this->robotState->geometry->leftFrontJoint.setY(p1.y());
                const lunabotics::Point p2 = tm.joints_data().right_front();
                this->robotState->geometry->rightFrontJoint.setX(p2.x());
                this->robotState->geometry->rightFrontJoint.setY(p2.y());
                const lunabotics::Point p3 = tm.joints_data().left_rear();
                this->robotState->geometry->leftRearJoint.setX(p3.x());
                this->robotState->geometry->leftRearJoint.setY(p3.y());
                const lunabotics::Point p4 = tm.joints_data().right_rear();
                this->robotState->geometry->rightRearJoint.setX(p4.x());
                this->robotState->geometry->rightRearJoint.setY(p4.y());
                this->robotState->geometry->jointPositionsAcquired = true;
                emit jointPositionsUpdated(this->robotState->geometry);
            }
        }
    }

    delete buffer;

}
void MainWindow::sendTelecommand(lunabotics::Telecommand::Type contentType)
{
    this->socketMutex.lock();
    this->outgoingSocket->abort();
    QSettings settings("ivany4", "lunabotics");
    settings.beginGroup("connection");
    this->outgoingSocket->connectToHost(settings.value("ip", CONN_OUTGOING_ADDR).toString(), REMOTE_PORT);
    settings.endGroup();

    lunabotics::Telecommand tc;
    tc.set_type(contentType);

    switch (contentType) {
    case lunabotics::Telecommand::SET_AUTONOMY:
        tc.mutable_autonomy_data()->set_enabled(this->robotState->autonomous);
        break;
    case lunabotics::Telecommand::STEERING_MODE: {
        lunabotics::Telecommand::SteeringMode *steeringMode = tc.mutable_steering_mode_data();
        steeringMode->set_type(this->robotState->steeringMode);
        if (this->robotState->steeringMode == lunabotics::ACKERMANN) {
            lunabotics::Telecommand::SteeringMode::AckermannSteeringData *steeringData = steeringMode->mutable_ackermann_steering_data();
            steeringData->set_max_linear_velocity(ui->ackermannLinearSpeedEdit->text().toFloat());
            steeringData->set_bezier_curve_segments(ui->bezierSegmentsEdit->text().toInt());
        }
    }
        break;
    case lunabotics::Telecommand::TELEOPERATION: {
        lunabotics::Telecommand::Teleoperation *teleoperation = tc.mutable_teleoperation_data();
        teleoperation->set_forward(this->robotState->drivingMask & FORWARD);
        teleoperation->set_backward(this->robotState->drivingMask & BACKWARD);
        teleoperation->set_left(this->robotState->drivingMask & LEFT);
        teleoperation->set_right(this->robotState->drivingMask & RIGHT);
    }
        break;
    case lunabotics::Telecommand::DEFINE_ROUTE: {
        lunabotics::Telecommand::DefineRoute *route = tc.mutable_define_route_data();
        QPointF goal = this->map->positionOf(this->goal);
        route->mutable_goal()->set_x(goal.x());
        route->mutable_goal()->set_y(goal.y());
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
        tc.mutable_all_wheel_control_data()->set_all_wheel_type(this->robotState->allWheelControlType);
        switch (this->robotState->allWheelControlType) {
        case lunabotics::AllWheelControl::EXPLICIT: {
            lunabotics::AllWheelState::Wheels *steering = tc.mutable_all_wheel_control_data()->mutable_explicit_data()->mutable_steering();
            lunabotics::AllWheelState::Wheels *driving = tc.mutable_all_wheel_control_data()->mutable_explicit_data()->mutable_driving();

            steering->set_left_front(this->robotState->steeringMotors->leftFront);
            steering->set_right_front(this->robotState->steeringMotors->rightFront);
            steering->set_left_rear(this->robotState->steeringMotors->leftRear);
            steering->set_right_rear(this->robotState->steeringMotors->rightRear);

            driving->set_left_front(this->robotState->drivingMotors->leftFront);
            driving->set_right_front(this->robotState->drivingMotors->rightFront);
            driving->set_left_rear(this->robotState->drivingMotors->leftRear);
            driving->set_right_rear(this->robotState->drivingMotors->rightRear);
        }
            break;

        case lunabotics::AllWheelControl::PREDEFINED: {
            tc.mutable_all_wheel_control_data()->mutable_predefined_data()->set_command(this->robotState->predefinedControlType);
        }
            break;

        case lunabotics::AllWheelControl::ICR: {
            tc.mutable_all_wheel_control_data()->mutable_icr_data()->mutable_icr()->set_x(this->robotState->ICR.x());
            tc.mutable_all_wheel_control_data()->mutable_icr_data()->mutable_icr()->set_y(this->robotState->ICR.y());
            tc.mutable_all_wheel_control_data()->mutable_icr_data()->set_velocity(this->robotState->ICRVelocity);
        }
            break;
        }
    }
        break;

    default:
        break;
    }

    if (!tc.IsInitialized()) {
        qDebug() << "Telecommand not initialized";
        qDebug() << QString(tc.InitializationErrorString().c_str());
    }
    else {
        if (this->outgoingSocket->state() != QTcpSocket::UnconnectedState) {
            QByteArray byteArray(tc.SerializeAsString().c_str(), tc.ByteSize());
            qDebug() << "Sending " << byteArray.size() << " bytes";
            this->outgoingSocket->write(byteArray);
            this->outgoingSocket->waitForDisconnected(1);
        }
        else {
            qDebug() << "Socket is not connected!";
        }
    }

    this->socketMutex.unlock();
}


void MainWindow::mapCell_clicked(QPoint coordinate)
{
    this->goal = coordinate;
    this->setAutonomy(true);
    this->sendTelecommand(lunabotics::Telecommand::DEFINE_ROUTE);
}

void MainWindow::mapCell_hovered(QPoint coordinate)
{
    ui->cursorPositionLabel->setText(QString("%1,%2").arg(QString::number(coordinate.x())).arg(QString::number(coordinate.y())));
}

void MainWindow::on_actionPreferences_triggered()
{
    PreferenceDialog *preferenceDialog = new PreferenceDialog(this);
    preferenceDialog->setWindowModality(Qt::WindowModal);
    if (preferenceDialog->exec() == QDialog::Accepted) {
        this->connectRobot();
    }
}

void MainWindow::on_autonomyButton_clicked()
{
    this->toggleAutonomy();
}

void MainWindow::on_actionExit_triggered()
{
    this->disconnectRobot();
    QApplication::quit();
}


void MainWindow::on_useLateralButton_clicked()
{
    qDebug() << "Switching to lateral driving mode";
    this->robotState->steeringMode = lunabotics::CRAB;
    this->sendTelecommand(lunabotics::Telecommand::STEERING_MODE);
}

void MainWindow::on_useSpotButton_clicked()
{
    qDebug() << "Switching to spot driving mode";
    this->robotState->steeringMode = lunabotics::TURN_IN_SPOT;
    this->sendTelecommand(lunabotics::Telecommand::STEERING_MODE);
}

void MainWindow::on_useAckermannButton_clicked()
{
    qDebug() << "Switching to Ackermann driving mode";
    this->robotState->steeringMode = lunabotics::ACKERMANN;
    this->sendTelecommand(lunabotics::Telecommand::STEERING_MODE);
}

void MainWindow::on_refreshMapButton_clicked()
{
    this->sendTelecommand(lunabotics::Telecommand::REQUEST_MAP);
}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    this->setAutonomy(false);
    if (event->key() == Qt::Key_F5 || event->key() == Qt::Key_W) {
        this->robotState->drivingMask |= FORWARD;
        ui->driveForwardLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
        ui->ctrlUpPixmap->setVisible(true);
    }
    if (event->key() == Qt::Key_F6 || event->key() == Qt::Key_A) {
        this->robotState->drivingMask |= LEFT;
        ui->driveLeftLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
        ui->ctrlLeftPixmap->setVisible(true);
    }
    if (event->key() == Qt::Key_F7 || event->key() == Qt::Key_S) {
        this->robotState->drivingMask |= BACKWARD;
        ui->driveBackLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
        ui->ctrlDownPixmap->setVisible(true);
    }
    if (event->key() == Qt::Key_F8 || event->key() == Qt::Key_D) {
        this->robotState->drivingMask |= RIGHT;
        ui->driveRightLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
        ui->ctrlRightPixmap->setVisible(true);
    }
    this->sendTelecommand(lunabotics::Telecommand::TELEOPERATION);
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    this->setAutonomy(false);
    if (event->key() == Qt::Key_F5 || event->key() == Qt::Key_W) {
        this->robotState->drivingMask &= ~FORWARD;
        ui->driveForwardLabel->setStyleSheet("QLabel { background-color : blue; color : white; }");
        ui->ctrlUpPixmap->setVisible(false);
    }
    if (event->key() == Qt::Key_F6 || event->key() == Qt::Key_A) {
        this->robotState->drivingMask &= ~LEFT;
        ui->driveLeftLabel->setStyleSheet("QLabel { background-color : blue; color : white; }");
        ui->ctrlLeftPixmap->setVisible(false);
    }
    if (event->key() == Qt::Key_F7 || event->key() == Qt::Key_S) {
        this->robotState->drivingMask &= ~BACKWARD;
        ui->driveBackLabel->setStyleSheet("QLabel { background-color : blue; color : white; }");
        ui->ctrlDownPixmap->setVisible(false);
    }
    if (event->key() == Qt::Key_F8 || event->key() == Qt::Key_D) {
        this->robotState->drivingMask &= ~RIGHT;
        ui->driveRightLabel->setStyleSheet("QLabel { background-color : blue; color : white; }");
        ui->ctrlRightPixmap->setVisible(false);
    }
    this->sendTelecommand(lunabotics::Telecommand::TELEOPERATION);
}

void MainWindow::predefinedControlSelected(lunabotics::AllWheelControl::PredefinedControlType controlType)
{
    this->robotState->allWheelControlType = lunabotics::AllWheelControl::PREDEFINED;
    this->robotState->predefinedControlType = controlType;
    this->setAutonomy(false);
    this->sendTelecommand(lunabotics::Telecommand::ADJUST_WHEELS);
}

void MainWindow::explicitControlSelected(AllWheelState *steering, AllWheelState *driving)
{
    this->setAutonomy(false);
    this->robotState->steeringMotors->setState(steering);
    this->robotState->drivingMotors->setState(driving);
    this->robotState->allWheelControlType = lunabotics::AllWheelControl::EXPLICIT;
    this->sendTelecommand(lunabotics::Telecommand::ADJUST_WHEELS);
}

void MainWindow::ICRControlSelected(QPointF ICR, float velocity)
{
    this->setAutonomy(false);
    this->robotState->ICR = ICR;
    this->robotState->ICRVelocity = velocity;
    this->robotState->allWheelControlType = lunabotics::AllWheelControl::ICR;
    this->sendTelecommand(lunabotics::Telecommand::ADJUST_WHEELS);
}

void MainWindow::nullifyAllWheelPanel()
{
    qDebug() << "Close signal got";
    if (this->allWheelPanel) {
        delete this->allWheelPanel; this->allWheelPanel = NULL;
    }
}

void MainWindow::nullifyFollowingPanel()
{
    qDebug() << "Close signal got";
    if (this->followingPanel) {
        delete this->followingPanel; this->followingPanel = NULL;
    }
}

void MainWindow::on_actionAll_wheel_control_triggered()
{
    if (!this->allWheelPanel) {
        qDebug() << "Opening ALl Wheel Panel";
        this->allWheelPanel = new AllWheelForm();
        connect(this->allWheelPanel, SIGNAL(predefinedControlSelected(lunabotics::AllWheelControl::PredefinedControlType)), this, SLOT(predefinedControlSelected(lunabotics::AllWheelControl::PredefinedControlType)));
        connect(this->allWheelPanel, SIGNAL(explicitControlSelected(AllWheelState*,AllWheelState*)), this, SLOT(explicitControlSelected(AllWheelState*,AllWheelState*)));
        connect(this->allWheelPanel, SIGNAL(ICRControlSelected(QPointF,float)), this, SLOT(ICRControlSelected(QPointF,float)));
        connect(this->allWheelPanel, SIGNAL(closing()), this, SLOT(nullifyAllWheelPanel()));
        connect(this, SIGNAL(allWheelStateUpdated(AllWheelState*,AllWheelState*)), this->allWheelPanel, SLOT(allWheelStateUpdated(AllWheelState*,AllWheelState*)));
        connect(this, SIGNAL(ICRUpdated(QPointF)), this->allWheelPanel, SLOT(ICRUpdated(QPointF)));
        connect(this, SIGNAL(jointPositionsUpdated(RobotGeometry*)), this->allWheelPanel, SLOT(updateJoints(RobotGeometry*)));

        this->allWheelPanel->show();

        if (this->robotState->geometry->jointPositionsAcquired) {
            emit jointPositionsUpdated(this->robotState->geometry);
        }
    }
    else {
        qDebug() << "Bringing ALl Wheel Panel to front";
        this->allWheelPanel->setWindowState( (windowState() & ~Qt::WindowMinimized) | Qt::WindowActive);
        this->allWheelPanel->raise();  // for MacOS
        this->allWheelPanel->activateWindow(); // for Windows
    }
}

void MainWindow::on_actionTrajectory_following_triggered()
{

    if (!this->followingPanel) {
        qDebug() << "Opening Following Panel";
        this->followingPanel = new TrajectoryFollowingForm();

        connect(this->followingPanel, SIGNAL(closing()), this, SLOT(nullifyFollowingPanel()));
        connect(this->followingPanel, SIGNAL(sendPID()), this, SLOT(sendPID()));
        connect(this, SIGNAL(clearLocalFrame()), this->followingPanel, SLOT(clearLocalFrame()));
        connect(this, SIGNAL(updateLocalFrame(QPointF,QPointF)), this->followingPanel, SLOT(updateLocalFrame(QPointF,QPointF)));

        this->followingPanel->show();
    }
    else {
        qDebug() << "Bringing Folliowing Panel to front";
        this->followingPanel->setWindowState( (windowState() & ~Qt::WindowMinimized) | Qt::WindowActive);
        this->followingPanel->raise();  // for MacOS
        this->followingPanel->activateWindow(); // for Windows
    }
}

void MainWindow::sendPID()
{
    this->sendTelecommand(lunabotics::Telecommand::ADJUST_PID);
}
