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
    this->multiWaypoints = false;
    this->waypoints = new QVector<QPoint>();
    this->currentChunk = this->chunksTotal = 0;

    this->hasAckermannData = false;
    this->robotGeometryDrawn = false;

    //Setup UI
    statusBar()->setVisible(false);


    this->allWheelPanel = NULL;
    this->analysisPanel = NULL;
    this->followingPanel = NULL;

    this->mapScene = new QGraphicsScene(this);
    ui->mapView->setScene(this->mapScene);
    this->path = new QVector<QPointF>();
    this->pathGraphicsItem = NULL;

    this->multiWaypointsItem = NULL;
    ui->multiWaypointsButtonWidget->setVisible(false);
    ui->multiWaypointsButtonWidget->setEnabled(false);


    QString text;

    QSettings settings("ivany4", "lunabotics");
    settings.beginGroup("control");
    ui->linearSpeedEdit->setText(settings.value(SETTINGS_VELOCITY, DEFAULT_LINEAR_SPEED_LIMIT).toString());
    ui->bezierSegmentsEdit->setText(settings.value(SETTINGS_BEZIER, DEFAULT_BEZIER_SEGMENTS).toString());
    ui->angleTolerance->setText(settings.value(SETTINGS_ANGLE_ACC, DEFAULT_ANGLE_ACCURACY).toString());
    ui->distanceTolerance->setText(settings.value(SETTINGS_DIST_ACC, DEFAULT_DISTANCE_ACCURACY).toString());
    settings.endGroup();

    this->pathTableModel = new QStandardItemModel(0, 2, this); //0 Rows and 2 Columns
    ui->pathTableView->setModel(this->pathTableModel);
    this->resetPathModel();

    this->telemetryTableModel = new QStandardItemModel(0, 2, this); //0 Rows and 2 Columns
    ui->telemetryTableView->setModel(this->telemetryTableModel);
    this->resetTelemetryModel();

    this->redrawMap();

    ui->autonomyLabelWidget->setVisible(false);
    ui->autonomyButton->setVisible(false);
    ui->ctrlDownPixmap->setVisible(false);
    ui->ctrlLeftPixmap->setVisible(false);
    ui->ctrlRightPixmap->setVisible(false);
    ui->ctrlUpPixmap->setVisible(false);

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
    settings.setValue(SETTINGS_VELOCITY, ui->linearSpeedEdit->text());
    settings.setValue(SETTINGS_BEZIER, ui->bezierSegmentsEdit->text());
    settings.setValue(SETTINGS_ANGLE_ACC, ui->angleTolerance->text());
    settings.setValue(SETTINGS_DIST_ACC, ui->distanceTolerance->text());
    settings.endGroup();

    delete ui;
    this->disconnectRobot();

    delete this->waypoints;
    delete this->incomingServer;
    delete this->outgoingSocket;
    delete this->mapScene;
    delete this->map;
    delete this->path;
    delete this->allWheelPanel;
    delete this->followingPanel;
    delete this->analysisPanel;
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
        if (!enabled && !this->path->isEmpty()) {
            this->path->clear();
            this->updateMapPath();
        }
        this->sendTelecommand(lunabotics::proto::Telecommand::SET_AUTONOMY);
    }
}

void MainWindow::setAutonomyLabel(bool enabled)
{
    ui->autonomyLabelWidget->setVisible(enabled);
    ui->autonomyButton->setVisible(enabled);
}

void MainWindow::updateMapPath()
{
    if (this->map->isValid()) {
        this->removeMultiWaypointsPrint();
        this->resetPathModel();
        this->minICRRadius = -1;
        this->trajectoryCurves.clear();
        emit updateCurves(this->trajectoryCurves);
        if (this->pathGraphicsItem) {
            this->mapScene->removeItem(this->pathGraphicsItem);
            delete this->pathGraphicsItem, this->pathGraphicsItem = NULL;
        }
        if (!this->pathGraphicsItem) {
            this->pathGraphicsItem = new QGraphicsItemGroup();
            this->mapScene->addItem(this->pathGraphicsItem);
        }
        if (this->robotState->steeringMode == lunabotics::proto::POINT_TURN) {
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

        if (!this->robotPointerItem && this->robotState->geometry->jointPositionsAcquired) {
            QGraphicsEllipseItem *ellipse = new QGraphicsEllipseItem(-robotRadius, -robotRadius, robotRadius*2, robotRadius*2);
            ellipse->setPen(PEN_RED);
            ellipse->setBrush(BRUSH_CLEAR);
            QGraphicsLineItem *line = new QGraphicsLineItem(0, 0, -robotRadius, 0);
            line->setPen(PEN_RED);
            this->robotPointerItem = new QGraphicsItemGroup();
            this->robotPointerItem->addToGroup(ellipse);
            this->robotPointerItem->addToGroup(line);

            QPointF leftFront(this->robotState->geometry->leftFrontJoint.x()/this->map->resolution*this->mapViewInfo->cellEdge, this->robotState->geometry->leftFrontJoint.y()/this->map->resolution*this->mapViewInfo->cellEdge);
            QPointF rightFront(this->robotState->geometry->rightFrontJoint.x()/this->map->resolution*this->mapViewInfo->cellEdge, this->robotState->geometry->rightFrontJoint.y()/this->map->resolution*this->mapViewInfo->cellEdge);
            QPointF leftRear(this->robotState->geometry->leftRearJoint.x()/this->map->resolution*this->mapViewInfo->cellEdge, this->robotState->geometry->leftRearJoint.y()/this->map->resolution*this->mapViewInfo->cellEdge);
            QPointF rightRear(this->robotState->geometry->rightRearJoint.x()/this->map->resolution*this->mapViewInfo->cellEdge, this->robotState->geometry->rightRearJoint.y()/this->map->resolution*this->mapViewInfo->cellEdge);

            qDebug() << "TEST " << leftFront.x() << " " << leftFront.y();

            line = new QGraphicsLineItem(-leftFront.x(), -leftFront.y(), -rightFront.x(), -rightFront.y());
            line->setPen(PEN_GREEN);
            this->robotPointerItem->addToGroup(line);
            line = new QGraphicsLineItem(-rightFront.x(), -rightFront.y(), -rightRear.x(), -rightRear.y());
            line->setPen(PEN_GREEN);
            this->robotPointerItem->addToGroup(line);
            line = new QGraphicsLineItem(-rightRear.x(), -rightRear.y(), -leftRear.x(), -leftRear.y());
            line->setPen(PEN_GREEN);
            this->robotPointerItem->addToGroup(line);
            line = new QGraphicsLineItem(-leftRear.x(), -leftRear.y(), -leftFront.x(), -leftFront.y());
            line->setPen(PEN_GREEN);
            this->robotPointerItem->addToGroup(line);

            this->mapScene->addItem(this->robotPointerItem);
        }

        if (this->robotPointerItem) {

            this->robotPointerItem->setPos(robotCenter);
            this->robotPointerItem->setRotation(-this->robotState->pose->heading*180.0/M_PI);


            if (this->robotState->steeringMode == lunabotics::proto::ACKERMANN && this->robotState->autonomous
                    && this->hasAckermannData) {
                QPointF closestPoint = this->mapViewInfo->pointFromWorld(this->feedbackPathPoint, this->map->resolution);
                QPointF feedbackPoint = this->mapViewInfo->pointFromWorld(this->feedbackPoint, this->map->resolution);
                if (!this->velocityVectorItem) {
                    this->velocityVectorItem = new QGraphicsLineItem();
                    this->velocityVectorItem->setPen(PEN_PURPLE);
                    this->mapScene->addItem(this->velocityVectorItem);
                }
                if (!this->closestDistanceItem) {
                    this->closestDistanceItem = new QGraphicsLineItem();
                    this->closestDistanceItem->setPen(PEN_PURPLE);
                    this->mapScene->addItem(this->closestDistanceItem);
                }
                if (isvalid(feedbackPoint)) {
                    this->velocityVectorItem->setLine(robotCenter.x(), robotCenter.y(), feedbackPoint.x(), feedbackPoint.y());
                    if (isvalid(closestPoint)) {
                        this->closestDistanceItem->setLine(feedbackPoint.x(), feedbackPoint.y(), closestPoint.x(), closestPoint.y());
                    }
                }
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
}

void MainWindow::redrawMap()
{
    this->removeAndDeleteAllMapItems();

    if (this->isMapValid()) {
        if (map->isValid()) {
            this->mapViewInfo->viewportWidth = ui->mapView->width();
            this->mapViewInfo->viewportHeight = ui->mapView->height();
            if (this->map->width > this->map->height) {
                this->mapViewInfo->cellEdge = floor(std::min(this->mapViewInfo->viewportWidth,this->mapViewInfo->viewportHeight)/this->map->width);
            }
            else {
                this->mapViewInfo->cellEdge = floor(std::min(this->mapViewInfo->viewportWidth,this->mapViewInfo->viewportHeight)/this->map->height);
            }

            for (int x = 0; x < this->map->width; x++) {
                for (int y = 0; y < this->map->height; y++) {
                    quint8 occupancy = this->map->at(x, y);
                    QPoint point(x,y);

                    OccupancyGraphicsItem *rect = new OccupancyGraphicsItem(point, this->mapViewInfo->cellRectAt(x, y), occupancy, 0);

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
    }
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
    this->multiWaypointsItem = NULL;
}


void MainWindow::connectRobot()
{
    this->disconnectRobot();

    QSettings settings("ivany4", "lunabotics");
    settings.beginGroup("connection");

    if (!this->outgoingSocket) {
        this->outgoingSocket = new QTcpSocket(this);
    }

    this->outgoingSocket->connectToHost(settings.value(SETTINGS_IP, CONN_OUTGOING_ADDR).toString(), settings.value(SETTINGS_REMOTE_PORT, CONN_REMOTE_PORT).toInt());

    if (!this->incomingServer) {
        this->incomingServer = new QTcpServer(this);
        connect(this->incomingServer, SIGNAL(newConnection()), this, SLOT(acceptConnection()));
    }


    QHostAddress addr = QHostAddress::Any;
    if (!this->incomingServer->listen(addr, settings.value(SETTINGS_LOCAL_PORT, CONN_LOCAL_PORT).toInt())) {
        qDebug() << "Failed to start listening";
    }

    settings.endGroup();

    //Acknowledge about ip and port and ask for a map right away
    this->sendTelecommand(lunabotics::proto::Telecommand::REQUEST_MAP);
}

void MainWindow::disconnectRobot()
{
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

    lunabotics::proto::Telemetry tm;
    if (!tm.ParseFromArray(buffer, bytesAvailable) || !tm.IsInitialized()) {
        qDebug() << "Failed to parse telemetry object";
    }
    else {
        if (tm.has_world_data()) {
            const lunabotics::proto::Telemetry::World world = tm.world_data();
            int width = world.width();
            int height = world.height();
            this->chunksTotal = world.total_chunks();
            this->currentChunk = world.chunk_number();
            if (width == 0 || height == 0) {
                qDebug() << "ERROR: Receiving zero dimensions for map";
            }
            else {
                this->map->width = width;
                this->map->height = height;
                this->map->resolution = world.resolution();
                QVector<quint8> *cells = new QVector<quint8>();

                ui->mapResolutionLabel->setText(QString("1 cell = %1x%2m").arg(QString::number(this->map->resolution, 'f', 2)).arg(QString::number(this->map->resolution, 'f', 2)));

                for (int i = 0; i < world.cell_size(); i++) {
                    cells->push_back(world.cell(i));
                }

                qDebug() << "Receiving map chunk " << this->currentChunk+1 << " of " << this->chunksTotal << " (" << cells->size() << " cells)";

                if (this->currentChunk == 0) {
                    this->map->setCells(cells);
                }
                else {
                    this->map->appendCells(cells);
                }

                if (this->currentChunk < this->chunksTotal-1) {
                    ui->refreshMapButton->setEnabled(false);
                    ui->refreshMapButton->setText(QString("Downloading %1%...").arg(QString::number(100*this->currentChunk/((float)this->chunksTotal), 'f', 0)));
                }
                else {
                    ui->refreshMapButton->setEnabled(true);
                    ui->refreshMapButton->setText("Refresh map");
                    this->currentChunk = this->chunksTotal = 0;
                }
            }
            this->redrawMap();
        }

        if (tm.has_path_data()) {
            this->path->clear();

            const lunabotics::proto::Telemetry::Path path = tm.path_data();

            for (int i = 0; i < path.position_size(); i++) {
                const lunabotics::proto::Point position = path.position(i);
                QPointF point;
                point.setX(position.x());
                point.setY(position.y());
                this->path->push_back(point);
            }
            this->updateMapPath();

            QVector<lunabotics::proto::Telemetry::Path::Curve> curves;
            for (int i = 0; i < path.curves_size(); i++) {
                const lunabotics::proto::Telemetry::Path::Curve curve = path.curves(i);
                curves.push_back(curve);
            }
            this->trajectoryCurves = curves;
            emit updateCurves(curves);
        }


        this->resetTelemetryModel();

        if (tm.has_state_data()) {
            const lunabotics::proto::Telemetry::State state = tm.state_data();

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
            case lunabotics::proto::ACKERMANN: str = "Ackermann"; break;
            case lunabotics::proto::POINT_TURN: str = "Point-turn"; break;
            case lunabotics::proto::AUTO: str = "Automatic"; break;
            default: str = "Undefined"; break;
            }

            this->setRow(row, "steering.mode", str);

            if (state.has_next_waypoint_idx()) {
                this->nextWaypointIdx = state.next_waypoint_idx()-1;
                this->setRow(row, "traj.next_waypoint", QString("%1").arg(QString::number(this->nextWaypointIdx)));
            }

            if (state.has_segment_idx()) {
                this->segmentIdx = state.segment_idx();
                this->setRow(row, "traj.segment", QString("%1").arg(QString::number(this->segmentIdx)));
            }

            if (state.has_icr()) {
                if (this->robotState->steeringMode == lunabotics::proto::ACKERMANN || this->robotState->steeringMode == lunabotics::proto::AUTO) {
                    this->setRow(row, "ICR.x", QString("%1 m").arg(QString::number(state.icr().x(), 'f', 2)));
                    this->setRow(row, "ICR.y", QString("%1 m").arg(QString::number(state.icr().y(), 'f', 2)));
                }
                emit ICRUpdated(QPointF(state.icr().x(), state.icr().y()));
            }

            if (state.has_min_icr_offset()) {
                this->minICRRadius = state.min_icr_offset();
                emit updateRadius(state.min_icr_offset());
                this->setRow(row, "min ICR", QString("%1 m").arg(QString::number(state.min_icr_offset(), 'f', 2)));
            }

            this->hasAckermannData = state.has_ackermann_telemetry();

            if (state.has_ackermann_telemetry()) {

                const lunabotics::proto::Telemetry::State::AckermannTelemetry params = state.ackermann_telemetry();

                this->feedbackPathPoint.setX(params.feedback_path_point().x());
                this->feedbackPathPoint.setY(params.feedback_path_point().y());
                this->feedbackPoint.setX(params.feedback_point().x());
                this->feedbackPoint.setY(params.feedback_point().y());
                this->feedbackPathPointLocal.setX(params.feedback_path_point_local().x());
                this->feedbackPathPointLocal.setY(params.feedback_path_point_local().y());
                this->feedbackPointLocal.setX(params.feedback_point_local().x());
                this->feedbackPointLocal.setY(params.feedback_point_local().y());

                QVector<QPointF> feedforwardPoints;
                for (int i = 0; i < params.feedforward_points_local_size(); i++) {
                    const lunabotics::proto::Point point = params.feedforward_points_local(i);
                    feedforwardPoints.push_back(QPointF(point.x(),point.y()));
                }

                QPointF feedforwardCenter(params.feedforward_center().x(), params.feedforward_center().y());

                emit updateLocalFrame(this->feedbackPointLocal, this->feedbackPathPointLocal, feedforwardPoints, feedforwardCenter);

                this->setRow(row, "PID.err", QString("%1 m").arg(QString::number(params.feedback_error(), 'f', 3)));
                this->setRow(row, "FF.prediction", QString("%1 rad").arg(QString::number(params.feedforward_prediction(), 'f', 3)));
                this->setRow(row, "FF.radius", QString("%1 m").arg(QString::number(params.feedforward_curve_radius(), 'f', 3)));
                this->setRow(row, "head.err", QString("%1 rad").arg(QString::number(params.heading_error(), 'f', 3)));

                if (isvalid(this->feedbackPathPointLocal)) {
                    this->setRow(row, "traj.ff.c.x", QString("%1 m").arg(QString::number(feedforwardCenter.x(), 'f', 2)));
                    this->setRow(row, "traj.ff.c.y", QString("%1 m").arg(QString::number(feedforwardCenter.y(), 'f', 2)));
                }
                if (isvalid(this->feedbackPathPointLocal)) {
                    this->setRow(row, "local.traj.pt.x", QString("%1 m").arg(QString::number(this->feedbackPathPointLocal.x(), 'f', 2)));
                    this->setRow(row, "local.traj.pt.y", QString("%1 m").arg(QString::number(this->feedbackPathPointLocal.y(), 'f', 2)));
                }
                if (isvalid(this->feedbackPointLocal)) {
                    this->setRow(row, "local.vec.pt.x", QString("%1 m").arg(QString::number(this->feedbackPointLocal.x(), 'f', 2)));
                    this->setRow(row, "local.vec.pt.y", QString("%1 m").arg(QString::number(this->feedbackPointLocal.y(), 'f', 2)));
                }
                if (isvalid(this->feedbackPathPoint)) {
                    this->setRow(row, "global.traj.pt.x", QString("%1 m").arg(QString::number(this->feedbackPathPoint.x(), 'f', 2)));
                    this->setRow(row, "global.traj.pt.y", QString("%1 m").arg(QString::number(this->feedbackPathPoint.y(), 'f', 2)));
                }
                if (isvalid(this->feedbackPoint)) {
                    this->setRow(row, "global.vec.pt.x", QString("%1 m").arg(QString::number(this->feedbackPoint.x(), 'f', 2)));
                    this->setRow(row, "global.vec.pt.y", QString("%1 m").arg(QString::number(this->feedbackPoint.y(), 'f', 2)));
                }
            }
            else {
                emit clearLocalFrame();
                if (this->robotState->steeringMode == lunabotics::proto::POINT_TURN) { //state.has_point_turn_telemetry()) {
                    switch(state.point_turn_telemetry().state()) {
                    case lunabotics::proto::Telemetry::DRIVING: str = "Driving"; break;
                    case lunabotics::proto::Telemetry::TURNING: str = "Turning"; break;
                    case lunabotics::proto::Telemetry::STOPPED: str = "Stopped"; break;
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
            const lunabotics::proto::AllWheelState::Wheels steering = tm.all_wheel_state().steering();
            const lunabotics::proto::AllWheelState::Wheels driving = tm.all_wheel_state().driving();
            AllWheelState *steeringMotors = new AllWheelState(steering.left_front(), steering.right_front(), steering.left_rear(), steering.right_rear());
            AllWheelState *drivingMotors = new AllWheelState(driving.left_front(), driving.right_front(), driving.left_rear(), driving.right_rear());
            emit allWheelStateUpdated(steeringMotors, drivingMotors);
            delete steeringMotors;
            delete drivingMotors;
        }

        if (tm.has_geometry_data()) {
            if (!this->robotState->geometry->jointPositionsAcquired) {
                const lunabotics::proto::Telemetry::Geometry geometry = tm.geometry_data();
                const lunabotics::proto::Point p1 = geometry.left_front_joint();
                this->robotState->geometry->leftFrontJoint.setX(p1.x());
                this->robotState->geometry->leftFrontJoint.setY(p1.y());
                const lunabotics::proto::Point p2 = geometry.right_front_joint();
                this->robotState->geometry->rightFrontJoint.setX(p2.x());
                this->robotState->geometry->rightFrontJoint.setY(p2.y());
                const lunabotics::proto::Point p3 = geometry.left_rear_joint();
                this->robotState->geometry->leftRearJoint.setX(p3.x());
                this->robotState->geometry->leftRearJoint.setY(p3.y());
                const lunabotics::proto::Point p4 = geometry.right_rear_joint();
                this->robotState->geometry->rightRearJoint.setX(p4.x());
                this->robotState->geometry->rightRearJoint.setY(p4.y());
                this->robotState->geometry->wheelOffset = geometry.wheel_offset();
                this->robotState->geometry->wheelRadius = geometry.wheel_radius();
                this->robotState->geometry->wheelWidth = geometry.wheel_width();
                this->robotState->geometry->jointPositionsAcquired = true;
                emit jointPositionsUpdated(this->robotState->geometry);
            }
        }
    }

    delete buffer;

}
void MainWindow::sendTelecommand(lunabotics::proto::Telecommand::Type contentType)
{
    this->socketMutex.lock();
    this->outgoingSocket->abort();
    QSettings settings("ivany4", "lunabotics");
    settings.beginGroup("connection");
    this->outgoingSocket->connectToHost(settings.value(SETTINGS_IP, CONN_OUTGOING_ADDR).toString(), settings.value(SETTINGS_REMOTE_PORT, CONN_REMOTE_PORT).toInt());
    settings.endGroup();

    lunabotics::proto::Telecommand tc;
    tc.set_type(contentType);
    tc.set_reply_port(settings.value(SETTINGS_LOCAL_PORT, CONN_LOCAL_PORT).toString().toStdString());

    switch (contentType) {
    case lunabotics::proto::Telecommand::SET_AUTONOMY:
        tc.mutable_autonomy_data()->set_enabled(this->robotState->autonomous);
        break;
    case lunabotics::proto::Telecommand::STEERING_MODE: {
        lunabotics::proto::Telecommand::SteeringMode *steeringMode = tc.mutable_steering_mode_data();
        steeringMode->set_type(this->robotState->steeringMode);
        steeringMode->set_heading_accuracy(ui->angleTolerance->text().toFloat());
        steeringMode->set_position_accuracy(ui->distanceTolerance->text().toFloat());
        steeringMode->set_max_linear_velocity(ui->linearSpeedEdit->text().toFloat());
        steeringMode->set_bezier_curve_segments(ui->bezierSegmentsEdit->text().toInt());
    }
        break;
    case lunabotics::proto::Telecommand::TELEOPERATION: {
        lunabotics::proto::Telecommand::Teleoperation *teleoperation = tc.mutable_teleoperation_data();
        teleoperation->set_forward(this->robotState->drivingMask & FORWARD);
        teleoperation->set_backward(this->robotState->drivingMask & BACKWARD);
        teleoperation->set_left(this->robotState->drivingMask & LEFT);
        teleoperation->set_right(this->robotState->drivingMask & RIGHT);
    }
        break;
    case lunabotics::proto::Telecommand::DEFINE_ROUTE: {
        lunabotics::proto::Telecommand::DefineRoute *route = tc.mutable_define_route_data();
        for (int i = 0; i < this->waypoints->size(); i++) {
            QPoint p = this->waypoints->at(i);
            qreal p_x = p.x();
            qreal p_y = p.y();
            lunabotics::proto::Point *waypoint = route->add_waypoints();
            waypoint->set_x(p_x);
            waypoint->set_y(p_y);
        }
    }
        break;

    case lunabotics::proto::Telecommand::REQUEST_MAP:
        //Nothing to include
        break;

    case lunabotics::proto::Telecommand::ADJUST_PID: {
        lunabotics::proto::Telecommand::AdjustPID *pid = tc.mutable_adjust_pid_data();
        settings.beginGroup("pid");
        pid->set_p(settings.value(SETTING_PID_P, PID_KP).toFloat());
        pid->set_i(settings.value(SETTING_PID_I, PID_KI).toFloat());
        pid->set_d(settings.value(SETTING_PID_D, PID_KD).toFloat());
        pid->set_feedback_min_offset(settings.value(SETTING_FEEDBACK_MIN_OFFSET, FEEDBACK_MIN_OFFSET).toFloat());
        pid->set_feedback_multiplier(settings.value(SETTING_FEEDBACK_MULTIPLIER, FEEDBACK_MULTIPLIER).toFloat());
        pid->set_feedforward_fraction(settings.value(SETTING_FEEDFORWARD_FRACTION, FEEDFORWARD_FRACTION).toFloat());
        pid->set_feedforward_min_offset(settings.value(SETTING_FEEDFORWARD_MIN_OFFSET, FEEDFORWARD_MIN_OFFSET).toFloat());
        settings.endGroup();
    }
        break;

    case lunabotics::proto::Telecommand::ADJUST_WHEELS: {
        tc.mutable_all_wheel_control_data()->set_all_wheel_type(this->robotState->allWheelControlType);
        switch (this->robotState->allWheelControlType) {
        case lunabotics::proto::AllWheelControl::EXPLICIT: {
            lunabotics::proto::AllWheelState::Wheels *steering = tc.mutable_all_wheel_control_data()->mutable_explicit_data()->mutable_steering();
            lunabotics::proto::AllWheelState::Wheels *driving = tc.mutable_all_wheel_control_data()->mutable_explicit_data()->mutable_driving();

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

        case lunabotics::proto::AllWheelControl::PREDEFINED: {
            tc.mutable_all_wheel_control_data()->mutable_predefined_data()->set_command(this->robotState->predefinedControlType);
        }
            break;

        case lunabotics::proto::AllWheelControl::ICR: {
            tc.mutable_all_wheel_control_data()->mutable_icr_data()->mutable_icr()->set_x(this->robotState->ICR.x());
            tc.mutable_all_wheel_control_data()->mutable_icr_data()->mutable_icr()->set_y(this->robotState->ICR.y());
            tc.mutable_all_wheel_control_data()->mutable_icr_data()->set_velocity(this->robotState->ICRVelocity);
        }
            break;

        case lunabotics::proto::AllWheelControl::CRAB: {
            tc.mutable_all_wheel_control_data()->mutable_crab_data()->set_heading(this->crabHeading);
            tc.mutable_all_wheel_control_data()->mutable_crab_data()->set_velocity(this->crabVelocity);
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

void MainWindow::removeMultiWaypointsPrint()
{
    this->mapScene->removeItem(this->multiWaypointsItem);
    delete this->multiWaypointsItem;
    this->multiWaypointsItem = NULL;
    this->waypoints->clear();
}


void MainWindow::mapCell_clicked(QPoint coordinate)
{
    if (this->multiWaypoints) {
        if (!this->multiWaypointsItem) {
            this->multiWaypointsItem = new QGraphicsItemGroup();
            this->mapScene->addItem(this->multiWaypointsItem);
        }
        this->waypoints->push_back(coordinate);
        QGraphicsEllipseItem *ellipse = new QGraphicsEllipseItem(this->mapViewInfo->cellRectAt(coordinate));
        ellipse->setPen(PEN_BLUE);
        ellipse->setBrush(BRUSH_CLEAR);
        this->multiWaypointsItem->addToGroup(ellipse);
        QPoint previousCoordinate;
        if (this->waypoints->size() > 1) {
            previousCoordinate = this->waypoints->at(this->waypoints->size()-2);
        }
        else {
            previousCoordinate = this->map->coordinateOf(this->robotState->pose->position);
        }
        QPointF pathPoint1 = this->mapViewInfo->cellCenterAt(previousCoordinate);
        QPointF pathPoint2 = this->mapViewInfo->cellCenterAt(coordinate);
        QGraphicsLineItem *line = new QGraphicsLineItem(pathPoint1.x(), pathPoint1.y(), pathPoint2.x(), pathPoint2.y());
        line->setPen(PEN_BLUE);
        this->multiWaypointsItem->addToGroup(line);
        ui->multiWaypointsButtonWidget->setEnabled(!this->waypoints->empty());
    }
    else {
        ui->multiWaypointsButtonWidget->setEnabled(false);
        this->removeMultiWaypointsPrint();
        this->waypoints->push_back(coordinate);
        this->setAutonomy(true);
        this->sendTelecommand(lunabotics::proto::Telecommand::DEFINE_ROUTE);
    }
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

void MainWindow::on_useSpotButton_clicked()
{
    this->robotState->steeringMode = lunabotics::proto::POINT_TURN;
    this->sendTelecommand(lunabotics::proto::Telecommand::STEERING_MODE);
}

void MainWindow::on_useAckermannButton_clicked()
{
    this->robotState->steeringMode = lunabotics::proto::ACKERMANN;
    this->sendTelecommand(lunabotics::proto::Telecommand::STEERING_MODE);
}

void MainWindow::on_refreshMapButton_clicked()
{
    this->sendTelecommand(lunabotics::proto::Telecommand::REQUEST_MAP);
}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    this->setAutonomy(false);
    if (event->key() == Qt::Key_F5 || event->key() == Qt::Key_W) {
        this->robotState->drivingMask |= FORWARD;
        ui->ctrlUpPixmap->setVisible(true);
    }
    if (event->key() == Qt::Key_F6 || event->key() == Qt::Key_A) {
        this->robotState->drivingMask |= LEFT;
        ui->ctrlLeftPixmap->setVisible(true);
    }
    if (event->key() == Qt::Key_F7 || event->key() == Qt::Key_S) {
        this->robotState->drivingMask |= BACKWARD;
        ui->ctrlDownPixmap->setVisible(true);
    }
    if (event->key() == Qt::Key_F8 || event->key() == Qt::Key_D) {
        this->robotState->drivingMask |= RIGHT;
        ui->ctrlRightPixmap->setVisible(true);
    }
    this->sendTelecommand(lunabotics::proto::Telecommand::TELEOPERATION);
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    this->setAutonomy(false);
    if (event->key() == Qt::Key_F5 || event->key() == Qt::Key_W) {
        this->robotState->drivingMask &= ~FORWARD;
        ui->ctrlUpPixmap->setVisible(false);
    }
    if (event->key() == Qt::Key_F6 || event->key() == Qt::Key_A) {
        this->robotState->drivingMask &= ~LEFT;
        ui->ctrlLeftPixmap->setVisible(false);
    }
    if (event->key() == Qt::Key_F7 || event->key() == Qt::Key_S) {
        this->robotState->drivingMask &= ~BACKWARD;
        ui->ctrlDownPixmap->setVisible(false);
    }
    if (event->key() == Qt::Key_F8 || event->key() == Qt::Key_D) {
        this->robotState->drivingMask &= ~RIGHT;
        ui->ctrlRightPixmap->setVisible(false);
    }
    this->sendTelecommand(lunabotics::proto::Telecommand::TELEOPERATION);
}

void MainWindow::predefinedControlSelected(lunabotics::proto::AllWheelControl::PredefinedControlType controlType)
{
    this->robotState->allWheelControlType = lunabotics::proto::AllWheelControl::PREDEFINED;
    this->robotState->predefinedControlType = controlType;
    this->setAutonomy(false);
    this->sendTelecommand(lunabotics::proto::Telecommand::ADJUST_WHEELS);
}

void MainWindow::explicitControlSelected(AllWheelState *steering, AllWheelState *driving)
{
    this->setAutonomy(false);
    this->robotState->steeringMotors->setState(steering);
    this->robotState->drivingMotors->setState(driving);
    this->robotState->allWheelControlType = lunabotics::proto::AllWheelControl::EXPLICIT;
    this->sendTelecommand(lunabotics::proto::Telecommand::ADJUST_WHEELS);
}

void MainWindow::ICRControlSelected(QPointF ICR, float velocity)
{
    this->setAutonomy(false);
    this->robotState->ICR = ICR;
    this->robotState->ICRVelocity = velocity;
    this->robotState->allWheelControlType = lunabotics::proto::AllWheelControl::ICR;
    this->sendTelecommand(lunabotics::proto::Telecommand::ADJUST_WHEELS);
}

void MainWindow::nullifyAllWheelPanel()
{
    if (this->allWheelPanel) {
        delete this->allWheelPanel; this->allWheelPanel = NULL;
    }
}

void MainWindow::nullifyFollowingPanel()
{
    if (this->followingPanel) {
        delete this->followingPanel; this->followingPanel = NULL;
    }
}

void MainWindow::nullifyAnalysisPanel()
{
    if (this->analysisPanel) {
        delete this->analysisPanel; this->analysisPanel = NULL;
    }
}

void MainWindow::on_actionAll_wheel_control_triggered()
{
    if (!this->allWheelPanel) {
        this->allWheelPanel = new AllWheelForm();
        connect(this->allWheelPanel, SIGNAL(predefinedControlSelected(lunabotics::proto::AllWheelControl::PredefinedControlType)), this, SLOT(predefinedControlSelected(lunabotics::proto::AllWheelControl::PredefinedControlType)));
        connect(this->allWheelPanel, SIGNAL(explicitControlSelected(AllWheelState*,AllWheelState*)), this, SLOT(explicitControlSelected(AllWheelState*,AllWheelState*)));
        connect(this->allWheelPanel, SIGNAL(ICRControlSelected(QPointF,float)), this, SLOT(ICRControlSelected(QPointF,float)));
        connect(this->allWheelPanel, SIGNAL(crabControlSelected(qreal,qreal)), this, SLOT(crabControlSelected(qreal,qreal)));
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
        this->allWheelPanel->setWindowState( (windowState() & ~Qt::WindowMinimized) | Qt::WindowActive);
        this->allWheelPanel->raise();  // for MacOS
        this->allWheelPanel->activateWindow(); // for Windows
    }
}

void MainWindow::on_actionTrajectory_following_triggered()
{

    if (!this->followingPanel) {
        this->followingPanel = new TrajectoryFollowingForm();

        connect(this->followingPanel, SIGNAL(closing()), this, SLOT(nullifyFollowingPanel()));
        connect(this->followingPanel, SIGNAL(sendPID()), this, SLOT(sendPID()));
        connect(this, SIGNAL(clearLocalFrame()), this->followingPanel, SLOT(clearLocalFrame()));
        connect(this, SIGNAL(updateLocalFrame(QPointF,QPointF,QVector<QPointF>,QPointF)), this->followingPanel, SLOT(updateLocalFrame(QPointF,QPointF,QVector<QPointF>,QPointF)));

        this->followingPanel->show();
    }
    else {
        this->followingPanel->setWindowState( (windowState() & ~Qt::WindowMinimized) | Qt::WindowActive);
        this->followingPanel->raise();  // for MacOS
        this->followingPanel->activateWindow(); // for Windows
    }
}

void MainWindow::sendPID()
{
    this->sendTelecommand(lunabotics::proto::Telecommand::ADJUST_PID);
}

void MainWindow::on_actionTrajectory_analysis_triggered()
{

    if (!this->analysisPanel) {
        this->analysisPanel = new AnalysisForm();

        connect(this->analysisPanel, SIGNAL(closing()), this, SLOT(nullifyAnalysisPanel()));
        connect(this, SIGNAL(updateCurves(QVector<lunabotics::proto::Telemetry::Path::Curve>)), this->analysisPanel, SLOT(updateCurves(QVector<lunabotics::proto::Telemetry::Path::Curve>)));
        connect(this, SIGNAL(updateRadius(float)), this->analysisPanel, SLOT(updateRadius(float)));

        this->analysisPanel->show();
        emit updateCurves(this->trajectoryCurves);
        emit updateRadius(this->minICRRadius);
    }
    else {
        this->analysisPanel->setWindowState( (windowState() & ~Qt::WindowMinimized) | Qt::WindowActive);
        this->analysisPanel->raise();  // for MacOS
        this->analysisPanel->activateWindow(); // for Windows
    }
}

void MainWindow::resetTelemetryModel()
{
    this->telemetryTableModel->clear();
    this->telemetryTableModel->setHorizontalHeaderItem(0, new QStandardItem(QString("Param")));
    this->telemetryTableModel->setHorizontalHeaderItem(1, new QStandardItem(QString("Value")));
}

void MainWindow::resetPathModel()
{
    this->pathTableModel->clear();
    this->pathTableModel->setHorizontalHeaderItem(0, new QStandardItem(QString("x")));
    this->pathTableModel->setHorizontalHeaderItem(1, new QStandardItem(QString("y")));
}

void MainWindow::on_multiWaypointsButton_clicked()
{
    this->multiWaypoints = !this->multiWaypoints;
    ui->multiWaypointsButtonWidget->setVisible(this->multiWaypoints);
    ui->multiWaypointsButton->setText(this->multiWaypoints ? "Single selection" : "Multi selection");
}

void MainWindow::on_waypointsResetButton_clicked()
{
    this->removeMultiWaypointsPrint();
}

void MainWindow::on_waypointsSendButton_clicked()
{
    this->sendTelecommand(lunabotics::proto::Telecommand::DEFINE_ROUTE);
}

void MainWindow::crabControlSelected(qreal head, qreal vel)
{
    this->crabHeading = head;
    this->crabVelocity = vel;
    this->robotState->allWheelControlType = lunabotics::proto::AllWheelControl::CRAB;
    this->sendTelecommand(lunabotics::proto::Telecommand::ADJUST_WHEELS);
}

void MainWindow::on_useAutoButton_clicked()
{
    this->robotState->steeringMode = lunabotics::proto::AUTO;
    this->sendTelecommand(lunabotics::proto::Telecommand::STEERING_MODE);
}

bool MainWindow::isMapValid()
{
    return (this->currentChunk >= this->chunksTotal || this->chunksTotal == 0) && this->map->isValid();
}
