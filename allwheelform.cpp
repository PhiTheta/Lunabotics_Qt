#include "allwheelform.h"
#include "ui_allwheelform.h"
#include <QCloseEvent>
#include <QDebug>
#include <QtCore>
#include "Graphics.h"
#include "Common.h"

#define SCALE   150
#define LINK_THICKNESS  0.03*SCALE

AllWheelForm::AllWheelForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::AllWheelForm)
{
    this->geometry = new RobotGeometry();
    this->steeringMotors = new AllWheelState();
    this->drivingMotors = new AllWheelState();
    this->graphicItemsCreated = false;

    this->leftFrontWheel = NULL;
    this->rightFrontWheel = NULL;
    this->leftRearWheel = NULL;
    this->rightRearWheel = NULL;
    this->closeICRItem = NULL;
    this->farICRItem = NULL;
    this->baseLink = NULL;
    this->leftFrontLink = NULL;
    this->leftRearLink = NULL;
    this->rightFrontLink = NULL;
    this->rightReartLink = NULL;

    ui->setupUi(this);
    this->robotSketchScene = new QGraphicsScene(this);
    ui->graphicsView->setScene(this->robotSketchScene);

    this->ICR.setX(0);
    this->ICR.setY(0);


    this->stateTableModel = new QStandardItemModel(0, 2, this); //0 Rows and 2 Columns
    this->stateTableModel->setHorizontalHeaderItem(0, new QStandardItem(QString("Param")));
    this->stateTableModel->setHorizontalHeaderItem(1, new QStandardItem(QString("Value")));
    ui->stateTableView->setModel(this->stateTableModel);

    this->createGrphicItems();
}

AllWheelForm::~AllWheelForm()
{
    qDebug() << "DELETING PANEL";
    qDeleteAll(this->robotSketchScene->items());
    this->robotSketchScene->items().clear();

    delete ui;

    delete this->robotSketchScene;
    delete this->steeringMotors;
    delete this->drivingMotors;
    delete this->geometry;
    delete this->stateTableModel;
}

void AllWheelForm::setRow(int &rowNumber, const QString &label, const QString &value)
{
    QStandardItem *item = new QStandardItem(label);
    item->setToolTip(label);
    this->stateTableModel->setItem(rowNumber, 0, item);
    item = new QStandardItem(value);
    item->setToolTip(value);
    this->stateTableModel->setItem(rowNumber++, 1, item);
}

void AllWheelForm::createGrphicItems()
{
    if (!this->graphicItemsCreated && this->geometry->jointPositionsAcquired) {
        qreal leftTopX = -this->geometry->leftFrontJoint.y()*SCALE;
        qreal leftTopY = -this->geometry->leftFrontJoint.x()*SCALE;
        qreal rightTopX = -this->geometry->rightFrontJoint.y()*SCALE;
        qreal rightTopY = -this->geometry->rightFrontJoint.x()*SCALE;
        qreal leftBottomX = -this->geometry->leftRearJoint.y()*SCALE;
        qreal leftBottomY = -this->geometry->leftRearJoint.x()*SCALE;
        qreal rightBottomX = -this->geometry->rightRearJoint.y()*SCALE;
        qreal rightBottomY = -this->geometry->rightRearJoint.x()*SCALE;
        qreal wheelOffset = this->geometry->wheelOffset*SCALE;
        qreal wheelRadius = this->geometry->wheelRadius*SCALE;
        qreal wheelWidth = this->geometry->wheelWidth*SCALE;

        this->baseLink = new QGraphicsRectItem(leftTopX, leftTopY, rightTopX-leftTopX, rightBottomY-rightTopY);
        this->baseLink->setBrush(BRUSH_CLEAR);
        this->baseLink->setPen(PEN_BLACK_BOLD);
        this->robotSketchScene->addItem(this->baseLink);

        this->closeICRItem = new QGraphicsItemGroup();
        QGraphicsLineItem *item = new QGraphicsLineItem(0, -10, 0, 10);
        item->setPen(PEN_RED_BOLD);
        this->closeICRItem->addToGroup(item);
        item = new QGraphicsLineItem(-10, 0, 10, 0);
        item->setPen(PEN_RED_BOLD);
        this->closeICRItem->addToGroup(item);
        this->robotSketchScene->addItem(this->closeICRItem);
        this->closeICRItem->setVisible(false);

        this->farICRItem = new QGraphicsItemGroup();
        item = new QGraphicsLineItem(0, 0, 10, 0);
        item->setPen(PEN_RED_BOLD);
        this->farICRItem->addToGroup(item);
        item = new QGraphicsLineItem(0, 0, 5, 5);
        item->setPen(PEN_RED_BOLD);
        this->farICRItem->addToGroup(item);
        item = new QGraphicsLineItem(0, 0, 5, -5);
        item->setPen(PEN_RED_BOLD);
        this->farICRItem->addToGroup(item);
        this->robotSketchScene->addItem(this->farICRItem);
        this->farICRItem->setVisible(false);

        this->leftFrontWheel = new QGraphicsRectItem(-wheelOffset-wheelWidth/2, -wheelRadius, wheelWidth, wheelRadius*2);
        this->rightFrontWheel = new QGraphicsRectItem(wheelOffset-wheelWidth/2, -wheelRadius, wheelWidth, wheelRadius*2);
        this->leftRearWheel = new QGraphicsRectItem(-wheelOffset-wheelWidth/2, -wheelRadius, wheelWidth, wheelRadius*2);
        this->rightRearWheel = new QGraphicsRectItem(wheelOffset-wheelWidth/2, -wheelRadius, wheelWidth, wheelRadius*2);

        this->leftFrontWheel->setBrush(BRUSH_BLACK);
        this->leftFrontWheel->setPen(PEN_BLACK);
        this->rightFrontWheel->setBrush(BRUSH_BLACK);
        this->rightFrontWheel->setPen(PEN_BLACK);
        this->leftRearWheel->setBrush(BRUSH_BLACK);
        this->leftRearWheel->setPen(PEN_BLACK);
        this->rightRearWheel->setBrush(BRUSH_BLACK);
        this->rightRearWheel->setPen(PEN_BLACK);

        this->robotSketchScene->addItem(this->leftFrontWheel);
        this->robotSketchScene->addItem(this->rightFrontWheel);
        this->robotSketchScene->addItem(this->leftRearWheel);
        this->robotSketchScene->addItem(this->rightRearWheel);

        this->leftFrontWheel->setPos(leftTopX, leftTopY);
        this->rightFrontWheel->setPos(rightTopX, rightTopY);
        this->leftRearWheel->setPos(leftBottomX, leftBottomY);
        this->rightRearWheel->setPos(rightBottomX, rightBottomY);

        this->leftFrontLink = new QGraphicsRectItem(-wheelOffset+wheelWidth/2, -LINK_THICKNESS/2, wheelOffset-wheelWidth/2, LINK_THICKNESS);
        this->leftRearLink = new QGraphicsRectItem(-wheelOffset+wheelWidth/2, -LINK_THICKNESS/2, wheelOffset-wheelWidth/2, LINK_THICKNESS);
        this->rightFrontLink = new QGraphicsRectItem(0, -LINK_THICKNESS/2, wheelOffset-wheelWidth/2, LINK_THICKNESS);
        this->rightReartLink = new QGraphicsRectItem(0, -LINK_THICKNESS/2, wheelOffset-wheelWidth/2, LINK_THICKNESS);

        this->leftFrontLink->setBrush(BRUSH_CLEAR);
        this->leftRearLink->setBrush(BRUSH_CLEAR);
        this->rightFrontLink->setBrush(BRUSH_CLEAR);
        this->rightReartLink->setBrush(BRUSH_CLEAR);
        this->leftFrontLink->setPen(PEN_GREEN_BOLD);
        this->leftRearLink->setPen(PEN_GREEN_BOLD);
        this->rightFrontLink->setPen(PEN_GREEN_BOLD);
        this->rightReartLink->setPen(PEN_GREEN_BOLD);

        this->robotSketchScene->addItem(this->leftFrontLink);
        this->robotSketchScene->addItem(this->rightFrontLink);
        this->robotSketchScene->addItem(this->leftRearLink);
        this->robotSketchScene->addItem(this->rightReartLink);

        this->leftFrontLink->setPos(leftTopX, leftTopY);
        this->rightFrontLink->setPos(rightTopX, rightTopY);
        this->leftRearLink->setPos(leftBottomX, leftBottomY);
        this->rightReartLink->setPos(rightBottomX, rightBottomY);

        this->graphicItemsCreated = true;
    }
}

void AllWheelForm::redrawSketch()
{
    if (!this->graphicItemsCreated) {
        this->createGrphicItems();
    }
    else {
        this->leftFrontWheel->setRotation(-this->steeringMotors->leftFront*180.0/M_PI);
        this->rightFrontWheel->setRotation(-this->steeringMotors->rightFront*180.0/M_PI);
        this->leftRearWheel->setRotation(-this->steeringMotors->leftRear*180.0/M_PI);
        this->rightRearWheel->setRotation(-this->steeringMotors->rightRear*180.0/M_PI);
        this->leftFrontLink->setRotation(-this->steeringMotors->leftFront*180.0/M_PI);
        this->rightFrontLink->setRotation(-this->steeringMotors->rightFront*180.0/M_PI);
        this->leftRearLink->setRotation(-this->steeringMotors->leftRear*180.0/M_PI);
        this->rightReartLink->setRotation(-this->steeringMotors->rightRear*180.0/M_PI);

        qreal y = this->ICR.x()*-SCALE;
        qreal x = this->ICR.y()*-SCALE;

        qreal w = ui->graphicsView->width();

        if (x > w/2) {
            this->closeICRItem->setVisible(false);
            this->farICRItem->setVisible(true);
            this->farICRItem->setRotation(180);
            this->farICRItem->setPos(w/2-5, 0);
        }
        else if (x < -w/2) {
            this->closeICRItem->setVisible(false);
            this->farICRItem->setVisible(true);
            this->farICRItem->setRotation(0);
            this->farICRItem->setPos(-w/2+5, 0);
        }
        else {
            this->closeICRItem->setVisible(true);
            this->farICRItem->setVisible(false);
            this->closeICRItem->setPos(x, y);
        }
    }
}

void AllWheelForm::on_forwardButton_clicked()
{
    emit predefinedControlSelected(lunabotics::proto::AllWheelControl::DRIVE_FORWARD);
}

void AllWheelForm::on_backwardButton_clicked()
{
    emit predefinedControlSelected(lunabotics::proto::AllWheelControl::DRIVE_BACKWARD);
}

void AllWheelForm::on_leftButton_clicked()
{
    emit predefinedControlSelected(lunabotics::proto::AllWheelControl::CRAB_LEFT);
}

void AllWheelForm::on_rightButton_clicked()
{
    emit predefinedControlSelected(lunabotics::proto::AllWheelControl::CRAB_RIGHT);
}

void AllWheelForm::on_turnLeftButton_clicked()
{
    emit predefinedControlSelected(lunabotics::proto::AllWheelControl::TURN_CCW);
}

void AllWheelForm::on_turnRightButton_clicked()
{
    emit predefinedControlSelected(lunabotics::proto::AllWheelControl::TURN_CW);
}

void AllWheelForm::on_stopButton_clicked()
{
    emit predefinedControlSelected(lunabotics::proto::AllWheelControl::STOP);
}

void AllWheelForm::closeEvent(QCloseEvent *event)
{
    emit closing();
    event->accept();
}

void AllWheelForm::allWheelStateUpdated(AllWheelState *steering, AllWheelState *driving)
{
    this->steeringMotors->setState(steering);
    this->drivingMotors->setState(driving);
    this->redrawSketch();
    this->updateTable();
}

void AllWheelForm::updateTable()
{
    this->stateTableModel->clear();
    int row = 0;
    this->setRow(row, "driving.left.front", QString("%1 Rad/s").arg(QString::number(round(this->drivingMotors->leftFront, 2), 'f', 2)));
    this->setRow(row, "driving.right.front", QString("%1 Rad/s").arg(QString::number(round(this->drivingMotors->rightFront, 2), 'f', 2)));
    this->setRow(row, "driving.left.rear", QString("%1 Rad/s").arg(QString::number(round(this->drivingMotors->leftRear, 2), 'f', 2)));
    this->setRow(row, "driving.right.rear", QString("%1 Rad/s").arg(QString::number(round(this->drivingMotors->rightRear, 2), 'f', 2)));
    this->setRow(row, "steering.left.front", QString("%1 Rad").arg(QString::number(round(this->steeringMotors->leftFront, 2), 'f', 2)));
    this->setRow(row, "steering.right.front", QString("%1 Rad").arg(QString::number(round(this->steeringMotors->rightFront, 2), 'f', 2)));
    this->setRow(row, "steering.left.rear", QString("%1 Rad").arg(QString::number(round(this->steeringMotors->leftRear, 2), 'f', 2)));
    this->setRow(row, "steering.right.rear", QString("%1 Rad").arg(QString::number(round(this->steeringMotors->rightRear, 2), 'f', 2)));
    this->setRow(row, "ICR.x", QString("%1 m").arg(QString::number(this->ICR.x(), 'f', 2)));
    this->setRow(row, "ICR.y", QString("%1 m").arg(QString::number(this->ICR.y(), 'f', 2)));
}

void AllWheelForm::ICRUpdated(QPointF ICR)
{
    this->ICR = ICR;
    this->updateTable();
    this->redrawSketch();
}

void AllWheelForm::on_sendICRButton_clicked()
{
    emit ICRControlSelected(QPointF(ui->ICRXEdit->text().toFloat(), ui->ICRYEdit->text().toFloat()), ui->velocityEdit->text().toFloat());
}

void AllWheelForm::updateJoints(RobotGeometry *geometry)
{
    this->geometry->setGeometry(geometry);
    this->redrawSketch();
}

void AllWheelForm::on_resetButton_clicked()
{
    ui->lfDrivingEdit->setText("0");
    ui->rfDrivingEdit->setText("0");
    ui->lrDrivingEdit->setText("0");
    ui->rrDrivingEdit->setText("0");
    ui->lfSteeringEdit->setText("0");
    ui->rfSteeringEdit->setText("0");
    ui->lrSteeringEdit->setText("0");
    ui->rrSteeringEdit->setText("0");
    AllWheelState *empty = new AllWheelState();
    emit explicitControlSelected(empty, empty);
    delete empty;
}

void AllWheelForm::on_sendButton_clicked()
{
    AllWheelState *steering = new AllWheelState(ui->lfSteeringEdit->text().toFloat(), ui->rfSteeringEdit->text().toFloat(), ui->lrSteeringEdit->text().toFloat(), ui->rrSteeringEdit->text().toFloat());
    AllWheelState *driving = new AllWheelState(ui->lfDrivingEdit->text().toFloat(), ui->rfDrivingEdit->text().toFloat(), ui->lrDrivingEdit->text().toFloat(), ui->rrDrivingEdit->text().toFloat());

    emit explicitControlSelected(steering, driving);

    delete steering;
    delete driving;
}
