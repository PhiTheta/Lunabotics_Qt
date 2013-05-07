#include "allwheelform.h"
#include "ui_allwheelform.h"
#include <QCloseEvent>
#include <QDebug>
#include <QtCore>
#include "Graphics.h"
#include "Common.h"

#define WHEEL_W 10
#define WHEEL_H 20
#define OFFSET  30
#define THICKNESS   20

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
    this->frontWheel = NULL;
    this->rearWheel = NULL;
    this->verticalICR = NULL;
    this->horizontalICR = NULL;
    this->baseLink = NULL;

    ui->setupUi(this);
    this->robotSketchScene = new QGraphicsScene(this);
    ui->graphicsView->setScene(this->robotSketchScene);

    this->ICR.setX(0);
    this->ICR.setY(0);

    this->createGrphicItems();
}

AllWheelForm::~AllWheelForm()
{
    qDebug() << "DELETING PANEL";
    delete ui;

    delete this->robotSketchScene;
    delete this->steeringMotors;
    delete this->drivingMotors;
    delete this->geometry;
}

void AllWheelForm::createGrphicItems()
{
    if (!this->graphicItemsCreated && this->geometry->jointPositionsAcquired) {
        qreal leftTopX = -this->geometry->leftFrontJoint.y()*100;
        qreal leftTopY = -this->geometry->leftFrontJoint.x()*100;
        qreal rightTopX = -this->geometry->rightFrontJoint.y()*100;
        qreal rightTopY = -this->geometry->rightFrontJoint.x()*100;
        qreal leftBottomX = -this->geometry->leftRearJoint.y()*100;
        qreal leftBottomY = -this->geometry->leftRearJoint.x()*100;
        qreal rightBottomX = -this->geometry->rightRearJoint.y()*100;
        qreal rightBottomY = -this->geometry->rightRearJoint.x()*100;

        this->baseLink = new QGraphicsRectItem(leftTopX, leftTopY, rightTopX-leftTopX, rightBottomY-rightTopY);
        this->baseLink->setBrush(BRUSH_CLEAR);
        this->baseLink->setPen(PEN_BLACK);
        this->robotSketchScene->addItem(this->baseLink);

        this->verticalICR = new QGraphicsRectItem(0, -10, 1, 20);
        this->verticalICR->setPen(PEN_RED);
        this->horizontalICR = new QGraphicsRectItem(-10, 0, 20, 1);
        this->horizontalICR->setPen(PEN_RED);
        this->robotSketchScene->addItem(this->horizontalICR);
        this->robotSketchScene->addItem(this->verticalICR);

        qreal offset = rightBottomY+OFFSET;
        this->robotSketchScene->addRect(leftTopX, leftBottomY+OFFSET, rightTopX-leftTopX, THICKNESS, PEN_BLACK, BRUSH_CLEAR);
        this->robotSketchScene->addEllipse(leftTopX-WHEEL_H/2, offset+THICKNESS/2, WHEEL_H, WHEEL_H, PEN_BLACK, BRUSH_CLEAR);
        this->robotSketchScene->addEllipse(rightBottomX-WHEEL_H/2, offset+THICKNESS/2, WHEEL_H, WHEEL_H, PEN_BLACK, BRUSH_CLEAR);


        this->leftFrontWheel = new QGraphicsRectItem(-WHEEL_W/2, -WHEEL_H/2, WHEEL_W, WHEEL_H);
        this->rightFrontWheel = new QGraphicsRectItem(-WHEEL_W/2, -WHEEL_H/2, WHEEL_W, WHEEL_H);
        this->leftRearWheel = new QGraphicsRectItem(-WHEEL_W/2, -WHEEL_H/2, WHEEL_W, WHEEL_H);
        this->rightRearWheel = new QGraphicsRectItem(-WHEEL_W/2, -WHEEL_H/2, WHEEL_W, WHEEL_H);

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


        this->frontWheel = new QGraphicsRectItem(-WHEEL_H/2+5, -WHEEL_H/2+5, WHEEL_H-10, WHEEL_H-10);
        this->rearWheel = new QGraphicsRectItem(-WHEEL_H/2+5, -WHEEL_H/2+5, WHEEL_H-10, WHEEL_H-10);
        this->frontWheel->setBrush(BRUSH_BLACK);
        this->frontWheel->setPen(PEN_RED);
        this->rearWheel->setBrush(BRUSH_BLACK);
        this->rearWheel->setPen(PEN_RED);
        this->robotSketchScene->addItem(this->frontWheel);
        this->robotSketchScene->addItem(this->rearWheel);

        this->frontWheel->setPos(rightBottomX, offset+THICKNESS);
        this->rearWheel->setPos(leftBottomX, offset+THICKNESS);

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
        this->frontWheel->rotate(this->drivingMotors->leftFront*180.0/M_PI);
        this->rearWheel->rotate(-this->drivingMotors->leftRear*180.0/M_PI);

        qreal x = this->ICR.x()*100;
        qreal y = this->ICR.y()*-100;

        this->verticalICR->setPos(x, y);
        this->horizontalICR->setPos(x, y);
    }
}

void AllWheelForm::on_allWheelButton_clicked()
{
    AllWheelState *steering = new AllWheelState(ui->lfSteeringEdit->text().toFloat(), ui->rfSteeringEdit->text().toFloat(), ui->lrSteeringEdit->text().toFloat(), ui->rrSteeringEdit->text().toFloat());
    AllWheelState *driving = new AllWheelState(ui->lfDrivingEdit->text().toFloat(), ui->rfDrivingEdit->text().toFloat(), ui->lrDrivingEdit->text().toFloat(), ui->rrDrivingEdit->text().toFloat());

    emit explicitControlSelected(steering, driving);

    delete steering;
    delete driving;
}

void AllWheelForm::on_forwardButton_clicked()
{
    emit predefinedControlSelected(lunabotics::AllWheelControl::DRIVE_FORWARD);
}

void AllWheelForm::on_backwardButton_clicked()
{
    emit predefinedControlSelected(lunabotics::AllWheelControl::DRIVE_BACKWARD);
}

void AllWheelForm::on_leftButton_clicked()
{
    emit predefinedControlSelected(lunabotics::AllWheelControl::CRAB_LEFT);
}

void AllWheelForm::on_rightButton_clicked()
{
    emit predefinedControlSelected(lunabotics::AllWheelControl::CRAB_RIGHT);
}

void AllWheelForm::on_turnLeftButton_clicked()
{
    emit predefinedControlSelected(lunabotics::AllWheelControl::TURN_CCW);
}

void AllWheelForm::on_turnRightButton_clicked()
{
    emit predefinedControlSelected(lunabotics::AllWheelControl::TURN_CW);
}

void AllWheelForm::closeEvent(QCloseEvent *event)
{
    emit closing();
    event->accept();
}

void AllWheelForm::allWheelStateUpdated(AllWheelState *steering, AllWheelState *driving)
{
    ui->dlfLabel->setText(QString("%1 Rad/s").arg(QString::number(round(driving->leftFront, 2), 'f', 2)));
    ui->drfLabel->setText(QString("%1 Rad/s").arg(QString::number(round(driving->rightFront, 2), 'f', 2)));
    ui->dlrLabel->setText(QString("%1 Rad/s").arg(QString::number(round(driving->leftRear, 2), 'f', 2)));
    ui->drrLabel->setText(QString("%1 Rad/s").arg(QString::number(round(driving->rightRear, 2), 'f', 2)));
    ui->slfLabel->setText(QString("%1 Rad").arg(QString::number(round(steering->leftFront, 2), 'f', 2)));
    ui->srfLabel->setText(QString("%1 Rad").arg(QString::number(round(steering->rightFront, 2), 'f', 2)));
    ui->slrLabel->setText(QString("%1 Rad").arg(QString::number(round(steering->leftRear, 2), 'f', 2)));
    ui->srrLabel->setText(QString("%1 Rad").arg(QString::number(round(steering->rightRear, 2), 'f', 2)));
    this->steeringMotors->setState(steering);
    this->drivingMotors->setState(driving);
    this->redrawSketch();
}

void AllWheelForm::ICRUpdated(QPointF ICR)
{
    this->ICR = ICR;
    ui->ICRLabel->setText(QString("%1,%2 m").arg(QString::number(ICR.x(), 'f', 2)).arg(QString::number(ICR.y(), 'f', 2)));
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

void AllWheelForm::on_stopButton_clicked()
{
    emit predefinedControlSelected(lunabotics::AllWheelControl::STOP);
}
