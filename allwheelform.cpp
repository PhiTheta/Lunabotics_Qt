#include "allwheelform.h"
#include "ui_allwheelform.h"
#include <QCloseEvent>
#include <QDebug>

#define WHEEL_W 10
#define WHEEL_H 20
#define LEFT_WHEEL_ORIG   -50
#define TOP_WHEEL_ORIG    -60
#define OFFSET  30
#define THICKNESS   20

AllWheelForm::AllWheelForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::AllWheelForm)
{
    ui->setupUi(this);
    this->robotSketchScene = new QGraphicsScene(this);
    ui->graphicsView->setScene(this->robotSketchScene);

    QPen blackPen(Qt::black);
    QPen redPen(Qt::red);
    QBrush blackBrush(Qt::black);
    QBrush transparentBrush(Qt::transparent);
    this->robotSketchScene->addLine(0, -10, 0, 10, redPen);
    this->robotSketchScene->addLine(-10, 0, 10, 0, redPen);


    this->robotSketchScene->addRect(LEFT_WHEEL_ORIG, TOP_WHEEL_ORIG, (-LEFT_WHEEL_ORIG)*2, (-TOP_WHEEL_ORIG)*2, blackPen, transparentBrush);
    qreal offset = (-TOP_WHEEL_ORIG)*2+OFFSET;
    this->robotSketchScene->addRect(LEFT_WHEEL_ORIG, (-TOP_WHEEL_ORIG)*2+OFFSET, (-LEFT_WHEEL_ORIG)*2, THICKNESS, blackPen, transparentBrush);
    this->robotSketchScene->addEllipse(LEFT_WHEEL_ORIG-WHEEL_H/2, offset+THICKNESS/2, WHEEL_H, WHEEL_H, blackPen, transparentBrush);
    this->robotSketchScene->addEllipse(-LEFT_WHEEL_ORIG-WHEEL_H/2, offset+THICKNESS/2, WHEEL_H, WHEEL_H, blackPen, transparentBrush);


    this->leftFrontWheel = new QGraphicsRectItem(-WHEEL_W/2, -WHEEL_H/2, WHEEL_W, WHEEL_H);
    this->rightFrontWheel = new QGraphicsRectItem(-WHEEL_W/2, -WHEEL_H/2, WHEEL_W, WHEEL_H);
    this->leftRearWheel = new QGraphicsRectItem(-WHEEL_W/2, -WHEEL_H/2, WHEEL_W, WHEEL_H);
    this->rightRearWheel = new QGraphicsRectItem(-WHEEL_W/2, -WHEEL_H/2, WHEEL_W, WHEEL_H);

    this->leftFrontWheel->setBrush(blackBrush);
    this->leftFrontWheel->setPen(blackPen);
    this->rightFrontWheel->setBrush(blackBrush);
    this->rightFrontWheel->setPen(blackPen);
    this->leftRearWheel->setBrush(blackBrush);
    this->leftRearWheel->setPen(blackPen);
    this->rightRearWheel->setBrush(blackBrush);
    this->rightRearWheel->setPen(blackPen);

    this->robotSketchScene->addItem(this->leftFrontWheel);
    this->robotSketchScene->addItem(this->rightFrontWheel);
    this->robotSketchScene->addItem(this->leftRearWheel);
    this->robotSketchScene->addItem(this->rightRearWheel);

    this->leftFrontTransform.translate(LEFT_WHEEL_ORIG, TOP_WHEEL_ORIG);
    this->leftFrontWheel->setTransform(this->leftFrontTransform);
    this->rightFrontTransform.translate(-LEFT_WHEEL_ORIG, TOP_WHEEL_ORIG);
    this->rightFrontWheel->setTransform(this->rightFrontTransform);
    this->leftRearTransform.translate(LEFT_WHEEL_ORIG, -TOP_WHEEL_ORIG);
    this->leftRearWheel->setTransform(this->leftRearTransform);
    this->rightRearTransform.translate(-LEFT_WHEEL_ORIG, -TOP_WHEEL_ORIG);
    this->rightRearWheel->setTransform(this->rightRearTransform);


    this->frontWheel = new QGraphicsRectItem(-WHEEL_H/2+5, -WHEEL_H/2+5, WHEEL_H-10, WHEEL_H-10);
    this->rearWheel = new QGraphicsRectItem(-WHEEL_H/2+5, -WHEEL_H/2+5, WHEEL_H-10, WHEEL_H-10);
    this->frontWheel->setBrush(blackBrush);
    this->frontWheel->setPen(redPen);
    this->rearWheel->setBrush(blackBrush);
    this->rearWheel->setPen(redPen);
    this->robotSketchScene->addItem(this->frontWheel);
    this->robotSketchScene->addItem(this->rearWheel);

    this->frontTransform.translate(-LEFT_WHEEL_ORIG, offset+THICKNESS);
    this->frontWheel->setTransform(this->frontTransform);
    this->rearTransform.translate(LEFT_WHEEL_ORIG, offset+THICKNESS);
    this->rearWheel->setTransform(this->rearTransform);
}

AllWheelForm::~AllWheelForm()
{
    qDebug() << "DELETING PANEL";
    delete ui;

    delete this->robotSketchScene;
}

void AllWheelForm::redrawSketch()
{
    QTransform t;
    t = this->leftFrontTransform;
    t.rotateRadians(-slf);
    this->leftFrontWheel->setTransform(t);
    t = this->rightFrontTransform;
    t.rotateRadians(-srf);
    this->rightFrontWheel->setTransform(t);
    t = this->leftRearTransform;
    t.rotateRadians(-slr);
    this->leftRearWheel->setTransform(t);
    t = this->rightRearTransform;
    t.rotateRadians(-srr);
    this->rightRearWheel->setTransform(t);
    t = this->frontWheel->transform();
    t.rotateRadians(dlf);
    this->frontWheel->setTransform(t);
    t = this->rearWheel->transform();
    t.rotateRadians(-dlr);
    this->rearWheel->setTransform(t);
}

void AllWheelForm::on_allWheelButton_clicked()
{
    emit explicitControlSelected(ui->lfSteeringEdit->text().toFloat(),
                                 ui->rfSteeringEdit->text().toFloat(),
                                 ui->lrSteeringEdit->text().toFloat(),
                                 ui->rrSteeringEdit->text().toFloat(),
                                 ui->lfDrivingEdit->text().toFloat(),
                                 ui->rfDrivingEdit->text().toFloat(),
                                 ui->lrDrivingEdit->text().toFloat(),
                                 ui->rrDrivingEdit->text().toFloat());
}

void AllWheelForm::on_forwardButton_clicked()
{
    emit predefinedControlSelected(lunabotics::AllWheelControl::DRIVE_FORWARD);
}

void AllWheelForm::on_backwardButton_clicked()
{
    emit predefinedControlSelected(lunabotics::AllWheelControl::DRIVE_BACKWARD);
}

void AllWheelForm::on_stopButton_clicked()
{
    ui->lfDrivingEdit->setText("0");
    ui->rfDrivingEdit->setText("0");
    ui->lrDrivingEdit->setText("0");
    ui->rrDrivingEdit->setText("0");
    ui->lfSteeringEdit->setText("0");
    ui->rfSteeringEdit->setText("0");
    ui->lrSteeringEdit->setText("0");
    ui->rrSteeringEdit->setText("0");
    emit explicitControlSelected(0,0,0,0,0,0,0,0);
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

void AllWheelForm::allWheelStateUpdated(float slf, float srf, float slr, float srr, float dlf, float drf, float dlr, float drr)
{
    ui->dlfLabel->setText(QString("%1 Rad/s").arg(QString::number(dlf, 'f', 2)));
    ui->drfLabel->setText(QString("%1 Rad/s").arg(QString::number(drf, 'f', 2)));
    ui->dlrLabel->setText(QString("%1 Rad/s").arg(QString::number(dlr, 'f', 2)));
    ui->drrLabel->setText(QString("%1 Rad/s").arg(QString::number(drr, 'f', 2)));
    ui->slfLabel->setText(QString("%1 Rad").arg(QString::number(slf, 'f', 2)));
    ui->srfLabel->setText(QString("%1 Rad").arg(QString::number(srf, 'f', 2)));
    ui->slrLabel->setText(QString("%1 Rad").arg(QString::number(slr, 'f', 2)));
    ui->srrLabel->setText(QString("%1 Rad").arg(QString::number(srr, 'f', 2)));
    this->slf = slf;
    this->srf = srf;
    this->slr = slr;
    this->srr = srr;
    this->dlf = dlf;
    this->drf = drf;
    this->dlr = dlr;
    this->drr = drr;
    this->redrawSketch();
}
