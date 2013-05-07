#include "trajectoryfollowingform.h"
#include "ui_trajectoryfollowingform.h"
#include "Graphics.h"
#include "constants.h"
#include <QDebug>
#include <QCloseEvent>
#include <QSettings>
#include <QtCore>

TrajectoryFollowingForm::TrajectoryFollowingForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TrajectoryFollowingForm)
{
    ui->setupUi(this);

    this->localTrajectoryEllipseItem = NULL;
    this->localVectorEllipseItem = NULL;
    this->localVectorLineItem = NULL;
    this->localDistanceLineItem = NULL;


    this->localFrameInfo = new MapViewMetaInfo(ui->graphicsView->width(), ui->graphicsView->height());

    QSettings settings("ivany4", "lunabotics");
    settings.beginGroup("pid");
    ui->KpEdit->setText(settings.value("p", PID_KP).toString());
    ui->KiEdit->setText(settings.value("i", PID_KI).toString());
    ui->KdEdit->setText(settings.value("d", PID_KD).toString());
    ui->offsetEdit->setText(settings.value("offset", PID_OFFSET).toString());
    ui->velocityEdit->setText(settings.value("v", PID_VEL_M).toString());
    settings.endGroup();

    this->localFrameScene = new QGraphicsScene(this);
    ui->graphicsView->setScene(this->localFrameScene);
}

TrajectoryFollowingForm::~TrajectoryFollowingForm()
{
    qDebug() << "DELETING PANEL";
    this->clearLocalFrame();

    delete ui;
    delete this->localFrameScene;
    delete this->localFrameInfo;
}

void TrajectoryFollowingForm::closeEvent(QCloseEvent *event)
{
    this->saveSettings();
    emit closing();
    event->accept();
}

void TrajectoryFollowingForm::saveSettings()
{
    QSettings settings( "ivany4", "lunabotics");
    settings.beginGroup("pid");
    settings.setValue("p", ui->KpEdit->text());
    settings.setValue("i", ui->KiEdit->text());
    settings.setValue("d", ui->KdEdit->text());
    settings.setValue("offset", ui->offsetEdit->text());
    settings.setValue("v", ui->velocityEdit->text());
    settings.endGroup();
}

void TrajectoryFollowingForm::clearLocalFrame()
{
    qDeleteAll(this->localFrameScene->items());
    this->localFrameScene->items().clear();
    this->localDistanceLineItem = NULL;
    this->localVectorLineItem = NULL;
    this->localTrajectoryEllipseItem = NULL;
    this->localVectorEllipseItem = NULL;
}

void TrajectoryFollowingForm::updateLocalFrame(QPointF velocityPoint, QPointF trajectoryPoint)
{
    QPointF origin(ui->graphicsView->width()/2, ui->graphicsView->height()/2);
    if (!this->localDistanceLineItem) {
        this->localDistanceLineItem = new QGraphicsLineItem();
        this->localDistanceLineItem->setPen(PEN_BLUE);
        this->localFrameScene->addItem(this->localDistanceLineItem);
    }
    if (!this->localVectorLineItem) {
        this->localVectorLineItem = new QGraphicsLineItem();
        this->localVectorLineItem->setPen(PEN_RED);
        this->localFrameScene->addItem(this->localVectorLineItem);
    }
    if (!this->localTrajectoryEllipseItem) {
        this->localTrajectoryEllipseItem = new QGraphicsEllipseItem(-2, -2, 4, 4);
        this->localTrajectoryEllipseItem->setPen(PEN_BLUE);
        this->localTrajectoryEllipseItem->setBrush(BRUSH_BLUE);
        this->localFrameScene->addItem(this->localTrajectoryEllipseItem);
    }
    if (!this->localVectorEllipseItem) {
        this->localVectorEllipseItem = new QGraphicsEllipseItem(-1, -1, 2, 2);
        this->localVectorEllipseItem->setPen(PEN_RED);
        this->localVectorEllipseItem->setBrush(BRUSH_RED);
        this->localFrameScene->addItem(this->localVectorEllipseItem);
    }

    qreal y = origin.x()+velocityPoint.x()*100;
    qreal x = origin.y()+velocityPoint.y()*100;
    qreal y1 = origin.x()+trajectoryPoint.x()*100;
    qreal x1 = origin.y()+trajectoryPoint.y()*100;

    this->localTrajectoryEllipseItem->setPos(x1, y1);
    this->localVectorEllipseItem->setPos(x, y);
    this->localDistanceLineItem->setLine(origin.x(), origin.y(), x, y);
    this->localVectorLineItem->setLine(x, y, x1, y1);
    this->localTrajectoryEllipseItem->setRotation(-90);
    this->localVectorEllipseItem->setRotation(-90);
    this->localDistanceLineItem->setRotation(-90);
    this->localVectorLineItem->setRotation(-90);
    //this->localFrameScene->addEllipse(origin.x()-5, origin.y()-5, 10.0, 10.0, PEN_BLACK, BRUSH_GREEN);
}

void TrajectoryFollowingForm::on_resetButton_clicked()
{
    ui->KpEdit->setText(PID_KP);
    ui->KiEdit->setText(PID_KI);
    ui->KdEdit->setText(PID_KD);
}

void TrajectoryFollowingForm::on_sendToRobotButton_clicked()
{
    this->saveSettings();
    emit sendPID();
}
