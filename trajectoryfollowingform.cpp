#include "trajectoryfollowingform.h"
#include "ui_trajectoryfollowingform.h"
#include "Graphics.h"
#include "constants.h"
#include <QDebug>
#include <QCloseEvent>
#include <QSettings>
#include <QtCore>

#define SCALE   1//0.0

TrajectoryFollowingForm::TrajectoryFollowingForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TrajectoryFollowingForm)
{
    ui->setupUi(this);

    this->localFrameInfo = new MapViewMetaInfo(ui->graphicsView->width(), ui->graphicsView->height());

    QSettings settings("ivany4", "lunabotics");
    settings.beginGroup("pid");
    ui->KpEdit->setText(settings.value(SETTING_PID_P, PID_KP).toString());
    ui->KiEdit->setText(settings.value(SETTING_PID_I, PID_KI).toString());
    ui->KdEdit->setText(settings.value(SETTING_PID_D, PID_KD).toString());
    ui->feedbackOffsetEdit->setText(settings.value(SETTING_FEEDBACK_MIN_OFFSET, FEEDBACK_MIN_OFFSET).toString());
    ui->feedbackMultiplierEdit->setText(settings.value(SETTING_FEEDBACK_MULTIPLIER, FEEDBACK_MULTIPLIER).toString());
    ui->feedforwardFractionEdit->setText(settings.value(SETTING_FEEDFORWARD_FRACTION, FEEDFORWARD_FRACTION).toString());
    ui->feedforwardOffsetEdit->setText(settings.value(SETTING_FEEDFORWARD_MIN_OFFSET, FEEDFORWARD_MIN_OFFSET).toString());
    settings.endGroup();

    this->localFrameScene = new QGraphicsScene(this);
    ui->graphicsView->setScene(this->localFrameScene);
    this->clearLocalFrame();
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
    settings.setValue(SETTING_PID_P, ui->KpEdit->text());
    settings.setValue(SETTING_PID_I, ui->KiEdit->text());
    settings.setValue(SETTING_PID_D, ui->KdEdit->text());
    settings.setValue(SETTING_FEEDBACK_MIN_OFFSET, ui->feedbackOffsetEdit->text());
    settings.setValue(SETTING_FEEDBACK_MULTIPLIER, ui->feedbackMultiplierEdit->text());
    settings.setValue(SETTING_FEEDFORWARD_FRACTION, ui->feedforwardFractionEdit->text());
    settings.setValue(SETTING_FEEDFORWARD_MIN_OFFSET, ui->feedforwardOffsetEdit->text());
    settings.endGroup();
}

void TrajectoryFollowingForm::clearLocalFrame()
{
    qDeleteAll(this->localFrameScene->items());
    this->localFrameScene->items().clear();
    this->feedbackLookAheadLineItem = NULL;
    this->feedbackErrorLineItem = NULL;
    this->feedbackPathPointEllipseItem = NULL;
    this->feedbackPointEllipseItem = NULL;
    this->feedforwardPointsItem = NULL;
    this->graphicItemsCreated = false;
}



void TrajectoryFollowingForm::updateLocalFrame(QPointF feedbackPoint, QPointF feedbackPathPoint, QVector<QPointF> feedforwardPoints, QPointF feedforwardCenter)
{
    QPointF origin(ui->graphicsView->width()/2, ui->graphicsView->height()/2);

    if (!this->graphicItemsCreated) {
        this->feedbackErrorLineItem = new QGraphicsLineItem();
        this->feedbackErrorLineItem->setPen(PEN_BLUE);
        this->localFrameScene->addItem(this->feedbackErrorLineItem);
        this->feedbackLookAheadLineItem = new QGraphicsLineItem();
        this->feedbackLookAheadLineItem->setPen(PEN_RED);
        this->localFrameScene->addItem(this->feedbackLookAheadLineItem);
        this->feedbackPathPointEllipseItem = new QGraphicsEllipseItem(-2, -2, 4, 4);
        this->feedbackPathPointEllipseItem->setPen(PEN_BLUE);
        this->feedbackPathPointEllipseItem->setBrush(BRUSH_BLUE);
        this->localFrameScene->addItem(this->feedbackPathPointEllipseItem);
        this->feedbackPointEllipseItem = new QGraphicsEllipseItem(-1, -1, 2, 2);
        this->feedbackPointEllipseItem->setPen(PEN_RED);
        this->feedbackPointEllipseItem->setBrush(BRUSH_RED);
        this->localFrameScene->addItem(this->feedbackPointEllipseItem);
        this->graphicItemsCreated = true;

        this->localFrameScene->addLine(origin.x(), origin.y()-5, origin.x()+7, origin.y()+7, PEN_RED_BOLD);
        this->localFrameScene->addLine(origin.x()+7, origin.y()+7, origin.x()-7, origin.y()+7, PEN_RED_BOLD);
        this->localFrameScene->addLine(origin.x()-7, origin.y()+7, origin.x(), origin.y()-5, PEN_RED_BOLD);
    }

    if (this->feedforwardPointsItem) {
        this->localFrameScene->removeItem(this->feedforwardPointsItem);
        delete this->feedforwardPointsItem;
    }
    this->feedforwardPointsItem = new QGraphicsItemGroup();
    this->localFrameScene->addItem(this->feedforwardPointsItem);


    qreal x = origin.x()+feedbackPoint.x()*SCALE;
    qreal y = origin.y()+feedbackPoint.y()*SCALE;
    qreal x1 = origin.x()+feedbackPathPoint.x()*SCALE;
    qreal y1 = origin.y()+feedbackPathPoint.y()*SCALE;

    this->feedbackPathPointEllipseItem->setPos(x1, y1);
    this->feedbackPointEllipseItem->setPos(x, y);
    this->feedbackErrorLineItem->setLine(origin.x(), origin.y(), x, y);
    this->feedbackLookAheadLineItem->setLine(x, y, x1, y1);
    this->feedbackPathPointEllipseItem->setRotation(-90);
    this->feedbackPointEllipseItem->setRotation(-90);
    this->feedbackErrorLineItem->setRotation(-90);
    this->feedbackLookAheadLineItem->setRotation(-90);
    //this->localFrameScene->addEllipse(origin.x()-5, origin.y()-5, 10.0, 10.0, PEN_BLACK, BRUSH_GREEN);

    x1 = origin.x()+feedforwardCenter.x()*SCALE;
    y1 = origin.y()+feedforwardCenter.y()*SCALE;

    for (int i = 0; i < feedforwardPoints.size(); i++) {
        QPointF point = feedforwardPoints.at(i);
        y = origin.x()+point.x()*SCALE;
        x = origin.y()+point.y()*SCALE;
        QGraphicsLineItem *line = new QGraphicsLineItem(x, y, x1, y1);
        line->setPen(PEN_GREEN);
        QGraphicsEllipseItem *ellipse = new QGraphicsEllipseItem(x-2, y-2, 4, 4);
        ellipse->setPen(PEN_GREEN);
        ellipse->setBrush(BRUSH_CLEAR);
        this->feedforwardPointsItem->addToGroup(ellipse);
        this->feedforwardPointsItem->addToGroup(line);
    }
}

void TrajectoryFollowingForm::on_resetButton_clicked()
{
    ui->KpEdit->setText(PID_KP);
    ui->KiEdit->setText(PID_KI);
    ui->KdEdit->setText(PID_KD);
    ui->feedbackMultiplierEdit->setText(FEEDBACK_MULTIPLIER);
    ui->feedbackOffsetEdit->setText(FEEDBACK_MIN_OFFSET);
    ui->feedforwardFractionEdit->setText(FEEDFORWARD_FRACTION);
    ui->feedforwardOffsetEdit->setText(FEEDFORWARD_MIN_OFFSET);
}

void TrajectoryFollowingForm::on_sendToRobotButton_clicked()
{
    this->saveSettings();
    emit sendPID();
}
