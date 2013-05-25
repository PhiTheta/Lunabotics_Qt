#include "trajectoryfollowingform.h"
#include "ui_trajectoryfollowingform.h"
#include "Graphics.h"
#include "constants.h"
#include "Common.h"
#include <QDebug>
#include <QCloseEvent>
#include <QSettings>
#include <QtCore>

#define SCALE   100.0

TrajectoryFollowingForm::TrajectoryFollowingForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TrajectoryFollowingForm)
{
    ui->setupUi(this);
    this->graphicItemsCreated = false;
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
    this->localFrameScene->addLine(0, -5, 7, 7, PEN_RED_BOLD);
    this->localFrameScene->addLine(7, 7, -7, 7, PEN_RED_BOLD);
    this->localFrameScene->addLine(-7, 7, 0, -5, PEN_RED_BOLD);
}



void TrajectoryFollowingForm::updateLocalFrame(QPointF feedbackPoint, QPointF feedbackPathPoint, QVector<QPointF> feedforwardPoints, QPointF feedforwardCenter)
{
    if (!this->graphicItemsCreated) {
        this->feedbackErrorLineItem = new QGraphicsLineItem();
        this->feedbackErrorLineItem->setPen(PEN_RED);
        this->localFrameScene->addItem(this->feedbackErrorLineItem);
        this->feedbackLookAheadLineItem = new QGraphicsLineItem();
        this->feedbackLookAheadLineItem->setPen(PEN_BLUE);
        this->localFrameScene->addItem(this->feedbackLookAheadLineItem);
        this->feedbackPathPointEllipseItem = new QGraphicsEllipseItem(-2, -2, 4, 4);
        this->feedbackPathPointEllipseItem->setPen(PEN_RED);
        this->feedbackPathPointEllipseItem->setBrush(BRUSH_CLEAR);
        this->localFrameScene->addItem(this->feedbackPathPointEllipseItem);
        this->feedbackPointEllipseItem = new QGraphicsEllipseItem(-2, -2, 4, 4);
        this->feedbackPointEllipseItem->setPen(PEN_BLUE);
        this->feedbackPointEllipseItem->setBrush(BRUSH_CLEAR);
        this->localFrameScene->addItem(this->feedbackPointEllipseItem);
        this->graphicItemsCreated = true;
    }

    if (this->feedforwardPointsItem) {
        this->localFrameScene->removeItem(this->feedforwardPointsItem);
        delete this->feedforwardPointsItem;
    }
    this->feedforwardPointsItem = new QGraphicsItemGroup();
    this->localFrameScene->addItem(this->feedforwardPointsItem);



    if (isvalid(feedbackPoint)) {
        qreal x = -feedbackPoint.y()*SCALE;
        qreal y = -feedbackPoint.x()*SCALE;
        this->feedbackPointEllipseItem->setPos(x, y);
        this->feedbackLookAheadLineItem->setLine(0, 0, x, y);
        if (isvalid(feedbackPathPoint)) {
            qreal x1 = -feedbackPathPoint.y()*SCALE;
            qreal y1 = -feedbackPathPoint.x()*SCALE;

            this->feedbackPathPointEllipseItem->setPos(x1, y1);
            this->feedbackErrorLineItem->setLine(x, y, x1, y1);
        }
    }

    if (isvalid(feedforwardCenter)) {
        qreal x1 = -feedforwardCenter.y()*SCALE;
        qreal y1 = -feedforwardCenter.x()*SCALE;
        if (x1 > -ui->graphicsView->width()/2 && x1 < ui->graphicsView->width()/2 &&
            y1 > -ui->graphicsView->height()/2 && y1 < ui->graphicsView->height()/2) {
            QGraphicsEllipseItem *ellipse = new QGraphicsEllipseItem(x1-2, y1-2, 4, 4);
            ellipse->setPen(PEN_PURPLE);
            ellipse->setBrush(BRUSH_CLEAR);
            this->feedforwardPointsItem->addToGroup(ellipse);
        }

        for (int i = 0; i < feedforwardPoints.size(); i++) {
            QPointF point = feedforwardPoints.at(i);
            if (isvalid(point)) {
                qreal x = -point.y()*SCALE;
                qreal y = -point.x()*SCALE;
                QGraphicsLineItem *line = new QGraphicsLineItem(x, y, x1, y1);
                line->setPen(PEN_GREEN);
                QGraphicsEllipseItem *ellipse = new QGraphicsEllipseItem(x-2, y-2, 4, 4);
                ellipse->setPen(PEN_GREEN);
                ellipse->setBrush(BRUSH_CLEAR);
                this->feedforwardPointsItem->addToGroup(ellipse);
               // this->feedforwardPointsItem->addToGroup(line);
            }
        }
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
