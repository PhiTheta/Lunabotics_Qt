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
    this->localFrameInfo = new MapViewMetaInfo(ui->graphicsView->width(), ui->graphicsView->height());

    this->mode = LocalFrameAckermann;

    this->localFrameScene = new QGraphicsScene(this);
    ui->graphicsView->setScene(this->localFrameScene);


    this->localFrameScene->addLine(0, -5, 7, 7, PEN_RED_BOLD);
    this->localFrameScene->addLine(7, 7, -7, 7, PEN_RED_BOLD);
    this->localFrameScene->addLine(-7, 7, 0, -5, PEN_RED_BOLD);

    this->feedbackErrorLineItem = new QGraphicsLineItem();
    this->feedbackErrorLineItem->setPen(PEN_RED);
    this->feedbackErrorLineItem->hide();
    this->localFrameScene->addItem(this->feedbackErrorLineItem);

    this->feedbackLookAheadLineItem = new QGraphicsLineItem();
    this->feedbackLookAheadLineItem->setPen(PEN_BLUE);
    this->feedbackLookAheadLineItem->hide();
    this->localFrameScene->addItem(this->feedbackLookAheadLineItem);

    this->feedbackPathPointEllipseItem = new QGraphicsEllipseItem(-2, -2, 4, 4);
    this->feedbackPathPointEllipseItem->setPen(PEN_RED);
    this->feedbackPathPointEllipseItem->setBrush(BRUSH_CLEAR);
    this->feedbackPathPointEllipseItem->hide();
    this->localFrameScene->addItem(this->feedbackPathPointEllipseItem);

    this->feedbackPointEllipseItem = new QGraphicsEllipseItem(-2, -2, 4, 4);
    this->feedbackPointEllipseItem->setPen(PEN_BLUE);
    this->feedbackPointEllipseItem->setBrush(BRUSH_CLEAR);
    this->feedbackPointEllipseItem->hide();
    this->localFrameScene->addItem(this->feedbackPointEllipseItem);

    this->feedforwardItems = new QGraphicsItemGroup();
    this->feedforwardItems->hide();
    this->localFrameScene->addItem(this->feedforwardItems);


    this->feedforwardCenterItem = new QGraphicsEllipseItem(-2, -2, 4, 4);
    this->feedforwardCenterItem->setPen(PEN_PURPLE);
    this->feedforwardCenterItem->setBrush(BRUSH_CLEAR);
    this->feedforwardCenterItem->hide();
    this->localFrameScene->addItem(this->feedforwardCenterItem);

    this->deviationLineItem = new QGraphicsLineItem();
    this->deviationLineItem->setPen(PEN_RED);
    this->deviationLineItem->hide();
    this->localFrameScene->addItem(this->deviationLineItem);

    this->deviationPathPointEllipseItem = new QGraphicsEllipseItem(-2, -2, 4, 4);
    this->deviationPathPointEllipseItem->setPen(PEN_RED);
    this->deviationPathPointEllipseItem->setBrush(BRUSH_CLEAR);
    this->deviationPathPointEllipseItem->hide();
    this->localFrameScene->addItem(this->deviationPathPointEllipseItem);


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
    this->feedbackErrorLineItem->hide();
    this->feedbackLookAheadLineItem->hide();
    this->feedbackPathPointEllipseItem->hide();
    this->feedbackPointEllipseItem->hide();
    this->feedforwardItems->hide();
    this->deviationPathPointEllipseItem->hide();
    this->deviationLineItem->hide();
    this->feedforwardCenterItem->hide();
}



void TrajectoryFollowingForm::updateLocalFrame(QPointF feedbackPoint, QPointF feedbackPathPoint, QVector<QPointF> feedforwardPoints, QPointF feedforwardCenter)
{
    this->mode = LocalFrameAckermann;
    this->deviationLineItem->hide();
    this->deviationPathPointEllipseItem->hide();


    bool pointOk = isvalid(feedbackPoint);
    this->feedbackLookAheadLineItem->setVisible(pointOk);
    this->feedbackPointEllipseItem->setVisible(pointOk);
    if (pointOk) {
        qreal x = -feedbackPoint.y()*SCALE;
        qreal y = -feedbackPoint.x()*SCALE;
        this->feedbackPointEllipseItem->setPos(x, y);
        this->feedbackLookAheadLineItem->setLine(0, 0, x, y);

        pointOk = isvalid(feedbackPathPoint);
        this->feedbackPathPointEllipseItem->setVisible(pointOk);
        this->feedbackErrorLineItem->setVisible(pointOk);

        if (pointOk) {
            qreal x1 = -feedbackPathPoint.y()*SCALE;
            qreal y1 = -feedbackPathPoint.x()*SCALE;

            this->feedbackPathPointEllipseItem->setPos(x1, y1);
            this->feedbackErrorLineItem->setLine(x, y, x1, y1);
        }
    }

    pointOk = isvalid(feedforwardCenter);

    this->feedforwardItems->setVisible(pointOk);
    if (pointOk) {

        qreal x1 = -feedforwardCenter.y()*SCALE;
        qreal y1 = -feedforwardCenter.x()*SCALE;
        if (x1 > -ui->graphicsView->width()/2 && x1 < ui->graphicsView->width()/2 &&
            y1 > -ui->graphicsView->height()/2 && y1 < ui->graphicsView->height()/2) {
            this->feedforwardCenterItem->setPos(x1, y1);
            this->feedforwardCenterItem->show();
        }
        else {
            this->feedforwardCenterItem->hide();
        }

        qDeleteAll(this->feedforwardItems->childItems());
        for (int i = 0; i < feedforwardPoints.size() && i < 5; i++) {
            QPointF point = feedforwardPoints.at(i);
            if (isvalid(point)) {
                qreal x = -point.y()*SCALE;
                qreal y = -point.x()*SCALE;
                QGraphicsLineItem *line = new QGraphicsLineItem(x, y, x1, y1);
                line->setPen(PEN_GREEN);
                QGraphicsEllipseItem *ellipse = new QGraphicsEllipseItem(x-2, y-2, 4, 4);
                ellipse->setPen(PEN_GREEN);
                ellipse->setBrush(BRUSH_CLEAR);
                this->feedforwardItems->addToGroup(ellipse);
               // this->feedforwardPointsItem->addToGroup(line);
            }
        }
    }
}

void TrajectoryFollowingForm::updateLocalFrame(QPointF deviationPathPoint)
{
    this->mode = LocalFramePointTurn;
    this->feedbackErrorLineItem->hide();
    this->feedbackLookAheadLineItem->hide();
    this->feedbackPathPointEllipseItem->hide();
    this->feedbackPointEllipseItem->hide();
    this->feedforwardItems->hide();
    this->feedforwardCenterItem->hide();

    bool pointOk = isvalid(deviationPathPoint);
    this->deviationPathPointEllipseItem->setVisible(pointOk);
    this->deviationLineItem->setVisible(pointOk);

    if (pointOk) {
        qreal x = -deviationPathPoint.y()*SCALE;
        qreal y = -deviationPathPoint.x()*SCALE;
        this->deviationPathPointEllipseItem->setPos(x, y);
        this->deviationLineItem->setLine(0, 0, x, y);
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
