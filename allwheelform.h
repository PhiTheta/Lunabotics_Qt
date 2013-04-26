#ifndef ALLWHEELFORM_H
#define ALLWHEELFORM_H

#include <QWidget>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include "Telecommand.pb.h"

namespace Ui {
class AllWheelForm;
}

class AllWheelForm : public QWidget
{
    Q_OBJECT
    
public:
    explicit AllWheelForm(QWidget *parent = 0);
    ~AllWheelForm();
    void closeEvent(QCloseEvent *event);

signals:
    void predefinedControlSelected(lunabotics::AllWheelControl::PredefinedControlType controlType);
    void explicitControlSelected(float slf, float srf, float slr, float srr, float dlf, float drf, float dlr, float drr);
    void ICRControlSelected(QPointF ICR, float velocity);
    void closing();

private slots:
    void allWheelStateUpdated(float slf, float srf, float slr, float srr, float dlf, float drf, float dlr, float drr);
    void ICRUpdated(QPointF ICR);
    void updateJoints(QPointF leftFront, QPointF rightFront, QPointF leftRear, QPointF rightRear);

    void on_turnRightButton_clicked();

    void on_turnLeftButton_clicked();

    void on_rightButton_clicked();

    void on_leftButton_clicked();

    void on_backwardButton_clicked();

    void on_forwardButton_clicked();

    void on_allWheelButton_clicked();

    void on_stopButton_clicked();

    void on_sendICRButton_clicked();

private:
    Ui::AllWheelForm *ui;
    QGraphicsScene *robotSketchScene;
    QGraphicsRectItem *leftFrontWheel;
    QGraphicsRectItem *rightFrontWheel;
    QGraphicsRectItem *leftRearWheel;
    QGraphicsRectItem *rightRearWheel;
    QGraphicsRectItem *frontWheel;
    QGraphicsRectItem *rearWheel;
    QGraphicsRectItem *verticalICR;
    QGraphicsRectItem *horizontalICR;
    QGraphicsRectItem *baseLink;

    QTransform leftFrontTransform;
    QTransform rightFrontTransform;
    QTransform leftRearTransform;
    QTransform rightRearTransform;
    QTransform frontTransform;
    QTransform rearTransform;

    void redrawSketch();
    void createGrphicItems();

    //All wheel steering state
    float slf;
    float srf;
    float slr;
    float srr;
    float dlf;
    float drf;
    float dlr;
    float drr;
    QPointF ICR;
    QPointF leftFront;
    QPointF rightFront;
    QPointF leftRear;
    QPointF rightRear;
    bool jointPositionsAcquired;
    bool graphicItemsCreated;
};

#endif // ALLWHEELFORM_H
