#ifndef ALLWHEELFORM_H
#define ALLWHEELFORM_H

#include <QWidget>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QStandardItem>
#include "Telecommand.pb.h"
#include "allwheelstate.h"
#include "robotgeometry.h"

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
    void predefinedControlSelected(lunabotics::proto::AllWheelControl::PredefinedControlType controlType);
    void explicitControlSelected(AllWheelState *steering, AllWheelState *driving);
    void ICRControlSelected(QPointF ICR, float velocity);
    void closing();

private slots:
    void allWheelStateUpdated(AllWheelState *steering, AllWheelState *driving);
    void ICRUpdated(QPointF ICR);
    void updateJoints(RobotGeometry *geometry);

    void on_turnRightButton_clicked();

    void on_turnLeftButton_clicked();

    void on_rightButton_clicked();

    void on_leftButton_clicked();

    void on_backwardButton_clicked();

    void on_forwardButton_clicked();

    void on_stopButton_clicked();

    void on_sendICRButton_clicked();

    void on_resetButton_clicked();

    void on_sendButton_clicked();

private:
    Ui::AllWheelForm *ui;
    QGraphicsScene *robotSketchScene;
    QGraphicsRectItem *leftFrontWheel;
    QGraphicsRectItem *rightFrontWheel;
    QGraphicsRectItem *leftRearWheel;
    QGraphicsRectItem *rightRearWheel;
    QGraphicsRectItem *verticalICR;
    QGraphicsRectItem *horizontalICR;
    QGraphicsRectItem *baseLink;
    QGraphicsRectItem *leftFrontLink;
    QGraphicsRectItem *rightFrontLink;
    QGraphicsRectItem *leftRearLink;
    QGraphicsRectItem *rightReartLink;
    QStandardItemModel *stateTableModel;

    void redrawSketch();
    void createGrphicItems();
    void setRow(int &rowNumber, const QString &label, const QString &value);
    void updateTable();

    //All wheel steering state
    AllWheelState *steeringMotors;
    AllWheelState *drivingMotors;
    RobotGeometry *geometry;
    QPointF ICR;
    bool graphicItemsCreated;
};

#endif // ALLWHEELFORM_H
