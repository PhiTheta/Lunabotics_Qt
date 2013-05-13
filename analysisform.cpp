#include "analysisform.h"
#include "ui_analysisform.h"
#include <QDebug>

AnalysisForm::AnalysisForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::AnalysisForm)
{
    ui->setupUi(this);
    this->tableModel = new QStandardItemModel(0, 4);
    ui->tableView->setModel(this->tableModel);
    this->resetTableModel();
}

AnalysisForm::~AnalysisForm()
{
    delete ui;
    delete this->tableModel;
}

void AnalysisForm::resetTableModel()
{
    this->tableModel->clear();
    this->tableModel->setHorizontalHeaderItem(0, new QStandardItem(QString("Start")));
    this->tableModel->setHorizontalHeaderItem(1, new QStandardItem(QString("End")));
    this->tableModel->setHorizontalHeaderItem(2, new QStandardItem(QString("Curvature")));
    this->tableModel->setHorizontalHeaderItem(3, new QStandardItem(QString("Radius")));
}

void AnalysisForm::closeEvent(QCloseEvent *event)
{
    emit closing();
    event->accept();
}

void AnalysisForm::updateCurves(QVector<lunabotics::proto::Telemetry::Path::Curve> curves)
{
    this->resetTableModel();
    for (int i = 0; i < curves.size(); i++) {
        lunabotics::proto::Telemetry::Path::Curve curve = curves.at(i);

        qDebug() << curve.start_idx() << " " << curve.end_idx();

        QStandardItem *item = new QStandardItem(QString::number(curve.start_idx()));
        this->tableModel->setItem(i, 0, item);
        item = new QStandardItem(QString::number(curve.end_idx()));
        this->tableModel->setItem(i, 1, item);
        item = new QStandardItem(QString::number(curve.curvature(), 'f', 2));
        this->tableModel->setItem(i, 2, item);
        item = new QStandardItem(QString::number(1/curve.curvature(), 'f', 2));
        this->tableModel->setItem(i, 3, item);

    }
}

void AnalysisForm::updateRadius(float minRadius)
{
    if (minRadius >= 0) {
        ui->minRadiusLabel->setText(QString("%1 m").arg(QString::number(minRadius, 'f', 5)));
    }
    else {
        ui->minRadiusLabel->setText("? m");
    }
}
