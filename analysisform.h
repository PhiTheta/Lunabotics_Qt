#ifndef ANALYSISFORM_H
#define ANALYSISFORM_H

#include <QWidget>
#include <QVector>
#include <QStandardItemModel>
#include "Telemetry.pb.h"
#include <QCloseEvent>

namespace Ui {
class AnalysisForm;
}

class AnalysisForm : public QWidget
{
    Q_OBJECT


signals:
    void closing();

public slots:
    void updateCurves(QVector<lunabotics::proto::Telemetry::Path::Curve> curves);
    void updateRadius(float minRadius);

public:
    explicit AnalysisForm(QWidget *parent = 0);
    ~AnalysisForm();
    void closeEvent(QCloseEvent *event);
    
private:
    Ui::AnalysisForm *ui;
    QStandardItemModel *tableModel;

    void resetTableModel();
};

#endif // ANALYSISFORM_H
