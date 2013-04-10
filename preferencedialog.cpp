#include "preferencedialog.h"
#include "ui_preferencedialog.h"
#include "constants.h"
#include <QSettings>
#include <QDebug>


PreferenceDialog::PreferenceDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PreferenceDialog)
{
    ui->setupUi(this);
    QSettings settings("ivany4", "lunabotics");

    settings.beginGroup("connection");
    ui->outIPLineEdit->setText(settings.value("ip", CONN_OUTGOING_ADDR).toString());
    settings.endGroup();
    settings.beginGroup("pid");
    ui->KpLineEdit->setText(settings.value("p", PID_KP).toString());
    ui->KiLineEdit->setText(settings.value("i", PID_KI).toString());
    ui->KdLineEdit->setText(settings.value("d", PID_KD).toString());
    ui->PIDOffsetEdit->setText(settings.value("offset", PID_OFFSET).toString());
    ui->PIDVelocityMEdit->setText(settings.value("v", PID_VEL_M).toString());
    settings.endGroup();
}

PreferenceDialog::~PreferenceDialog()
{
    delete ui;
}

void PreferenceDialog::on_buttonBox_accepted()
{
    QSettings settings( "ivany4", "lunabotics");
    settings.beginGroup("connection");
    settings.setValue("ip", ui->outIPLineEdit->text());
    settings.endGroup();
    settings.beginGroup("pid");
    settings.setValue("p", ui->KpLineEdit->text());
    settings.setValue("i", ui->KiLineEdit->text());
    settings.setValue("d", ui->KdLineEdit->text());
    settings.setValue("offset", ui->PIDOffsetEdit->text());
    settings.setValue("v", ui->PIDVelocityMEdit->text());
    settings.endGroup();
}

void PreferenceDialog::on_profileComboBox_currentIndexChanged(int index)
{
    switch (index) {
    case 0:
        ui->outIPLineEdit->setText("192.168.218.138");
        break;
    case 1:
        ui->outIPLineEdit->setText("192.168.1.110");
        break;
    case 2:
        ui->outIPLineEdit->setText("10.0.1.10");
        break;
    default:
        break;
    }
}

void PreferenceDialog::on_resetGainsButton_clicked()
{
    ui->KpLineEdit->setText(PID_KP);
    ui->KiLineEdit->setText(PID_KI);
    ui->KdLineEdit->setText(PID_KD);
}
