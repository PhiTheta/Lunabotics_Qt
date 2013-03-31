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

    QString inIPString = settings.value("in_ip", CONN_INCOMING_ADDR).toString();
    if (inIPString.compare("any") == 0) {
        ui->anyInIPCheckBox->setChecked(true);
        ui->inIPLineEdit->setEnabled(false);
        inIPString = CONN_INCOMING_ADDR;
    }
    else {
        ui->anyInIPCheckBox->setChecked(false);
        ui->inIPLineEdit->setEnabled(true);
    }

    ui->outIPLineEdit->setText(settings.value("out_ip", CONN_OUTGOING_ADDR).toString());
    ui->inIPLineEdit->setText(inIPString);
    ui->outPortLineEdit->setText(settings.value("out_port", CONN_OUTGOING_PORT).toString());
    ui->inPortLineEdit->setText(settings.value("in_port", CONN_INCOMING_PORT).toString());
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
    settings.setValue("out_ip", ui->outIPLineEdit->text());
    settings.setValue("in_ip", ui->anyInIPCheckBox->isChecked() ? "any" : ui->inIPLineEdit->text());
    settings.setValue("out_port", ui->outPortLineEdit->text());
    settings.setValue("in_port", ui->inPortLineEdit->text());
    settings.endGroup();
    settings.beginGroup("pid");
    settings.setValue("p", ui->KpLineEdit->text());
    settings.setValue("i", ui->KiLineEdit->text());
    settings.setValue("d", ui->KdLineEdit->text());
    settings.setValue("offset", ui->PIDOffsetEdit->text());
    settings.setValue("v", ui->PIDVelocityMEdit->text());
    settings.endGroup();
}

void PreferenceDialog::on_anyInIPCheckBox_clicked(bool checked)
{
    ui->inIPLineEdit->setEnabled(!checked);
}

void PreferenceDialog::on_profileComboBox_currentIndexChanged(int index)
{
    switch (index) {
    case 0:
        ui->anyInIPCheckBox->setChecked(false);
        ui->inIPLineEdit->setEnabled(true);
        ui->inIPLineEdit->setText("192.168.218.1");
        ui->inPortLineEdit->setText("5556");
        ui->outIPLineEdit->setText("192.168.218.132");
        ui->outPortLineEdit->setText("5555");
        break;
    case 1:
        ui->anyInIPCheckBox->setChecked(false);
        ui->inIPLineEdit->setEnabled(true);
        ui->inIPLineEdit->setText("192.168.1.113");
        ui->inPortLineEdit->setText("5556");
        ui->outIPLineEdit->setText("192.168.1.110");
        ui->outPortLineEdit->setText("5555");
        break;
    case 2:
        ui->anyInIPCheckBox->setChecked(true);
        ui->inIPLineEdit->setEnabled(false);
        ui->inPortLineEdit->setText("5556");
        ui->outIPLineEdit->setText("10.0.1.10");
        ui->outPortLineEdit->setText("5555");
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
