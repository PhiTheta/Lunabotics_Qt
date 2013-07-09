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
    ui->outIPLineEdit->setText(settings.value(SETTINGS_IP, CONN_OUTGOING_ADDR).toString());
    ui->portLineEdit->setText(settings.value(SETTINGS_REMOTE_PORT, CONN_REMOTE_PORT).toString());
    ui->selfPortLineEdit->setText(settings.value(SETTINGS_LOCAL_PORT, CONN_LOCAL_PORT).toString());
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
    settings.setValue(SETTINGS_IP, ui->outIPLineEdit->text());
    settings.setValue(SETTINGS_REMOTE_PORT, ui->portLineEdit->text());
    settings.setValue(SETTINGS_LOCAL_PORT, ui->selfPortLineEdit->text());
    settings.endGroup();
}

void PreferenceDialog::on_profileComboBox_currentIndexChanged(int index)
{
    switch (index) {
    case 0:
        ui->outIPLineEdit->setText("192.168.218.151");
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
