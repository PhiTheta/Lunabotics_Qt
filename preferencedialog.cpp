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
    ui->outIPLineEdit->setText(settings.value("out_ip", CONN_OUTGOING_ADDR).toString());
    ui->inIPLineEdit->setText(settings.value("in_ip", CONN_INCOMING_ADDR).toString());
    ui->outPortLineEdit->setText(settings.value("out_port", CONN_OUTGOING_PORT).toString());
    ui->inPortLineEdit->setText(settings.value("in_port", CONN_INCOMING_PORT).toString());
    settings.endGroup();
}

PreferenceDialog::~PreferenceDialog()
{
    delete ui;
}

void PreferenceDialog::on_buttonBox_accepted()
{
    QSettings settings("ivany4", "lunabotics");
    settings.beginGroup("connection");
    settings.setValue("out_ip", ui->outIPLineEdit->text());
    settings.setValue("in_ip", ui->inIPLineEdit->text());
    settings.setValue("out_port", ui->outPortLineEdit->text());
    settings.setValue("in_port", ui->inPortLineEdit->text());
    settings.endGroup();
}
