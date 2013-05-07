#ifndef PREFERENCEDIALOG_H
#define PREFERENCEDIALOG_H

#include <QDialog>

namespace Ui {
class PreferenceDialog;
}

class PreferenceDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PreferenceDialog(QWidget *parent = 0);
    ~PreferenceDialog();
    void saveSettings();
    
private slots:
    void on_buttonBox_accepted();

    void on_profileComboBox_currentIndexChanged(int index);

private:
    Ui::PreferenceDialog *ui;
};

#endif // PREFERENCEDIALOG_H
