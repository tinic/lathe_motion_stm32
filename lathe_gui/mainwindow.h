#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "commthread.h"

class TerminateableThread : public QThread {
    Q_OBJECT
public:
    TerminateableThread():QThread() {
       this->setTerminationEnabled(true);
    }
};

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

signals:
    void newfeed();

private slots:
   void update();

   void zeroButtonClicked();

   void stopButtonToggled(bool);
   void idleButtonToggled(bool);

   void aButtonToggled(bool);
   void xButtonToggled(bool);
   void zButtonToggled(bool);
   void directionButtonToggled(bool);

   void settingsDEGButtonToggled(bool);
   void settingsMMButtonToggled(bool);
   void settingsINButtonToggled(bool);

   void settings01ButtonToggled(bool);
   void settings02ButtonToggled(bool);
   void settings03ButtonToggled(bool);
   void settings04ButtonToggled(bool);
   void settings05ButtonToggled(bool);
   void settings06ButtonToggled(bool);
   void settings07ButtonToggled(bool);
   void settings08ButtonToggled(bool);
   void settings09ButtonToggled(bool);
   void settings10ButtonToggled(bool);
   void settings11ButtonToggled(bool);
   void settings12ButtonToggled(bool);
   void settings13ButtonToggled(bool);
   void settings14ButtonToggled(bool);
   void settings15ButtonToggled(bool);
   void settings16ButtonToggled(bool);
   void settings17ButtonToggled(bool);
   void settings18ButtonToggled(bool);
   void settings19ButtonToggled(bool);
   void settings20ButtonToggled(bool);

private:
   void updateStepValues();
   void untoggleButtons();
   void setNewButtonSetting(uint32_t buttonIdx);
   void passToCommThread();

    Ui::MainWindow *ui;

    int settingsAxisMode = 0;   // 0 == Z
                                // 1 == X
                                // 2 == compound

    int settingsSetPage = 0;    // 0 == mm
                                // 1 == in
                                // 2 == deg


    int settingsCurrent[3] = { 0 };

    double currentDirectionValue;
    double currentStepValue;
    double currentAngleValue;

    CommThread commThread;
    TerminateableThread qThread;
};

#endif // MAINWINDOW_H
