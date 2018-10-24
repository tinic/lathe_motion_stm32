#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QLabel>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    currentDirectionValue(1.0),
    currentStepValue(0.10),
    currentAngleValue(0.0)
{
    commThread.moveToThread(&qThread);
    connect(&qThread, SIGNAL (started()), &commThread, SLOT (run()));
    connect(&commThread, SIGNAL (update()), this, SLOT (update()));
    qThread.start();
    ui->setupUi(this);

    updateStepValues();
    settingsDirectionButtonToggled(true);
    passToCommThread();
    untoggleButtons();
}

MainWindow::~MainWindow()
{
    commThread.keepRunning = false;
    qThread.quit();
    qThread.wait();
    delete ui;
}

void MainWindow::update()
{
    lathe_status_packet local_packet;
    commThread.statusMutex.lock();
    memcpy(&local_packet, &commThread.status_packet, sizeof(lathe_status_packet));
    commThread.statusMutex.unlock();

    double rpm = local_packet.current_index_delta ? (60000.0 / double(local_packet.current_index_delta)) : 0.0;
    if (rpm < 60) rpm = 0;

    char str[256] = {0};
    sprintf(str, "POSZ: %c%07.02f%s\n"
                 "POSX: %c%07.02f%s\n"
                 "POSA: %c %06.02f%s\n"
                 "RPM :  %07.02f\n",
            local_packet.stepper_actual_pos_z < 0 ? '-' : '+',
            ((fabs(double(local_packet.stepper_actual_pos_z)) / 3200.0) / 12.0) * 25.4,
            "mm",
            local_packet.stepper_actual_pos_x < 0 ? '-' : '+',
            ((fabs(double(local_packet.stepper_actual_pos_x)) / 3200.0) / 12.0) * 25.4,
            "mm",
            local_packet.absolute_cnt < 0 ? '-' : '+',
            ( fabs(double(local_packet.absolute_cnt)) / 2880.0) * 360,
            "°",
            rpm);

   QLabel *statusLabel = findChild<QLabel *>("statusLabel");
   statusLabel->setText(str);

   QLabel *rawStatusLabel = findChild<QLabel *>("rawStatusLabel");

   sprintf(str,"A:%c%012llx "
               "Z:%c%012llx "
               "X:%c%012llx "
               "R:%c%04lx "
               "I:%c%04lx "
               "D:%c%04lx "
               "O:%c%04lx "
               "ZM:%c%06lx "
               "ZD:%c%06lx "
               "XM:%c%06lx "
               "XD:%c%06lx "
               "ES:%c%04lx "
               "EO:%c%04lx "
               "T:%c%08lx ",
           local_packet.absolute_pos >=0 ? '+' : '-',
           llabs(local_packet.absolute_pos),
           local_packet.stepper_actual_pos_z >=0 ? '+' : '-',
           llabs(local_packet.stepper_actual_pos_z),
           local_packet.stepper_actual_pos_x >=0 ? '+' : '-',
           llabs(local_packet.stepper_actual_pos_x),
           local_packet.absolute_cnt >=0 ? '+' : '-',
           labs(local_packet.absolute_cnt),
           local_packet.absolute_idx >=0 ? '+' : '-',
           labs(local_packet.absolute_idx),
           local_packet.current_index_delta >=0 ? '+' : '-',
           labs(local_packet.current_index_delta),
           local_packet.absolute_pos_start_offset >=0 ? '+' : '-',
           labs(local_packet.absolute_pos_start_offset),
           local_packet.stepper_follow_mul_z >=0 ? '+' : '-',
           labs(local_packet.stepper_follow_mul_z),
           local_packet.stepper_follow_div_z >=0 ? '+' : '-',
           labs(local_packet.stepper_follow_div_z),
           local_packet.stepper_follow_mul_x >=0 ? '+' : '-',
           labs(local_packet.stepper_follow_mul_x),
           local_packet.stepper_follow_div_x >=0 ? '+' : '-',
           labs(local_packet.stepper_follow_div_x),
           local_packet.error_state_out_of_sync >=0 ? '+' : '-',
           labs(local_packet.error_state_out_of_sync),
           local_packet.error_index_offset >=0 ? '+' : '-',
           labs(local_packet.error_index_offset),
           local_packet.absolute_tick >=0 ? '+' : '-',
           labs(local_packet.absolute_tick));
   rawStatusLabel->setText(str);
}

static const double settingsValues [3][20] =
{
    { 0.01,0.025, 0.05, 0.10,
      0.35, 0.40, 0.45, 0.50,
      0.60, 0.70, 0.80, 1.00,
      1.25, 1.50, 1.75, 2.00,
      2.50, 3.00, 3.50, 4.00
    },
    { 0.0254,0.127, 0.254, 0.508,
      25.4 / 40, 25.4 / 36, 25.4 / 32, 25.4 / 28,
      25.4 / 24, 25.4 / 20, 25.4 / 18, 25.4 / 16,
      25.4 / 14, 25.4 / 13, 25.4 / 12, 25.4 / 11,
      25.4 / 10, 25.4 / 9, 25.4 / 8, 25.4 / 7,
    },
    { 1.4908,1.4287, 1.4307, 1.4377,
      0.635, .7055555555555, .79375, 0.907,
      1.05833333, 1.270, 1.411, 1.588,
      1.814, 1.954, 2.117, 2.309,
      2.540, 2.822, 3.175, 3.629,
    }
};

static const char *settingsNames [3][20] =
{
    { "0.01","0.025", "0.05", "0.10",
      "0.35", "0.40", "0.45", "0.50",
      "0.60", "0.70", "0.80", "1.00",
      "1.25", "1.50", "1.75", "2.00",
      "2.50", "3.00", "3.50", "4.00"
    },
    { "0.001", "0.005", "0.010", "0.020",
      "40tpi", "36tpi", "32tpi", "28tpi",
      "24tpi", "20tpi", "18tpi", "16tpi",
      "14tpi", "13tpi", "12tpi", "11tpi",
      "10tpi", "9tpi", "8tpi", "7tpi"
    },
    { "MT#0", "MT#1", "MT#2", "MT#3",
      "MT#4", "MT#5", "MT#6", "MT#7",
      "NPT", "4.50", "5.625", "7.50",
      "10.00", "12.50", "15.00", "20.00",
      "25.00", "30.00", "35.00", "45"
    },
};

void MainWindow::passToCommThread()
{
    commThread.newzfeed = currentStepValue * currentDirectionValue;
    commThread.newxfeed = 0.0;
}

void MainWindow::setZeroButtonClicked()
{
    commThread.setzero = true;
}

void MainWindow::setNewButtonSetting(uint32_t buttonIdx)
{
    QPushButton *button0 = findChild<QPushButton *>("setMMButton");
    QPushButton *button1 = findChild<QPushButton *>("setINButton");
    QPushButton *button2 = findChild<QPushButton *>("setDEGButton");

    int32_t index = 0;
    if (button2->isChecked()) index = 2;
    if (button1->isChecked()) index = 1;
    if (button0->isChecked()) index = 0;

    if (index == 0 || index == 1) {
        currentStepValue = settingsValues[index][buttonIdx-1];
    } else {
        currentAngleValue = settingsValues[index][buttonIdx-1];
    }
    passToCommThread();
}

void MainWindow::untoggleButtons() {
    QPushButton *button0 = findChild<QPushButton *>("setMMButton");
    QPushButton *button1 = findChild<QPushButton *>("setINButton");
    QPushButton *button2 = findChild<QPushButton *>("setDEGButton");

    int32_t index = 0;
    if (button2->isChecked()) index = 2;
    if (button1->isChecked()) index = 1;
    if (button0->isChecked()) index = 0;

    for (uint32_t c=1; c<=20; c++) {
        char str[64]; sprintf(str, "set%02dButton", c);
        QPushButton *button = findChild<QPushButton *>(str);
        if (index == 0 || index == 1) {
            if (fabs(settingsValues[index][c-1] - currentStepValue) < 0.0001) {
                if (button) button->setChecked(true);
            } else {
                if (button) button->setChecked(false);
            }
        } else {
            if (fabs(settingsValues[index][c-1] - currentAngleValue) < 0.0001) {
                if (button) button->setChecked(true);
            } else {
                if (button) button->setChecked(false);
            }
        }
    }
}

void MainWindow::settingsDirectionButtonToggled(bool) {
    QPushButton *button = findChild<QPushButton *>("setDirectionButton");
    if (button->isChecked()) {
        button->setText("⮆");
        currentDirectionValue = -1.0;
    } else {
        button->setText("⮄");
        currentDirectionValue = +1.0;
    }
    passToCommThread();
}

void MainWindow::updateStepValues() {
    QPushButton *button0 = findChild<QPushButton *>("setMMButton");
    QPushButton *button1 = findChild<QPushButton *>("setINButton");
    QPushButton *button2 = findChild<QPushButton *>("setDEGButton");

    int32_t index = 0;
    if (button2->isChecked()) index = 2;
    if (button1->isChecked()) index = 1;
    if (button0->isChecked()) index = 0;

    for (uint32_t c=1; c<=20; c++) {
        char str[64]; sprintf(str, "set%02dButton", c);
        QPushButton *button = findChild<QPushButton *>(str);
        button->setText(settingsNames[index][c-1]);
    }
}

void MainWindow::settingsMMButtonToggled(bool){
    QPushButton *button0 = findChild<QPushButton *>("setMMButton");
    button0->setChecked(true);
    QPushButton *button1 = findChild<QPushButton *>("setINButton");
    button1->setChecked(false);
    QPushButton *button2 = findChild<QPushButton *>("setDEGButton");
    button2->setChecked(false);
    updateStepValues();
    untoggleButtons();
}

void MainWindow::settingsINButtonToggled(bool){
    QPushButton *button0 = findChild<QPushButton *>("setMMButton");
    button0->setChecked(false);
    QPushButton *button1 = findChild<QPushButton *>("setINButton");
    button1->setChecked(true);
    QPushButton *button2 = findChild<QPushButton *>("setDEGButton");
    button2->setChecked(false);
    updateStepValues();
    untoggleButtons();
}

void MainWindow::settingsDEGButtonToggled(bool){
    QPushButton *button0 = findChild<QPushButton *>("setMMButton");
    button0->setChecked(false);
    QPushButton *button1 = findChild<QPushButton *>("setINButton");
    button1->setChecked(false);
    QPushButton *button2 = findChild<QPushButton *>("setDEGButton");
    button2->setChecked(true);
    updateStepValues();
    untoggleButtons();
}

void MainWindow::settings01ButtonToggled(bool){
    setNewButtonSetting(1);
    untoggleButtons();
}

void MainWindow::settings02ButtonToggled(bool){
    setNewButtonSetting(2);
    untoggleButtons();
}

void MainWindow::settings03ButtonToggled(bool){
    setNewButtonSetting(3);
    untoggleButtons();
}

void MainWindow::settings04ButtonToggled(bool){
    setNewButtonSetting(4);
    untoggleButtons();
}

void MainWindow::settings05ButtonToggled(bool){
    setNewButtonSetting(5);
    untoggleButtons();
}

void MainWindow::settings06ButtonToggled(bool){
    setNewButtonSetting(6);
    untoggleButtons();
}

void MainWindow::settings07ButtonToggled(bool){
    setNewButtonSetting(7);
    untoggleButtons();
}

void MainWindow::settings08ButtonToggled(bool){
    setNewButtonSetting(8);
    untoggleButtons();
}

void MainWindow::settings09ButtonToggled(bool){
    setNewButtonSetting(9);
    untoggleButtons();
}

void MainWindow::settings10ButtonToggled(bool){
    setNewButtonSetting(10);
    untoggleButtons();
}

void MainWindow::settings11ButtonToggled(bool){
    setNewButtonSetting(11);
    untoggleButtons();
}

void MainWindow::settings12ButtonToggled(bool){
    setNewButtonSetting(12);
    untoggleButtons();
}

void MainWindow::settings13ButtonToggled(bool){
    setNewButtonSetting(13);
    untoggleButtons();
}

void MainWindow::settings14ButtonToggled(bool){
    setNewButtonSetting(14);
    untoggleButtons();
}

void MainWindow::settings15ButtonToggled(bool){
    setNewButtonSetting(15);
    untoggleButtons();
}

void MainWindow::settings16ButtonToggled(bool){
    setNewButtonSetting(16);
    untoggleButtons();
}

void MainWindow::settings17ButtonToggled(bool){
    setNewButtonSetting(17);
    untoggleButtons();
}

void MainWindow::settings18ButtonToggled(bool){
    setNewButtonSetting(18);
    untoggleButtons();
}

void MainWindow::settings19ButtonToggled(bool){
    setNewButtonSetting(19);
    untoggleButtons();
}

void MainWindow::settings20ButtonToggled(bool){
    setNewButtonSetting(20);
    untoggleButtons();
}

