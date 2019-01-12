#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "gcode.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <QLabel>
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>
#include <QTextStream>
#include <QPlainTextEdit>
#include <QTableView>
#include <QStandardItemModel>

enum run_mode {
    run_mode_idle,
    run_mode_none,

    run_mode_follow_z,
    run_mode_follow_x,
    run_mode_follow_d,
    run_mode_follow_zxd,

    run_mode_cycle,
    run_mode_cycle_pause
};

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
    findChild<QPushButton *>("zButton")->setChecked(true);
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
    commThread.mutex.lock();
    memcpy(&local_packet, &commThread.status_packet, sizeof(lathe_status_packet));
    commThread.mutex.unlock();

    double rpm = local_packet.current_index_delta ? (60000.0 / double(local_packet.current_index_delta)) : 0.0;
    if (rpm < 60) rpm = 0;

    char str[256] = {0};
    sprintf(str, "POSZ: %c%07.02f%s\n"
                 "POSX: %c%07.02f%s\n"
                 "POSD: %c%07.02f%s\n"
                 "POSA: %c %06.02f%s\n"
                 "RPM :  %07.02f\n",
            local_packet.stepper_actual_pos_z < 0 ? '-' : '+',
            ((fabs(double(local_packet.stepper_actual_pos_z)) / 3200.0) / 12.0) * 25.4,
            "mm",
            local_packet.stepper_actual_pos_x < 0 ? '-' : '+',
            ((fabs(double(local_packet.stepper_actual_pos_x)) / 3200.0) / 10.0) * 25.4,
            "mm",
            local_packet.stepper_actual_pos_d < 0 ? '-' : '+',
            ((fabs(double(local_packet.stepper_actual_pos_d)) / 3200.0) / 12.0) * 25.4,
            "mm",
            local_packet.absolute_cnt < 0 ? '-' : '+',
            ( fabs(double(local_packet.absolute_cnt)) / 2880.0) * 360,
            "°",
            rpm);

   QLabel *statusLabel = findChild<QLabel *>("statusLabel");
   statusLabel->setText(str);

   QLabel *rawStatusLabel = findChild<QLabel *>("rawStatusLabel");

   sprintf(str,"Mode:%02x "
               "Tick:%04x "
               "Cycles:%04x "

               "Pos:%c%08x "
               "Index:%c%08x "
               "Count:%c%04x "
               "Delta:%04x "

               "ZPos:%c%08x "
               "XPos:%c%08x "
               "DPos:%c%08x "

               "CycleI:%04x "
               "CycleL:%04x ",

           local_packet.current_run_mode,
           local_packet.absolute_tick,
           local_packet.cycle_counter,

           local_packet.absolute_pos >=0 ? '+' : '-',
           abs(local_packet.absolute_pos),
           local_packet.absolute_idx >=0 ? '+' : '-',
           abs(local_packet.absolute_idx),
           local_packet.absolute_cnt >=0 ? '+' : '-',
           abs(local_packet.absolute_cnt),

           local_packet.current_index_delta,

           local_packet.stepper_actual_pos_z >=0 ? '+' : '-',
           abs(local_packet.stepper_actual_pos_z),
           local_packet.stepper_actual_pos_x >=0 ? '+' : '-',
           abs(local_packet.stepper_actual_pos_x),
           local_packet.stepper_actual_pos_d >=0 ? '+' : '-',
           abs(local_packet.stepper_actual_pos_d),

           local_packet.cycle_index,
           local_packet.cycle_index_count);

    QTableView *progTableView = findChild<QTableView *>("progTableView");

    rawStatusLabel->setText(str);

    QPushButton *clearButton = findChild<QPushButton *>("clearButton");
    QPushButton *resetButton = findChild<QPushButton *>("resetButton");
    QPushButton *pauseButton = findChild<QPushButton *>("pauseButton");

    QGridLayout *gridLayoutConversational0 = findChild<QGridLayout *>("gridLayoutConversational0");
    QGridLayout *gridLayoutConversational1 = findChild<QGridLayout *>("gridLayoutConversational1");

    if (local_packet.current_run_mode == run_mode_cycle_pause ||
        local_packet.current_run_mode == run_mode_cycle) {

        clearButton->setEnabled(true);
        resetButton->setEnabled(true);
        pauseButton->setEnabled(true);

        QList<QPushButton *> buttons0 = gridLayoutConversational0->findChildren<QPushButton *>();
        QList<QPushButton *>::iterator i0;
        for (i0 = buttons0.begin(); i0 != buttons0.end(); i0++) {
            (*i0)->setDisabled(true);
        }

        QList<QPushButton *> buttons1 = gridLayoutConversational1->findChildren<QPushButton *>();
        QList<QPushButton *>::iterator i1;
        for (i1 = buttons1.begin(); i1 != buttons1.end(); i1++) {
            (*i1)->setDisabled(true);
        }

        if (local_packet.current_run_mode == run_mode_cycle_pause) {
            pauseButton->setText("RESUME");
        } else {

            int32_t row = int32_t(parser.cycle_index_to_gcode(size_t(local_packet.cycle_index+1)));
            progTableView->selectRow(row);
            progTableView->scrollTo(progTableView->currentIndex(), QAbstractItemView::PositionAtCenter);

            pauseButton->setText("PAUSE");
        }

    } else {
        clearButton->setDisabled(true);
        resetButton->setDisabled(true);
        pauseButton->setDisabled(true);

        QList<QPushButton *> buttons0 = gridLayoutConversational0->findChildren<QPushButton *>();
        QList<QPushButton *>::iterator i0;
        for (i0 = buttons0.begin(); i0 != buttons0.end(); i0++) {
            (*i0)->setEnabled(true);
        }

        QList<QPushButton *> buttons1 = gridLayoutConversational1->findChildren<QPushButton *>();
        QList<QPushButton *>::iterator i1;
        for (i1 = buttons1.begin(); i1 != buttons1.end(); i1++) {
            (*i1)->setEnabled(true);
        }
    }
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
      1.4876, 1.5072, 1.4933, 1.4894,
      1.789910608, 4.5, 5.625, 7.50,
      10, 12.5, 15, 20,
      25, 30, 35, 45,
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
    QPushButton *button0 = findChild<QPushButton *>("aButton");
    QPushButton *button1 = findChild<QPushButton *>("xButton");
    QPushButton *button2 = findChild<QPushButton *>("zButton");
    commThread.mutex.lock();
    if (button2->isChecked()) {
        commThread.newzfeed = currentStepValue * currentDirectionValue;
        commThread.newxfeed = 0.0;
        commThread.setfollow = true;
    } else if (button1->isChecked()) {
        commThread.newxfeed = currentStepValue * currentDirectionValue;
        commThread.newzfeed = 0.0;
        commThread.setfollow = true;
    } else if (button0->isChecked()) {
        double t = -tan((M_PI/180)*currentAngleValue);
        commThread.newzfeed = currentStepValue * currentDirectionValue;
        commThread.newxfeed = currentStepValue * currentDirectionValue * t;
        commThread.setfollow = true;
    }
    commThread.mutex.unlock();
}

GCodeListModel::GCodeListModel(const GCodeParser &parser, QObject* parent):
    parser(parser)
{
}

int GCodeListModel::rowCount(const QModelIndex &parent) const {
    return parser.g_code().size();
}

int GCodeListModel::columnCount(const QModelIndex &parent) const {
    return 2;
}

QVariant GCodeListModel::data(const QModelIndex &index, int role) const {
    if (!index.isValid()) return QVariant();
    if (index.row() >= parser.g_code().size()) return QVariant();
    if (role == Qt::DisplayRole) {
        if (index.column() == 0 ) {
            return QVariant(QString::number(index.row()));
        } else if (index.column() == 1) {
            return QVariant(QString::fromStdString(parser.g_code()[index.row()]));
        } else {
            return QVariant();
        }
    } else {
        return QVariant();
    }
}

void MainWindow::progLoadClicked()
{
    QTableView *progTableView = findChild<QTableView *>("progTableView");

    QString fileName = QFileDialog::getOpenFileName(nullptr,
        tr("Open Command File"), "",
        tr("NC File (*.nc);;All Files (*)"));

    if (fileName.length()>0) {
        std::ifstream t(fileName.toStdString());
        parser.loadAndParse(t);

        QStandardItemModel *model = new QStandardItemModel(parser.g_code().size(), 1);

        for(size_t c=0; c<parser.g_code().size();c++) {
            QStandardItem *col1 = new QStandardItem(QString::fromStdString(parser.g_code()[c]));
            if ((c&1) == 0) {
                col1->setBackground(QBrush(QColor(32,32,32)));
            } else {
                col1->setBackground(QBrush(QColor(64,64,64)));
            }
            model->setItem(c,0,col1);
        }

        model->setHeaderData(0, Qt::Horizontal, QObject::tr("CODE"));

        progTableView->setModel(model);

        progTableView->setShowGrid(false);

        progTableView->horizontalHeader()->setStretchLastSection(true);
        progTableView->horizontalHeader()->setStyleSheet("background: white; color: black");
        progTableView->verticalHeader()->setStyleSheet("background: white; color: black");

        progTableView->update();
        progTableView->show();

        progTableView->setEditTriggers(QAbstractItemView::NoEditTriggers);

        int32_t row = int32_t(parser.cycle_index_to_gcode(size_t(1)));
        progTableView->selectRow(row);
        progTableView->scrollTo(progTableView->currentIndex(), QAbstractItemView::PositionAtCenter);

        commThread.mutex.lock();
        commThread.code = parser.intermediate_code();
        commThread.mutex.unlock();
    }
}

void MainWindow::progPauseClicked()
{
    commThread.mutex.lock();
    commThread.cpause = true;
    commThread.mutex.unlock();
}

void MainWindow::progClearClicked()
{
    QTableView *progTableView = findChild<QTableView *>("progTableView");

    progTableView->model()->removeRows(0,progTableView->model()->rowCount());

    commThread.mutex.lock();
    commThread.cclear = true;
    commThread.mutex.unlock();
}

void MainWindow::progResetClicked()
{
    commThread.mutex.lock();
    commThread.creset = true;
    commThread.mutex.unlock();
}

void MainWindow::zeroButtonClicked()
{
    commThread.mutex.lock();
    commThread.setzero = true;
    commThread.setfollow = true;
    commThread.mutex.unlock();
}

void MainWindow::stopButtonToggled(bool)
{
    QPushButton *button0 = findChild<QPushButton *>("idleButton");
    QPushButton *button1 = findChild<QPushButton *>("stopButton");
    if (!button1->isChecked()) button0->setChecked(false);
    commThread.mutex.lock();
    commThread.idle = button0->isChecked();
    commThread.stop = button1->isChecked();
    commThread.setfollow = true;
    commThread.mutex.unlock();
}

void MainWindow::idleButtonToggled(bool)
{
    QPushButton *button0 = findChild<QPushButton *>("idleButton");
    QPushButton *button1 = findChild<QPushButton *>("stopButton");
    if (button0->isChecked()) button1->setChecked(true);
    commThread.mutex.lock();
    commThread.idle = button0->isChecked();
    commThread.stop = button1->isChecked();
    commThread.setfollow = true;
    commThread.mutex.unlock();
}

void MainWindow::aButtonToggled(bool) {
    QPushButton *button0 = findChild<QPushButton *>("aButton");
    QPushButton *button1 = findChild<QPushButton *>("xButton");
    QPushButton *button2 = findChild<QPushButton *>("zButton");
    if (button0) button0->setChecked(true);
    if (button1) button1->setChecked(false);
    if (button2) button2->setChecked(false);
    passToCommThread();
    directionButtonToggled(false);
}

void MainWindow::xButtonToggled(bool) {
    QPushButton *button0 = findChild<QPushButton *>("aButton");
    QPushButton *button1 = findChild<QPushButton *>("xButton");
    QPushButton *button2 = findChild<QPushButton *>("zButton");
    if (button0) button0->setChecked(false);
    if (button1) button1->setChecked(true);
    if (button2) button2->setChecked(false);
    passToCommThread();
    directionButtonToggled(false);
}

void MainWindow::zButtonToggled(bool) {
    QPushButton *button0 = findChild<QPushButton *>("aButton");
    QPushButton *button1 = findChild<QPushButton *>("xButton");
    QPushButton *button2 = findChild<QPushButton *>("zButton");
    if (button0) button0->setChecked(false);
    if (button1) button1->setChecked(false);
    if (button2) button2->setChecked(true);
    passToCommThread();
    directionButtonToggled(false);
}

void MainWindow::directionButtonToggled(bool) {
    QPushButton *button0 = findChild<QPushButton *>("aButton");
    QPushButton *button1 = findChild<QPushButton *>("xButton");
    QPushButton *button2 = findChild<QPushButton *>("zButton");
    QPushButton *button3 = findChild<QPushButton *>("directionButton");

    if (button3->isChecked()) {
        if (button2->isChecked()) button3->setText("⮆");
        if (button1->isChecked()) button3->setText("⮇");
        if (button0->isChecked()) button3->setText("⇗");
        currentDirectionValue = -1.0;
    } else {
        if (button2->isChecked()) button3->setText("⮄");
        if (button1->isChecked()) button3->setText("⮅");
        if (button0->isChecked()) button3->setText("⇙");
        currentDirectionValue = +1.0;
    }

    passToCommThread();
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

