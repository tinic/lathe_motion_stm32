#ifndef COMMTHREAD_H
#define COMMTHREAD_H

#include <QObject>
#include <QMutex>
#include <QThread>

struct lathe_status_packet {
        int64_t absolute_pos;
        int64_t stepper_actual_pos_z;
        int64_t stepper_actual_pos_x;
        int32_t absolute_cnt;
        int32_t absolute_idx;
        int32_t current_index_delta;
        int32_t absolute_pos_start_offset;
        int32_t stepper_follow_mul_z;
        int32_t stepper_follow_div_z;
        int32_t stepper_follow_mul_x;
        int32_t stepper_follow_div_x;
        int32_t error_state_out_of_sync;
        int32_t error_index_offset;
        int32_t absolute_tick;
};

class CommThread : public QObject
{
    Q_OBJECT
public:
    explicit CommThread(QObject *parent = nullptr);
    bool keepRunning;

    QMutex statusMutex;
    lathe_status_packet status_packet;

    double newzfeed;
    double newxfeed;
    bool setzero;

signals:
    void update();

public slots:
    void run();
    void newfeed();

private:
    std::string read_response(int fd, bool &bad_crc);
    void send_command(int fd, std::string &cmd);
};

#endif // COMMTHREAD_H
