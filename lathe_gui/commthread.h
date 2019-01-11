#ifndef COMMTHREAD_H
#define COMMTHREAD_H

#include <QObject>
#include <QMutex>
#include <QThread>

#define LOCAL_SOCKET

#ifdef LOCAL_SOCKET
#include <QtNetwork>
#endif  // #ifdef LOCAL_SOCKET

struct lathe_status_packet {
        int32_t current_run_mode;
        int32_t absolute_tick;
        int32_t cycle_counter;
        int32_t absolute_pos;
        int32_t absolute_idx;
        int32_t absolute_cnt;
        int32_t current_index_delta;
        int32_t stepper_actual_pos_z;
        int32_t stepper_actual_pos_x;
        int32_t stepper_actual_pos_d;
        int32_t cycle_index;
        int32_t cycle_index_count;
};

class CommThread : public QObject
{
    Q_OBJECT
public:
    explicit CommThread(QObject *parent = nullptr);
    virtual ~CommThread();
    bool keepRunning;

    QMutex mutex;
    lathe_status_packet status_packet;

    double newzfeed;
    double newxfeed;

    bool setzero;
    bool setfollow;
    bool idle;
    bool stop;

    bool cpause;
    bool cstart;
    bool creset;
    bool cclear;

#ifdef LOCAL_SOCKET
    QTcpSocket *socket;
#else  // #ifdef LOCAL_SOCKET
    int fd;
#endif   // #ifdef LOCAL_SOCKET

    std::vector<std::string> code;

signals:
    void update();

public slots:
    void run();
    void newfeed();

private:
    std::string read_response(bool &bad_crc);
    void send_command(std::string &cmd);
};

#endif // COMMTHREAD_H
