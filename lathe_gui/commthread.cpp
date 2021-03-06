#include "commthread.h"

#include <math.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <sstream>
#include <iomanip>


static const char int2hex[]= {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

static uint16_t CRC16(const uint8_t *data, size_t len)
{
    static const uint16_t crcTable[] = {
        0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
        0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
        0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
        0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
        0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
        0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
        0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
        0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
        0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
        0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
        0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
        0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
        0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
        0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
        0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
        0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
        0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
        0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
        0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
        0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
        0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
        0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
        0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
        0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
        0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
        0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
        0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
        0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
        0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
        0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
        0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
        0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
    };
    uint16_t crcWord = 0xFFFF;
    while (len--) {
        uint8_t d = *data++ ^ uint8_t(crcWord);
        crcWord >>= 8;
        crcWord ^= crcTable[d];
    }
    return crcWord;
}

CommThread::CommThread(QObject *parent) : QObject(parent), keepRunning(true), mutex(QMutex::Recursive),
    setzero(false),
    setfollow(false),
    idle(false),
    stop(false)
{
}

CommThread::~CommThread() {
#ifdef LOCAL_SOCKET
    socket->close();
#endif  // #ifdef LOCAL_SOCKET
}

std::string CommThread::read_response(bool &bad_crc)
{
    std::string result;
    char ch = 0;
#ifdef LOCAL_SOCKET
    int32_t timeout = 1000;
    do {
        ch = 0;
        socket->waitForReadyRead(1);
        if (socket->read(&ch, 1) == 1) {
            if (ch != '\n') {
                result.append(1, ch);
            }
        }
        if (!keepRunning || (--timeout == 0)) {
            bad_crc = true;
            return "";
        }
    } while (ch != '\n');
#else  // #ifdef LOCAL_SOCKET
    int32_t timeout = 10000;
    do {
        ch = 0;
        if (read(fd, &ch, 1) == 1) {
            if (ch != '\n') {
                result.append(1, ch);
            }
        } else {
            usleep(100);
        }
        usleep(100);
        if (!keepRunning || (--timeout == 0)) {
            bad_crc = true;
            return "";
        }
    } while (ch != '\n');
#endif  // #ifdef LOCAL_SOCKET

    if (result.size() < 4) {
        bad_crc = true;
    } else {
        uint16_t crc = CRC16(reinterpret_cast<const uint8_t *>(result.c_str()), result.size()-4);
        if ((char(int2hex[((crc>>12)&0xF)]) != result.c_str()[result.size()-4])||
            (char(int2hex[((crc>> 8)&0xF)]) != result.c_str()[result.size()-3])||
            (char(int2hex[((crc>> 4)&0xF)]) != result.c_str()[result.size()-2])||
            (char(int2hex[((crc>> 0)&0xF)]) != result.c_str()[result.size()-1])) {
            bad_crc = true;
        } else {
            bad_crc = false;
        }
    }
    return result;
}

void CommThread::send_command(std::string &cmd) {
    uint16_t crc = CRC16(reinterpret_cast<const uint8_t *>(cmd.data()), cmd.size());

    cmd.append(1,char(int2hex[((crc>>12)&0xF)]));
    cmd.append(1,char(int2hex[((crc>> 8)&0xF)]));
    cmd.append(1,char(int2hex[((crc>> 4)&0xF)]));
    cmd.append(1,char(int2hex[((crc>> 0)&0xF)]));

    cmd.append(1,'\n');

#ifdef LOCAL_SOCKET
    socket->write(cmd.c_str(),  qint64(cmd.size()));
    socket->flush();
#else  // #ifdef LOCAL_SOCKET
    write (fd, cmd.c_str(), cmd.size());
#endif  //#ifdef LOCAL_SOCKET
}

void CommThread::run() {

#ifdef LOCAL_SOCKET

    socket = new QTcpSocket;
    socket->connectToHost(QHostAddress::LocalHost, 33333);
    socket->waitForConnected();

#else  // #ifdef LOCAL_SOCKET
    const char *portname = "/dev/ttyS1";
    int fd = open (portname, O_RDWR | O_NOCTTY);

    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0) {
        return;
    }

    cfsetospeed (&tty, B115200);
    cfsetispeed (&tty, B115200);

    cfmakeraw(&tty);

    tty.c_cflag &= uint32_t(~PARENB);
    tty.c_cflag &= uint32_t(~CSTOPB);
    tty.c_cflag &= uint32_t(~CSIZE);
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tcflush(fd, TCIFLUSH);

    if (tcsetattr (fd, TCSANOW, &tty) != 0) {
        return;
    }
#endif  // #ifdef LOCAL_SOCKET

    double oldzfeed = -1000.0;
    newzfeed = 0;
    double oldxfeed = -1000.0;
    newxfeed = 0;

    setfollow = false;
    cstart = false;
    cpause = false;
    creset = false;

    bool bad_crc = false;

    while (keepRunning) {
        mutex.lock();
        if (idle) {
            std::string cmd("H1");
            send_command(cmd);
            read_response(bad_crc);
        } else if (stop) {
            std::string cmd("H0");
            send_command(cmd);
            read_response(bad_crc);
        } else if (setfollow) {
            setfollow = false;
            if (newzfeed != 0.0 && newxfeed != 0.0) {
                std::string cmd("FB");
                send_command(cmd);
                read_response(bad_crc);
            } else if (newzfeed != 0.0) {
                std::string cmd("FZ");
                send_command(cmd);
                read_response(bad_crc);
            } else {
                std::string cmd("FX");
                send_command(cmd);
                read_response(bad_crc);
            }
        } else if (cpause) {
            std::string cmdCS("CP");
            send_command(cmdCS);
            read_response(bad_crc);
            cpause = false;
        } else if (creset) {
            std::string cmdCS("CR");
            send_command(cmdCS);
            read_response(bad_crc);
            creset = false;
        } else if (cclear) {
            std::string cmdCS("CC");
            send_command(cmdCS);
            read_response(bad_crc);
            cclear = false;
        }

        if (setzero) {
            setzero = false;
            std::string cmd("R");
            send_command(cmd);
            read_response(bad_crc);
        }

        if (fabs(newzfeed - oldzfeed) > 0.00001 ||
            fabs(newxfeed - oldxfeed) > 0.00001 ) {

            oldzfeed = newzfeed;
            oldxfeed = newxfeed;

            const double motor_steps_per_rev = 3200.0;
            const double encoder_steps_per_rev = 2880.0;
            const double lead_screw_tpi_z = 12.0;
            const double lead_screw_tpi_x = 10.0;

            int32_t mul_z = int32_t(-newzfeed * lead_screw_tpi_z * motor_steps_per_rev * 10.0);
            int32_t div_z = int32_t(25.4 * encoder_steps_per_rev * 10.0);

            int32_t mul_x = int32_t( newxfeed * lead_screw_tpi_x * motor_steps_per_rev * 10.0);
            int32_t div_x = int32_t(25.4 * encoder_steps_per_rev * 10.0);

            for (int32_t c = 0; c < 2; c++) {
                std::stringstream ss;

                ss << (c==0 ? 'Z' : 'X');

                ss << std::hex;
                ss << std::right;
                ss << std::uppercase;
                ss << std::setfill('0');

                ss << std::setw(8);
                ss << uint32_t(c==0 ? mul_z : mul_x);
                ss << std::setw(8);
                ss << uint32_t(c==0 ? div_z : div_x);

                std::string cmd(ss.str());

                send_command(cmd);
                read_response(bad_crc);
            }

        }

        if (code.size()>0) {
            std::string cmdCC("CC");
            send_command(cmdCC);
            read_response(bad_crc);
            for (auto cmd : code) {
                qDebug("%s", cmd.c_str());
                send_command(cmd);
                std::string response = read_response(bad_crc);
                qDebug("%s", response.c_str());
            }
            std::string cmdCR("CR");
            send_command(cmdCR);
            read_response(bad_crc);
            code.clear();
        }

        std::string cmd("S");
        send_command(cmd);

        std::string response = read_response(bad_crc);

        if ( bad_crc ) {
            mutex.unlock();
            continue;
        }

        size_t pos = 0;
        status_packet.current_run_mode = int32_t(std::stoull(response.substr(pos,2), nullptr, 16)); pos += 2;

        status_packet.absolute_tick = int32_t(std::stoul(response.substr(pos,4), nullptr, 16)); pos += 4;
        status_packet.cycle_counter = int32_t(std::stoul(response.substr(pos,4), nullptr, 16)); pos += 4;

        status_packet.absolute_pos = int32_t(std::stoull(response.substr(pos,8), nullptr, 16)); pos += 8;
        status_packet.absolute_idx = int32_t(std::stoul(response.substr(pos,8), nullptr, 16)); pos += 8;
        status_packet.absolute_cnt = int32_t(std::stoul(response.substr(pos,4), nullptr, 16)); pos += 4;
        status_packet.current_index_delta = int32_t(std::stoul(response.substr(pos,4), nullptr, 16)); pos += 4;

        status_packet.stepper_actual_pos_z = int32_t(std::stoull(response.substr(pos,8), nullptr, 16)); pos += 8;
        status_packet.stepper_actual_pos_x = int32_t(std::stoull(response.substr(pos,8), nullptr, 16)); pos += 8;
        status_packet.stepper_actual_pos_d = int32_t(std::stoull(response.substr(pos,8), nullptr, 16)); pos += 8;

        status_packet.cycle_index = int32_t(std::stoul(response.substr(pos,4), nullptr, 16)); pos += 4;
        status_packet.cycle_index_count = int32_t(std::stoul(response.substr(pos,4), nullptr, 16)); pos += 84;

        mutex.unlock();

        emit update();

        usleep(16666);
    }
}

void CommThread::newfeed() {
}
