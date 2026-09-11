#include <cerrno>
#include <cstddef>
#include <cstdio>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <stdexcept>
#include <string>
#include <iostream>

//注意了：SerialSendData在多个线程调用时要加锁
//SerialSendData和SerialWaitForRecvSize可以在不同的两个线程调用，它们查的是 fd 表项里两个不同的标志位，不冲突
class LowLevelSerialPort {
    public:
        enum{
            ANS_OK        = 0,
            ANS_TIMEOUT   = -1,
            ANS_DEV_ERR   = -2,
            ANS_PARAM_ERR = -3,
        };

        LowLevelSerialPort(int baudrate, const char * tty) : baudrate_(baudrate) {
            if (SerialOpen(tty) < 0) {
                throw std::runtime_error("serial open failed");
                std::cout << "serial open failed" << std::endl;
            }
        }

        // 只记录波特率, 不打开串口；端口路径在调用方读到配置后再用 Open() 打开
        explicit LowLevelSerialPort(int baudrate) : baudrate_(baudrate) {}

        ~LowLevelSerialPort() {
            SerialClose();
        }

        int baudrate_ = 115200;
        std::string tty_;

    private:
        int fd_ = -1; //file description

        int SerialOpen(const char * tty) {
            /* open serial port */
            fd_ = ::open(tty, O_RDWR | O_NOCTTY | O_NONBLOCK);
            if(fd_ < 0) {
                ::perror("open");
                return fd_;
            }

            /* config serial port */
            struct termios tio;
            ::tcgetattr(fd_, &tio);

            ::cfsetispeed(&tio, B115200); /* set input baurdrate */
            ::cfsetospeed(&tio, B115200); /* set output baurdrate */
            tio.c_cflag |= CLOCAL | CREAD;   /* 忽略modem + 启用接收 */
            tio.c_cflag |= CS8;              /* 8数据位 */
            tio.c_cflag &= ~CSTOPB;          /* 1停止位 */
            tio.c_cflag &= ~PARENB;          /* 无校验 */
            tio.c_cflag &= ~CRTSCTS;         /* 无硬件流控 */
            tio.c_iflag = 0;           
            tio.c_oflag = 0;              
            tio.c_lflag = 0;                 
            tio.c_cc[VMIN]  = 0; /* no block */   
            tio.c_cc[VTIME] = 0;

            if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
                ::close(fd_);
                return -1;
            }
            ::tcflush(fd_, TCIOFLUSH);
            tty_ = tty;

            return fd_;
        }

#define TIMEOUT_FOREVER 0xffffffff
        static struct timespec DeadLineFromNow(unsigned int timeout_ms) {
            //clac deadline
            struct timespec deadline;
            clock_gettime(CLOCK_MONOTONIC, &deadline);
            deadline.tv_sec  += timeout_ms / 1000;
            deadline.tv_nsec += (timeout_ms % 1000) * 1000000;
            if (deadline.tv_nsec >= 1000000000) {
                deadline.tv_sec += 1;
                deadline.tv_nsec -= 1000000000;
            }
            return deadline;
        } 

        static int GetRemainMS(struct timespec deadline) {
            int remain;
            struct timespec now;

            clock_gettime(CLOCK_MONOTONIC, &now);
            if (now.tv_sec > deadline.tv_sec ||
                (now.tv_sec == deadline.tv_sec 
                && now.tv_nsec >= deadline.tv_nsec)
                )
                return 0;

            remain = (deadline.tv_sec - now.tv_sec) * 1000 +
                        (deadline.tv_nsec - now.tv_nsec) / 1000000;

            return remain;
        }

        //fd_必须是非阻塞模式才能调用SerialSendData
        int SerialSendData(const unsigned char * data, size_t size, unsigned int timeout_ms = TIMEOUT_FOREVER) {
            if (data == NULL || size == 0) return 0;
            
            struct timespec deadline;
            if (timeout_ms != TIMEOUT_FOREVER) {
                deadline = DeadLineFromNow(timeout_ms);
            }

            size_t tx_len = 0; 
            int remain;
            while (tx_len < size) {
                int ans = ::write(fd_, data + tx_len, size - tx_len);
                
                if (ans > 0) { 
                    tx_len += ans; 
                } else if (ans == -1) { //kernel buffer is full or other(EAGAIN or EIO or EBADF or ENXIO)
                    if (errno != EAGAIN) {
                        return -1; //其他错误
                    }

                    //每轮检查剩余超时时间
                    if (timeout_ms != TIMEOUT_FOREVER) {
                        remain = GetRemainMS(deadline);
                        if (remain == 0)
                        return static_cast<int>(tx_len);
                    } else {
                        remain = -1;
                    }
                    
                    //wait until write buffer is ready
                    struct pollfd poll_fd;
                    poll_fd.fd = fd_;
                    poll_fd.events = POLLOUT;
                    poll_fd.revents = 0;
                    int ret = ::poll(&poll_fd, 1, remain);
                    if (ret < 0) {
                        ::perror("poll");
                        return -1;
                    }
                    
                    if (!(poll_fd.revents & POLLOUT)) { //buffer is ready to be writtem
                        //what fuck happened!
                        break;
                    }
                } else {
                    return static_cast<int>(tx_len);
                }
            }
            
            return static_cast<int>(tx_len);
        }

        //等内核缓冲区攒到count个字节，超时返回
        int SerialWaitForRecvSize(int count, int * returned_size, unsigned int timeout_ms = TIMEOUT_FOREVER) {
            if (!returned_size) return ANS_PARAM_ERR;

            struct timespec deadline;
            if (timeout_ms != TIMEOUT_FOREVER) {
                deadline = DeadLineFromNow(timeout_ms);
            }

            int avaliable_size;
            
            if(-1 == ::ioctl(fd_, FIONREAD, &avaliable_size)) {
                * returned_size = 0;
                return ANS_DEV_ERR;
            }
            * returned_size = avaliable_size;
            if (avaliable_size >= count) {
                return ANS_OK;
            }

            int remain;
            while (* returned_size < count) {
                //每轮检查剩余超时时间
                if (timeout_ms != TIMEOUT_FOREVER) {
                    remain = GetRemainMS(deadline);
                    if (remain == 0)
                        return ANS_TIMEOUT;
                } else {
                    remain = -1;
                }

                struct pollfd poll_fd;
                poll_fd.fd = fd_;
                poll_fd.events = POLLIN;
                poll_fd.revents = 0;

                int ret = ::poll(&poll_fd, 1, remain);
                if (ret < 0) {
                    if (errno == EINTR) continue;    // 信号打断
                    return ANS_DEV_ERR;
                }
                if (0 == ret)                        // poll自身超时
                    return ANS_TIMEOUT;

                if (!(poll_fd.revents & POLLIN))
                    return ANS_DEV_ERR;

                if (-1 == ::ioctl(fd_, FIONREAD, &avaliable_size)) {
                    * returned_size = 0;
                    return ANS_DEV_ERR;
                }
                * returned_size = avaliable_size;
            }
            return ANS_OK;
        }

        int SerialRecvData(unsigned char * data, size_t size) {
            if (data == NULL || size ==0) return 0;
            
            int ans = ::read(fd_, data, size);
            if (ans > 0) {
                return ans;
            } else if (ans == -1) {
                if (errno != EAGAIN) {
                    return ANS_DEV_ERR;
                }
            } else {
                return ANS_TIMEOUT;
            }
            return 0; //never reach here
        }

        int SerialClose() {
            if (fd_ != -1) ::close(fd_);
            fd_ = -1;

            return 0;
        }

    public:
        int GetFd() { return fd_; }
        int Open(const char * tty)            { return SerialOpen(tty); }
        int Send(const unsigned char * d, size_t n, unsigned int t = 0xffffffff) {
            return SerialSendData(d, n, t); }
        int WaitRecvSize(int count, int * returned_size, unsigned int timeout_ms = TIMEOUT_FOREVER) {
            return SerialWaitForRecvSize(count, returned_size, timeout_ms);
        }
        int Recv(unsigned char * data, size_t size) {
            return SerialRecvData(data, size);
        }
};

