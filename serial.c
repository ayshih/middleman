#include <fcntl.h>  // open()
#include <stdio.h>  // perror()
#include <stdlib.h>  // exit()
#include <string.h>  // memset()
#include <termios.h>
#include <unistd.h>  // read(), write()

#include "serial.h"

int setup_serial_port(int device_number, speed_t baud_rate)
{
    char device[12];
    sprintf(device, "%s%d", DEVICE_STUB, device_number);

    int fd = -1;
    fd = open(device, O_RDWR | O_NONBLOCK);
    //fd = open(device, O_RDWR);

    if (fd < 0) {
        perror("Error opening serial port");
        exit(1);
    }

    struct termios newtio;
    memset(&newtio, 0, sizeof(newtio)); /* clear struct for new port settings */

    /* man termios get more info on below settings */
    newtio.c_cflag = baud_rate | CS8 | CLOCAL | CREAD;  // should use cfsetspeed() instead

    newtio.c_iflag = 0;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;

    // block for up till 4 characters
    newtio.c_cc[VMIN] = 4;

    // 0.5 seconds read timeout
    newtio.c_cc[VTIME] = 5;

    /* now clean the modem line and activate the settings for the port */
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd,TCSANOW,&newtio);

    return fd;
}

ssize_t polled_read(int fd, struct pollfd *serial_poll, void *buf, size_t count)
{
    int retval = poll(serial_poll, 1, 1000);
    if(retval == -1) {
        perror("poll()");
        return -1;
    } else if(retval && (serial_poll->revents & POLLIN)) {
        return read(fd, buf, count);
    }
    return 0;
}

ssize_t polled_write(int fd, struct pollfd *serial_poll, const void *buf, size_t count)
{
    int retval = poll(serial_poll, 1, 1000);
    if(retval == -1) {
        perror("poll()");
        return -1;
    } else if(retval && (serial_poll->revents & POLLOUT)) {
        return write(fd, buf, count);
    }
    return 0;
}
