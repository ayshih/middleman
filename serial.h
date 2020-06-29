#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <poll.h>

#define DEVICE_STUB "/dev/ttyS"
#define BAUD_RATE B230400

int setup_serial_port(int device_number);
ssize_t polled_read(int fd, struct pollfd *serial_poll, void *buf, size_t count);
ssize_t polled_write(int fd, struct pollfd *serial_poll, const void *buf, size_t count);

#endif
