#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>

#include "serial.h"

// read the current level from CTS pin
int get_cts_state(int fd)
{
    int serial = 0;
    if(ioctl(fd, TIOCMGET, &serial) < 0)
    {
        printf("ioctl() failed: %d: %s\n", errno, strerror(errno));
        return -1;
    }
  
    return (serial & TIOCM_CTS) ? 1 : 0;
}
  
//  until CTS state changes
int main(int argc, char** argv)
{
    if(argc != 2) {
        printf("Usage: %s <device number>\n", argv[0]);
        return -1;
    }

    int fd = setup_serial_port(atoi(argv[1]));

    printf("Device opened, CTS state: %d\n", get_cts_state(fd));
  
    // detect CTS changes forever
    int i=0;
    while(1)
    {
        printf("%6d CTS state: %d\n", i++, get_cts_state(fd));
  
        // block until line changes state
        if(ioctl(fd, TIOCMIWAIT, TIOCM_CTS) < 0)
        {
            printf("ioctl(TIOCMIWAIT) failed: %d: %s\n", errno, strerror(errno));
            return -1;
        }
    }
}
