#include <stdio.h>
#include <stdlib.h>  // atoi()
#include <string.h>  // memset()
#include <unistd.h>  // usleep(), write()
#include <poll.h>

#include "serial.h"

int main(int argc, char * argv[])
{
    if(argc != 3) {
        printf("Usage: %s <device number> <events per second>\n", argv[0]);
        return -1;
    }

    int fd = setup_serial_port(atoi(argv[1]), B1200);
    int events_per_sec = atoi(argv[2]);

    char buffer[20];
    memset(buffer, 0, 20);

    char spinner[] = "|/-\\";
    setbuf(stdout, NULL);
    printf(" ");

    struct pollfd serial_poll;
    serial_poll.fd = fd;
    serial_poll.events = POLLOUT;

    int count = 0;
    while(1) {
        // Send request-science-data packet
        buffer[0] = 0x10;
        buffer[1] = 0x13;
        buffer[2] = 0x03;

        ssize_t c = polled_write(fd, &serial_poll, buffer, 3);

        count++;

        printf("\b%c", spinner[count % 4]);

        usleep(1000000 / events_per_sec);
    }

    return 0;
}
