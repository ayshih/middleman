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

    int fd = setup_serial_port(atoi(argv[1]));
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
        buffer[0] = 0b10101100;
        buffer[1] = 0;
        memcpy(buffer + 2, &count, sizeof(count));

        // Mock event packet
	char id = 0b000;
        buffer[0] = 0b10101100 | (id >> 1);
        buffer[1] = (id & 0b001) << 7;
        ssize_t c = polled_write(fd, &serial_poll, buffer, 7);
        if(c != 7) perror("uh oh");

        // TODO: check whether all of the bytes were written

        // Mock deadtime packet (1 Hz)
        if(count % events_per_sec == 0) {
	    id = 0b111;
            buffer[0] = 0b10101100 | (id >> 1);
            buffer[1] = (id & 0b001) << 7;
            ssize_t c = polled_write(fd, &serial_poll, buffer, 10);
        }

        // Mock housekeeping packet (0.1 Hz)
        if(count % (events_per_sec * 10) == 0) {
	    id = 0b110;
            buffer[0] = 0b10101100 | (id >> 1);
            buffer[1] = (id & 0b001) << 7;
            ssize_t c = polled_write(fd, &serial_poll, buffer, 18);
        }

        count++;

        printf("\b%c", spinner[count % 4]);

        usleep(1000000 / events_per_sec);
    }

    return 0;
}
