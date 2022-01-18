#include <stdio.h>
#include <stdint.h>
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

    int fd = setup_serial_port(atoi(argv[1]), B19200);
    int events_per_sec = atoi(argv[2]);

    char buffer[52];

    char spinner[] = "|/-\\";
    setbuf(stdout, NULL);
    printf(" ");

    struct pollfd serial_poll;
    serial_poll.fd = fd;
    serial_poll.events = POLLOUT;

    uint32_t count = 12345678;
    while(1) {
        memset(buffer, 0, 52);

        buffer[0] = 0xEB;
        buffer[1] = 0x90;
        uint8_t *count_as_bytes = (uint8_t *)&count;
        buffer[2] = *(count_as_bytes + 3);
        buffer[3] = *(count_as_bytes + 2);
        buffer[4] = *(count_as_bytes + 1);
        buffer[5] = *(count_as_bytes);

        ssize_t c = polled_write(fd, &serial_poll, buffer, 52);
        if(c != 52) perror("uh oh");

        count++;

        printf("\b%c", spinner[count % 4]);

        usleep(1000000 / events_per_sec);
    }

    return 0;
}
