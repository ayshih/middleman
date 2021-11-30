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
        printf("Usage: %s <device number> <file>\n", argv[0]);
        return -1;
    }

    int fd = setup_serial_port(atoi(argv[1]));
    FILE *fp = fopen(argv[2], "r");

    if (fp == NULL) return -1;

    struct pollfd serial_poll;
    serial_poll.fd = fd;
    serial_poll.events = POLLOUT;

    int ch;
    while ((ch = fgetc(fp)) != EOF) {
	uint8_t byte = ch;
	if (byte == 0xac) printf("\n%ld:", ftell(fp));
	printf("%02x ", byte);
        polled_write(fd, &serial_poll, &byte, 1);
    }

    fclose(fp);

    return 0;
}
