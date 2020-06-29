#include <stdio.h>
#include <stdlib.h>  // atoi()
#include <string.h>  // memset()
#include <unistd.h>  // usleep(), write()

#include "serial.h"
#include "ring.hpp"

int main(int argc, char * argv[])
{
    if(argc != 2) {
        printf("Usage: %s <device number>\n", argv[0]);
        return -1;
    }

    int fd = setup_serial_port(atoi(argv[1]));

    char buffer1[1024], buffer2[20];
    RingBuffer ring_buffer;

    int last_number = -1;
    while(1) {
        int c = read(fd, &buffer1, sizeof(buffer1));

        ring_buffer.append(buffer1, c);

	int packet_size;
        while((packet_size = ring_buffer.smart_pop(buffer2)) != 0) {
            if(packet_size == -1) {
                fprintf(stderr, "Skipping a byte\n");
                continue;
            }

            int number;
            memcpy(&number, buffer2 + 2, sizeof(number));

            //printf("%d %d\n", packet_size, number);

            if(packet_size == 7) {
                if(number != last_number + 1) {
                    fprintf(stderr, "Desync with %d dropped packets (%d, %d)\n", number - last_number - 1, last_number, number);
                }
                last_number = number;
            }
        }
    }

    return 0;
}
