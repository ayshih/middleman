#include <fstream>
#include <iostream>
#include <stdio.h>

#include "Telemetry.hpp"
#include "UDPReceiver.hpp"

#define PORT_TM 60501

int main(int argc, char *argv[])
{
    TelemetryReceiver tm_receiver = TelemetryReceiver(PORT_TM);
    tm_receiver.init_connection();

    char timestamp[14];
    char filename[128];
    std::ofstream log;

    time_t now;
    time(&now);
    struct tm *now_tm;
    now_tm = gmtime(&now);
    strftime(timestamp, 14, "%y%m%d_%H%M%S", now_tm);

    sprintf(filename, "tm_log_%s.bin", timestamp);
    filename[128 - 1] = '\0';
    printf("Creating telemetry log file %s\n", filename);
    log.open(filename, std::ofstream::binary);

    while(1){
        int packet_length = tm_receiver.listen();
        if( packet_length > 0){
            uint8_t *packet = new uint8_t[packet_length];
            tm_receiver.get_packet(packet);

            uint16_t reported_length = *((uint16_t *)(packet + 6));

            TelemetryPacket tp = TelemetryPacket(packet, packet_length);

            tp.readAtTo(6, reported_length);

            if (tp.valid()){
                log.write((char *)packet, packet_length);
                log.flush();
                if (reported_length+16 != packet_length) {
                    std::cout << "  Length mismatch: reported+16 is " << reported_length+16 << ", actual is " << packet_length << std::endl;
                } else {
                    // Print out the information from the imager-statistics packet
                    if (tp.getSystemID() == 0xA0 && tp.getTmType() == 0x0C) {
                        uint16_t imager_counts[8], buffer_bytes[8], bad_bytes[8];
                        for (int i = 0; i < 8; i++) {
                            tp >> imager_counts[i];
                            tp >> buffer_bytes[i];
                            tp >> bad_bytes[i];
                        }
                        printf("[%f s]", (uint64_t)tp.getSystemTime() / 1e7);
                        printf("\nImager counts:");
                        for (int i = 0; i < 8; i++) printf(" %d", imager_counts[i]);
                        printf("\nBytes remaining in buffer:");
                        for (int i = 0; i < 8; i++) printf(" %d", buffer_bytes[i]);
                        printf("\nBad bytes seen:");
                        for (int i = 0; i < 8; i++) printf(" %d", bad_bytes[i]);
                        printf("\n");
                    }
                }
            } else {
                printf("  Invalid packet! %02x%02x %02x%02x %02x %02x\n", packet[0], packet[1], packet[2], packet[3], packet[4], packet[5]);
            }

            delete packet;
        }
    }

    log.close();

    return 0;
}
