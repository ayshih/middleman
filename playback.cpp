#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "UDPSender.hpp"
#include "Telemetry.hpp"

void help_message(char name[])
{
    std::cout << "Usage: " << name << " [options] <files>\n";
    std::cout << "Command-line options:\n";
    std::cout << "-i<ip>         Send telemetry packets to this IP instead of 127.0.0.1\n";
    std::cout << "-p<port>       Send telemetry packets to this port instead of 60501\n";
    std::cout << "-fs<SystemID>  Send only telemetry packets with this SystemID in decimal\n";
    std::cout << "-ft<TmType>    Send only telemetry packets with this TmType in decimal\n";
    std::cout << "-s<speed>      Multiplier to speed up or slow down playback\n";
}

int main(int argc, char *argv[])
{
    setbuf(stdout, NULL);

    char ip[20];
    strncpy(ip, "127.0.0.1", 20);

    uint16_t port = 60501;
    float speed_factor = 1;

    uint16_t nfiles = 0;

    uint8_t filter_systemid = 0xFF, filter_tmtype = 0xFF;

    Clock last_systemtime = 0;

    for(int i = 1; i < argc; i++) {
        if(argv[i][0] == '-') {
            for(int j = 1; argv[i][j] != 0; j++) {
                switch(argv[i][j]) {
                    case 'f':
                        if(argv[i][j+1] == 's') filter_systemid = atoi(&argv[i][j+2]);
                        if(argv[i][j+1] == 't') filter_tmtype = atoi(&argv[i][j+2]);
                        j = strlen(&argv[i][0]) - 1;
                        break;
                    case 'i':
                        strncpy(ip, &argv[i][j+1], 20);
                        ip[19] = 0;
                        j = strlen(&argv[i][0]) - 1;
                        break;
                    case 'p':
                        port = atoi(&argv[i][j+1]);
                        j = strlen(&argv[i][0]) - 1;
                        break;
                    case 's':
                        speed_factor = atof(&argv[i][j+1]);
                        j = strlen(&argv[i][0]) - 1;
                        break;
                    case '?':
                        help_message(argv[0]);
                        return -1;
                    default:
                        std::cerr << "Unknown option, use -? to list options\n";
                        return -1;
                }
            }
        } else {
            nfiles++;
        }
    }

    std::cout << "Playing back " << nfiles << " file(s) to " << ip << ":" << port;
    std::cout << " at " << speed_factor << "x speed\n";

    if(nfiles == 0) {
        help_message(argv[0]);
        return -1;
    }

    TelemetrySender telSender(ip, port);

    std::streampos cur;

    uint8_t buffer[TELEMETRY_PACKET_MAX_SIZE];
    buffer[0] = 0x90;

    uint16_t length;

    TelemetryPacket tp(NULL);

    for(int i = 1; i < argc; i++) {
        if(argv[i][0] != '-') {
            std::cout << "Playing back " << argv[i] << std::endl;

            uint8_t percent_completed = 0;

            std::ifstream ifs(argv[i]);

            if(ifs.good()) {
                ifs.seekg(0, ifs.end);
                std::streampos size = ifs.tellg();
                ifs.seekg(0, ifs.beg);

                std::cout << size << " bytes: ";
                std::cout.flush();

                while (ifs.good()) {
                    if(ifs.get() == 0x90) {
                        if(ifs.peek() == 0xeb) {
                            cur = ifs.tellg(); // points one byte into sync word
                            ifs.seekg(5, std::ios::cur);
                            ifs.read((char *)&length, 2);

                            if(length > TELEMETRY_PACKET_MAX_SIZE-16) continue; //invalid payload size

                            ifs.seekg(cur);

                            ifs.read((char *)buffer+1, length+15);

                            tp = TelemetryPacket(buffer, length+16);

                            if(((filter_systemid == 0xFF) || (tp.getSystemID() == filter_systemid)) &&
                               ((filter_tmtype == 0xFF) || (tp.getTmType() == filter_tmtype))) {
                                if(tp.valid()) {
                                    telSender.send(&tp);

                                    cur += length+15;

                                    if ((last_systemtime != 0) && (last_systemtime < tp.getSystemTime())) {
                                        usleep((tp.getSystemTime() - last_systemtime) / 10 / speed_factor);
                                        last_systemtime = tp.getSystemTime();
                                    }
                                    if (last_systemtime == 0) last_systemtime = tp.getSystemTime();
                                } else {
                                    fprintf(stderr, "Invalid checksum: SystemId 0x%02x, TmType 0x%02x\n", tp.getSystemID(), tp.getTmType());
                                }
                            }

                            if((((uint64_t)ifs.tellg())*100/size) > percent_completed) {
                                percent_completed++;
                                printf("%3u%%\b\b\b\b", percent_completed);
                            }

                            ifs.seekg(cur);
                        }
                    }
                }

                std::cout << std::endl;

                ifs.close();
            }
        }
    }

    return 0;
}
