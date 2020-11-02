#define SYS_ID_IMG 0xC0
#define TM_EVENTGROUP 0xC0

#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "Telemetry.hpp"

void help_message(char name[])
{
    std::cout << "Usage: " << name << " [options] <files>\n";
    std::cout << "Command-line options:\n";
    std::cout << "-e             Only inspect event packets (TmType 0xC0 from imagers)\n";
    std::cout << "-fs<SystemID>  Only inspect telemetry packets with this SystemID in decimal\n";
    std::cout << "-ft<TmType>    Only inspect telemetry packets with this TmType in decimal\n";
    std::cout << "-o<outfile>    Save all valid packets to a separate file\n";
    std::cout << "-s             Strip headers when saving, preserving only the payloads\n";
}

int main(int argc, char *argv[])
{
    setbuf(stdout, NULL);

    uint16_t nfiles = 0;

    uint8_t filter_systemid = 0xFF, filter_tmtype = 0xFF;
    bool filter_events = false;

    bool save_packets = false;
    char outfile[256];
    bool strip_headers = false;

    for(int i = 1; i < argc; i++) {
        if(argv[i][0] == '-') {
            for(int j = 1; argv[i][j] != 0; j++) {
                switch(argv[i][j]) {
                    case 'e':
                        filter_events = true;
                        j = strlen(&argv[i][0]) - 1;
                        break;
                    case 'f':
                        if(argv[i][j+1] == 's') filter_systemid = atoi(&argv[i][j+2]);
                        if(argv[i][j+1] == 't') filter_tmtype = atoi(&argv[i][j+2]);
                        j = strlen(&argv[i][0]) - 1;
                        break;
                    case 'o':
                        save_packets = true;
                        strncpy(outfile, &argv[i][j+1], 256);
                        outfile[255] = 0;
                        j = strlen(&argv[i][0]) - 1;
                        break;
                    case 's':
                        strip_headers = true;
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

    if(nfiles == 0) {
        help_message(argv[0]);
        return -1;
    }

    std::streampos cur;

    uint8_t buffer[TELEMETRY_PACKET_MAX_SIZE];
    buffer[0] = 0x90;

    uint16_t length;

    uint32_t count[256][256];
    uint32_t bad_checksum[256][256];
    uint64_t amount[256][256];
    uint16_t previous_frame_counter[256][256];
    uint32_t gaps[256][256];
    Clock first_systemtime = 0, last_systemtime = 0;

    std::ofstream log;
    if (save_packets) {
        log.open(outfile, std::ofstream::binary);
    }

    TelemetryPacket tp(NULL);

    for(int i = 1; i < argc; i++) {
        memset(count, 0, sizeof(count));
        memset(bad_checksum, 0, sizeof(bad_checksum));
        memset(amount, 0, sizeof(amount));
        memset(previous_frame_counter, 0xFF, sizeof(previous_frame_counter));
        memset(gaps, 0, sizeof(gaps));
        first_systemtime = last_systemtime = 0;

        if(argv[i][0] != '-') {
            std::cout << "Inspecting " << argv[i] << std::endl;

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
                            uint8_t this_systemid = tp.getSystemID();
                            uint8_t this_tmtype = tp.getTmType();

                            if(((!filter_events) || (((this_systemid & 0xF0) == SYS_ID_IMG) && (this_tmtype == TM_EVENTGROUP))) &&
                               ((filter_systemid == 0xFF) || (this_systemid == filter_systemid)) &&
                               ((filter_tmtype == 0xFF) || (this_tmtype == filter_tmtype))) {
                                if(tp.valid()) {
                                    cur += length+15;

                                    count[this_systemid][this_tmtype]++;
                                    amount[this_systemid][this_tmtype] += length+16;
                                    uint16_t this_frame_counter = tp.getCounter();
                                    int16_t frame_counter_difference = this_frame_counter -
                                                                       previous_frame_counter[this_systemid][this_tmtype];
                                    if((previous_frame_counter[this_systemid][this_tmtype] != 0xFFFF) &&
                                       (frame_counter_difference > 1)) {
                                        gaps[this_systemid][this_tmtype]++;
                                    }
                                    previous_frame_counter[this_systemid][this_tmtype] = this_frame_counter;

                                    if(first_systemtime == 0) first_systemtime = tp.getSystemTime();
                                    last_systemtime = tp.getSystemTime();

                                    if(save_packets && log.is_open()) {
                                        if (!strip_headers) {
                                            log.write((char *)buffer, length+16);
                                        } else {
                                            log.write(((char *)buffer) + 16, length);
                                        }
                                    }
                                } else {
                                    bad_checksum[this_systemid][tp.getTmType()]++;
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

                ifs.close();

                std::cout << "\nPacket breakdown:\n";
                for(int j = 0; j < 256; j++) {
                    for(int k = 0; k < 256; k++) {
                        if(count[j][k] > 0) {
                            printf("  0x%02x 0x%02x : [%4.1f%%] %lu", j, k, 100. * amount[j][k] / size, count[j][k]);
                            if(bad_checksum[j][k] > 0) {
                                printf(", plus %lu with bad checksums", bad_checksum[j][k]);
                            }
                            if(gaps[j][k] > 0) {
                                printf(", plus %lu gaps", gaps[j][k]);
                            }
                            printf("\n");
                        }
                    }
                }

                printf("Elapsed gondola time: %llu (%f minutes)\n\n", last_systemtime - first_systemtime, (last_systemtime - first_systemtime) * 1e-7 / 60);
            }
        }
    }

    if(save_packets && log.is_open()) {
        log.close();
    }

    return 0;
}
