#define DEFAULT_EVTM_BPS_CAP 5000000

#define MAX_THREADS 50

#define LOG_PACKETS true

#define LOG_LOCATION "/home/ayshih/middleman/logs"

//Sleep settings (microseconds)
#define USLEEP_KILL            3000000 // how long to wait before terminating threads
#define USLEEP_TM_SEND            1000 // period for popping off the telemetry queue
#define USLEEP_UDP_LISTEN         1000 // safety measure in case UDP listening is changed to non-blocking
#define USLEEP_MAIN               5000 // period for checking for new commands in the queue
#define USLEEP_SERIAL_PARSER     10000 // wait time for polling a ring buffer
#define USLEEP_ADIO              10000 // wait time for aDIO operations
#define USLEEP_MAGNETOMETER     250000 // wait time for magnetometer readings

//IP addresses
#define IP_GROUND "192.168.2.200"
#define IP_MC_LOS "239.255.0.1"
#define IP_LOOPBACK "127.0.0.1"
#define IP_TM IP_MC_LOS //default IP address unless overridden on the command line

//UDP ports
#define PORT_CMD      50501 // receive commands (receive)
#define PORT_TM       60501 // send telemetry to ground network
#define PORT_EVTM     20501 // send telemetry via EVTM

//Acknowledgement error codes
#define ACK_NOERROR 0x00
#define ACK_BADCRC  0x03
#define ACK_BADSYS  0x04
#define ACK_BADCOM  0x05
#define ACK_NOACTION 0x10
#define ACK_BADVALUE 0x11

//BOOMS system ID
#define SYS_ID_SIP 0x10
#define SYS_ID_GPS 0x60
#define SYS_ID_MM  0xA0
#define SYS_ID_MAG 0xB0
#define SYS_ID_IMG 0xC0
#define SYS_ID_NAI 0xD0

//BOOMS telemetry types
#define TM_ACK          0x01
#define TM_HOUSEKEEPING 0x02
#define TM_IMG_STATS    0x0C
#define TM_GPS_PPS      0x60
#define TM_GPS_POSITION 0x61
#define TM_GPS_VELOCITY 0x62

#define TM_MAGGROUP     0xB0

#define TM_EVENTGROUP   0xC0

#define TM_MINORGROUP   0xD0

//BOOMS commands, shared
#define KEY_NULL                 0x00

//MM commands
#define KEY_MANUAL_SBD_COMM1    0x10
#define KEY_MANUAL_SBD_COMM2    0x11
#define KEY_DESTINATION_IP      0xA1
#define KEY_ENABLE_TM_FULL      0xA2
#define KEY_DISABLE_TM_FULL     0xA3
#define KEY_SET_EVTM_CAP        0xA4
#define KEY_RESTART_WORKERS     0xF0
#define KEY_RESTART_ALL         0xF1
#define KEY_EXIT                0xF2
#define KEY_SHUTDOWN            0xFF

#include <cstring>
#include <stdio.h>      /* for printf() and fprintf() */
#include <pthread.h>    /* for multithreading */
#include <stdlib.h>     /* for atoi() and exit() */
#include <signal.h>     /* for signal() */
#include <math.h>
#include <ctime>        /* time_t, struct tm, time, gmtime */
#include <iostream>
#include <string>
#include <sys/statvfs.h> /* for statvfs */
#include <fstream>
#include <unistd.h>  // read()
#include <sys/io.h>  // iopl()
#include <poll.h>
#include <sys/ioctl.h>
#include <error.h> // error()

#include "UDPSender.hpp"
#include "UDPReceiver.hpp"
#include "Command.hpp"
#include "Telemetry.hpp"
#include "ring.hpp"
#include "serial.h"

#include <aDIO_library.h>

//#include "main.hpp"

// global declarations
uint8_t command_sequence_number = -1;
uint8_t latest_system_id = 0xFF;
uint8_t latest_command_key = 0xFF;
float temp_py = 0, temp_roll = 0, temp_mb = 0;
char ip_tm[20];
uint8_t latest_housekeeping_packet[255];
uint8_t latest_magnetometer_packet[18];
uint8_t latest_imager_housekeeping[154];
uint8_t latest_spectrometer_housekeeping[54];

uint32_t evtm_bps_cap = DEFAULT_EVTM_BPS_CAP;

// global flags
bool obtaining_gps_fixes = false;
bool capping_evtm = false;
bool sending_sbd_packets = false;

// global mode variables
bool MODE_NETWORK = false;
bool MODE_TIMING = false;
bool MODE_VERBOSE = false;
bool MODE_SIMULATED_DATA = false;
bool MODE_TM_FULL = false;
bool MODE_TM_LOS = true;

// interthread signals
bool SIGNAL_RESET_TELEMETRYSENDER = false;

// UDP packet queues
TelemetryPacketQueue tm_packet_queue; //for sending
CommandPacketQueue cm_packet_queue; //for receiving

// related to threads
bool stop_message[MAX_THREADS];
pthread_t threads[MAX_THREADS];
bool started[MAX_THREADS];
int tid_listen = -1; //Stores the ID for the CommandListener thread
pthread_mutex_t mutexStartThread; //Keeps new threads from being started simultaneously

struct Thread_data{
    int thread_id;
    uint8_t system_id;
    uint8_t command_key;
    uint8_t payload_size;
    uint8_t payload[15];
};
struct Thread_data thread_data[MAX_THREADS];

sig_atomic_t volatile g_running = 1;

//Function declarations
void sig_handler(int signum);

void start_thread(void *(*start_routine) (void *), const Thread_data *tdata);
void start_all_workers();
void kill_all_threads(); //kills all threads
void kill_all_workers(); //kills all threads except the one that listens for commands

void *TelemetrySenderThread(void *threadargs);
void *TelemetryHousekeepingThread(void *threadargs);
//void *TelemetryScienceThread(void *threadargs);

void send_sbd_packet(uint8_t device_id);

void *CommandListenerThread(void *threadargs);
void cmd_process_command(CommandPacket &cp);
void *CommandHandlerThread(void *threadargs);
void queue_cmd_proc_ack_tmpacket( uint8_t error_code, uint64_t response );
//void queue_settings_tmpacket();

void *SerialListenerThread(void *threadargs);
void *SIPParserThread(void *threadargs);
void *GPSParserThread(void *threadargs);
void *ImagerParserThread(void *threadargs);
void *SpectrometerParserThread(void *threadargs);
void *MagnetometerCommanderThread(void *threadargs);
void *MagnetometerParserThread(void *threadargs);

struct gps_pos_struct{
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    float latitude;
    float longitude;
    uint16_t altitude;
};
struct gps_pos_struct gps_pos;

struct gps_for_pps_struct{
    uint8_t hour = 255;
    uint8_t minute;
    uint8_t second;
    uint32_t second_offset;
    uint8_t day_offset = 0;
    uint8_t day_of_month = 255; // starts at 1
    uint8_t month; // starts at 1
    uint16_t year; // 4-digit year
};
struct gps_for_pps_struct gps_for_pps;

void pps_tick();
void *InternalPPSThread(void *threadargs);
void pps_handler(isr_info_t info);
bool pps_received = false;
bool use_fake_pps = false;
bool gps_fix_received = false;
bool gps_time_received = false;
bool system_clock_synchronized = false;
void *ExternalPPSThread(void *threadargs);

void generate_log_filename(char *buffer);
void printLogTimestamp();
uint64_t current_monotonic_time();

template <class T>
bool set_if_different(T& variable, T value); //returns true if the value is different


uint16_t imager_counts[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint32_t imager_bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint32_t imager_bad_bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};

uint32_t spectrometer_bad_bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};

uint32_t telemetry_bytes[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int device_fd[32];
RingBuffer ring_buffer[32];

uint8_t device_id_to_system_id[20] = {SYS_ID_GPS, SYS_ID_MAG, 255, 255,
                                      SYS_ID_IMG,   SYS_ID_IMG+1, SYS_ID_NAI,   SYS_ID_NAI+1,
                                      SYS_ID_SIP,   255,          SYS_ID_IMG+2, SYS_ID_IMG+3,
                                      SYS_ID_IMG+4, SYS_ID_IMG+5, SYS_ID_NAI+2, SYS_ID_NAI+3,
                                      SYS_ID_SIP+1, 255,          SYS_ID_IMG+6, SYS_ID_IMG+7};
uint8_t system_id_to_device_id[256]; // populated in main()

void sig_handler(int signum)
{
    if ((signum == SIGINT) || (signum == SIGTERM))
    {
        if (signum == SIGINT) std::cerr << "Keyboard interrupt received\n";
        if (signum == SIGTERM) std::cerr << "Termination signal received\n";
        g_running = 0;
    }
}

void generate_log_filename(char *buffer)
{
    /*
    time_t now;
    time(&now);
    struct tm now_tm;
    gmtime_r(&now, &now_tm);
    strftime(buffer, 21, "tm_%y%m%d_%H%M%S.bin", &now_tm);
    */

    char fullpath[128];

    sprintf(fullpath, "%s/%s", LOG_LOCATION, "last_log_number");
    fullpath[128 - 1] = '\0';

    int log_number = 0;

    FILE* fp = fopen(fullpath, "r");
    if (fp != NULL) {
        fscanf(fp, "%d", &log_number);
        fclose(fp);
    }

    log_number++;

    fp = fopen(fullpath, "w");
    if (fp != NULL) {
        fprintf(fp, "%d\n", log_number);
        fclose(fp);
    }

    if (!gps_time_received) {
        sprintf(buffer, "tm_%04d_XXXXXXXX_XXXXXX.bin", log_number);
    } else {
        sprintf(buffer, "tm_%04d_%04d%02d%02d_%02d%02d%02d.bin", log_number,
                gps_for_pps.year, gps_for_pps.month, gps_for_pps.day_of_month,
                gps_for_pps.hour, gps_for_pps.minute, gps_for_pps.second);
    }
}

void printLogTimestamp()
{
    time_t now;
    time(&now);
    struct tm now_tm;
    gmtime_r(&now, &now_tm);
    char timestamp[23];
    strftime(timestamp, 23, "%Y-%m-%d %H:%M:%S UT", &now_tm);
    printf("[%s] ", timestamp);
}

template <class T>
bool set_if_different(T& variable, T value)
{
    if(variable != value) {
        variable = value;
        return true;
    } else return false;
}

// Forces a sleep through any interrupts
// This is necessary because PvAPI creates a 1-second SIGALRM timer
// http://stackoverflow.com/questions/13865166/cant-pause-a-thread-in-conjunction-with-third-party-library
int usleep_force(uint64_t microseconds)
{
    struct timespec amount, remaining;
    amount.tv_nsec = (microseconds % 1000000) * 1000;
    amount.tv_sec = microseconds / 1000000;
    while (nanosleep(&amount, &remaining) == -1) {
        amount = remaining;
    }
    return 0;
}


// Returns the current monotonic time in units of 100 ns
uint64_t current_monotonic_time()
{
    timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    return current_time.tv_sec*10000000 + current_time.tv_nsec/100;
}


uint8_t serial_packet_type(const char *char_ptr)
{
    uint8_t *ptr = (uint8_t *)char_ptr;  // change the type to support bit shifting
    return ((ptr[0] & 0b11) << 1) + (ptr[1] >> 7);
}

void make_histo(uint8_t histo[], float histogram[])
{
    for(int i = 0; i < 16; i++) {
        float total1 = 0, total2 = 0;
        for(int j = 0; j < 8; j++) {
            total1 += histogram[i * 16 + j];
            total2 += histogram[i * 16 + j + 8];
        }

        //bin 0 is 0 pixels
        //bin 1 is 1-2 pixels
        //bin 2 is 3-6 pixels
        //bin 3 is 7-16 pixels
        //bin 4 is 17-42 pixels
        //bin 5 is 43-111 pixels
        //bin 6 is 112-291 pixels
        //bin 7 is 292-760 pixels
        //and so on
        uint8_t first = (total1 > 0 ? 2.4 * log10(total1) + 15.7 : 0);
        uint8_t second = (total2 > 0 ? 2.4 * log10(total2) + 15.7 : 0);
        histo[i] = (second << 4) + first;
    }
}

void kill_all_workers()
{
    for(int i = 0; i < MAX_THREADS; i++ ){
        if ((i != tid_listen) && started[i]) {
            stop_message[i] = true;
        }
    }
    usleep_force(USLEEP_KILL);
    for(int i = 0; i < MAX_THREADS; i++ ){
        if ((i != tid_listen) && started[i]) {
            printf("Quitting thread %i, quitting status is %i\n", i, pthread_cancel(threads[i]));
            started[i] = false;
        }
    }
}

void kill_all_threads()
{
    if (started[tid_listen]) {
        stop_message[tid_listen] = true;
    }

    kill_all_workers();

    if (started[tid_listen]) {
        printf("Quitting thread %i, quitting status is %i\n", tid_listen, pthread_cancel(threads[tid_listen]));
        started[tid_listen] = false;
    }
}

void *TelemetrySenderThread(void *threadargs)
{
    long tid = (long)((struct Thread_data *)threadargs)->thread_id;
    printf("TelemetrySender thread #%ld!\n", tid);

    char filename[50];
    char fullpath[128];
    std::ofstream log;

    if (LOG_PACKETS) {
        generate_log_filename(filename);
        sprintf(fullpath, "%s/%s", LOG_LOCATION, filename);
        fullpath[128 - 1] = '\0';
        printf("Creating telemetry log file %s\n", fullpath);
        log.open(fullpath, std::ofstream::binary);
    }

    TelemetrySender *ts_full = new TelemetrySender(ip_tm, (unsigned short) PORT_TM);
    TelemetrySender *ts_los = new TelemetrySender(ip_tm, (unsigned short) PORT_EVTM);

    uint64_t last_time = current_monotonic_time();
    uint32_t total_bytes = 0, dropped_bytes = 0, dropped_packets = 0;

    while(!stop_message[tid])
    {
        usleep_force(USLEEP_TM_SEND);

        if (SIGNAL_RESET_TELEMETRYSENDER) {
            std::cout << "Resetting TelemetrySenders\n";
            TelemetrySender *old_full = ts_full;
            TelemetrySender *old_los = ts_los;
            ts_full = new TelemetrySender(ip_tm, (unsigned short) PORT_TM);
            ts_los = new TelemetrySender(ip_tm, (unsigned short) PORT_EVTM);
            delete old_full;
            delete old_los;

            // Open a new log file
            if (LOG_PACKETS && log.is_open()) {
                log.flush();
                log.close();

                generate_log_filename(filename);
                sprintf(fullpath, "%s/%s", LOG_LOCATION, filename);
                fullpath[128 - 1] = '\0';
                printf("Creating telemetry log file %s\n", fullpath);
                log.open(fullpath, std::ofstream::binary);
            }

            SIGNAL_RESET_TELEMETRYSENDER = false;
        }

        if( !tm_packet_queue.empty() ){
            TelemetryPacket tp(NULL);
            tm_packet_queue >> tp;

            uint8_t system_id = tp.getSystemID();
            telemetry_bytes[(system_id & 0xF0) >> 4] += tp.getLength();

            if(MODE_TM_FULL) ts_full->send( &tp );
            if(MODE_TM_LOS) {
                // Keep a running total bytes sent over the past second
                uint64_t now_time = current_monotonic_time();
                if (now_time - last_time > 10000000) {
                    if (dropped_packets > 0) {
                        fprintf(stderr, "TelemetrySender: Discarded %d packets (%d bytes) to comply with %d bps rate limit\n", dropped_packets, dropped_bytes, evtm_bps_cap);
                    }

                    total_bytes = 0;
                    dropped_packets = 0;
                    dropped_bytes = 0;
                    last_time = now_time;
                }
                total_bytes += tp.getLength();

                // If under the cap, proceed with sending
                if (8*total_bytes < evtm_bps_cap) {
                    ts_los->send( &tp );
                } else {
                    dropped_packets++;
                    dropped_bytes += tp.getLength();
                    capping_evtm = true;
                }
            }
            if(MODE_NETWORK) std::cout << "TelemetrySender: " << tp.getLength() << " bytes, " << tp << std::endl;

            if (LOG_PACKETS && log.is_open()) {
                uint16_t length = tp.getLength();
                uint8_t *payload = new uint8_t[length];
                tp.outputTo(payload);
                log.write((char *)payload, length);
                delete payload;
            }
        } else {
            if (LOG_PACKETS && log.is_open()) {
                log.flush();
            }
        }
    }

    printf("TelemetrySender thread #%ld exiting\n", tid);

    delete ts_full;
    delete ts_los;
    if (LOG_PACKETS && log.is_open()) log.close();

    started[tid] = false;
    pthread_exit( NULL );
}

void *TelemetryHousekeepingThread(void *threadargs)
{
    long tid = (long)((struct Thread_data *)threadargs)->thread_id;
    printf("TelemetryHousekeeping thread #%ld!\n", tid);

    uint32_t tm_frame_sequence_number = 0;

    FILE* fp = fopen("/proc/stat", "r");
    long user1, nice1, system1, idle1;
    fscanf(fp, "%*s %ld %ld %ld %ld", &user1, &nice1, &system1, &idle1);
    fclose(fp);

    long user2, nice2, system2, idle2;

    double free;

    while(!stop_message[tid])
    {
        usleep_force(1000000);
        tm_frame_sequence_number++;

        uint16_t local_imager_counts[8], buffer_sizes[8];
        uint32_t local_imager_bytes[8], local_imager_bad_bytes[8];

        memcpy(&local_imager_counts, imager_counts, sizeof(imager_counts));
        memset(&imager_counts, 0, sizeof(imager_counts));
        memcpy(&local_imager_bytes, imager_bytes, sizeof(imager_bytes));
        memset(&imager_bytes, 0, sizeof(imager_bytes));
        memcpy(&local_imager_bad_bytes, imager_bad_bytes, sizeof(imager_bad_bytes));
        memset(&imager_bad_bytes, 0, sizeof(imager_bad_bytes));

        uint8_t i;
        for (i = 0; i < 8; i++) {
            uint8_t device_id = system_id_to_device_id[SYS_ID_IMG+i];
            if (device_id != 255) buffer_sizes[i] = ring_buffer[device_id].size();
        }

        /*
        printLogTimestamp();
        printf("\nImager bytes [all(bad)]:");
        for (i = 0; i < 8; i++) printf(" %d(%d)", local_imager_bytes[i], local_imager_bad_bytes[i]);
        printf("\nImager counts:");
        for (i = 0; i < 8; i++) printf(" %d", local_imager_counts[i]);
        printf("\nBuffer sizes:");
        for (i = 0; i < 8; i++) printf(" %d", buffer_sizes[i]);
        printf("\n");
        */

        TelemetryPacket tp(SYS_ID_MM, TM_IMG_STATS, tm_frame_sequence_number, current_monotonic_time());
        for (i = 0; i < 8; i++) {
            tp << (uint16_t)local_imager_counts[i];
            tp << (uint16_t)buffer_sizes[i];
            tp << (uint16_t)local_imager_bad_bytes[i];
        }
        tm_packet_queue << tp;

        TelemetryPacket tphk(SYS_ID_MM, TM_HOUSEKEEPING, tm_frame_sequence_number, current_monotonic_time());

        uint8_t status_bitfield = 0;
        bitwrite(&status_bitfield, 0, 1, obtaining_gps_fixes);
        bitwrite(&status_bitfield, 1, 1, !use_fake_pps);
        bitwrite(&status_bitfield, 4, 1, sending_sbd_packets);
        bitwrite(&status_bitfield, 5, 1, MODE_TM_FULL);
        bitwrite(&status_bitfield, 6, 1, MODE_TM_LOS);
        bitwrite(&status_bitfield, 7, 1, capping_evtm);
        obtaining_gps_fixes = false;
        sending_sbd_packets = false;
        capping_evtm = false;

        tphk << status_bitfield << latest_command_key;

        // CPU usage
        fp = fopen("/proc/stat", "r");
        fscanf(fp, "%*s %ld %ld %ld %ld", &user2, &nice2, &system2, &idle2);
        fclose(fp);
        tphk << (uint16_t)(100 * 100 * (1 - (double)(idle2 - idle1) / (user2 + nice2 + system2 + idle2 - user1 - nice1 - system1 - idle1)));
        user1 = user2;
        nice1 = nice2;
        system1 = system2;
        idle1 = idle2;

        // Disk usage
        struct statvfs vfs;
        statvfs("/", &vfs);
        free = (double)vfs.f_bavail / vfs.f_blocks;
        tphk << (uint16_t)(100 * 100 * (1 - free));

        // Uptime
        fp = fopen("/proc/uptime", "r");
        float uptime;
        fscanf(fp, "%f", &uptime);
        fclose(fp);
        tphk << (uint32_t)uptime;

        uint32_t local_telemetry_bytes[16];
        memcpy(&local_telemetry_bytes, telemetry_bytes, sizeof(telemetry_bytes));
        memset(&telemetry_bytes, 0, sizeof(telemetry_bytes));
        tphk << (uint8_t)local_telemetry_bytes[SYS_ID_MM >> 4];
        tphk << (uint8_t)local_telemetry_bytes[SYS_ID_GPS >> 4];
        tphk << (uint32_t)local_telemetry_bytes[SYS_ID_IMG >> 4];
        tphk << (uint16_t)local_telemetry_bytes[SYS_ID_NAI >> 4];
        tphk << (uint8_t)local_telemetry_bytes[SYS_ID_MAG >> 4];

        // Temperatures
        int8_t temperatures[7], max_core_temp = -128;
        int sensor_temp;
        memset(&temperatures, -128, sizeof(temperatures));
        char filename[128], sensor_name[20];
        for (i = 0; i < 4; i++) {
            sprintf(filename, "/sys/class/hwmon/hwmon%d/name", i);
            fp = fopen(filename, "r");
            if (fp == NULL) { perror(NULL); continue; }
            fscanf(fp, "%s", sensor_name);
            fclose(fp);

            if (strcmp(sensor_name, "coretemp") == 0) {
                for (int j = 2; j <= 5; j++) {
                    sprintf(filename, "/sys/class/hwmon/hwmon%d/temp%d_input", i, j);
                    fp = fopen(filename, "r");
                    if (fp == NULL) { perror(NULL); continue; }
                    fscanf(fp, "%d", &sensor_temp);
                    fclose(fp);
                    temperatures[j+1] = sensor_temp / 1000;
                    if (temperatures[j+1] > max_core_temp) max_core_temp = temperatures[j+1];
                }
            } else {
                sprintf(filename, "/sys/class/hwmon/hwmon%d/temp1_input", i);
                fp = fopen(filename, "r");
                if (fp == NULL) { perror(NULL); continue; }
                fscanf(fp, "%d", &sensor_temp);
                fclose(fp);
                if (strcmp(sensor_name, "acpitz") == 0) temperatures[0] = sensor_temp / 1000;
                if (strcmp(sensor_name, "soc_dts0") == 0) temperatures[1] = sensor_temp / 1000;
                if (strcmp(sensor_name, "soc_dts1") == 0) temperatures[2] = sensor_temp / 1000;
            }
        }
        tphk << temperatures[0] << temperatures[1] << temperatures[2] << max_core_temp;

        tm_packet_queue << tphk;

        tphk.outputTo(latest_housekeeping_packet);
    }

    printf("TelemetryHousekeeping thread #%ld exiting\n", tid);
    started[tid] = false;
    pthread_exit( NULL );
}

/*
void *TelemetryScienceThread(void *threadargs)
{
    long tid = (long)((struct Thread_data *)threadargs)->thread_id;
    printf("TelemetryScience thread #%ld!\n", tid);

    uint32_t tm_frame_sequence_number = 0;
    float old_grid_orientation = 0;

    while(!stop_message[tid])
    {
        if (MODE_POINTING) {
            usleep_force(1000000 / current_settings.PY_rate);
        } else {
            usleep_force(current_settings.cadence_science * 1000000);
        }
        tm_frame_sequence_number++;

        TelemetryPacket tp(SYS_ID_ASP, TM_SCIENCE, tm_frame_sequence_number, oeb_get_clock());

        pthread_mutex_lock(&mutexAnalysis);

        uint8_t quality_bitfield = 0;
        bitwrite(&quality_bitfield, 0, 1, PY_ANALYSIS.there[0]);
        bitwrite(&quality_bitfield, 1, 1, PY_ANALYSIS.there[1]);
        bitwrite(&quality_bitfield, 2, 1, PY_ANALYSIS.there[2]);
        // bit 3 is fiducial lock
        bitwrite(&quality_bitfield, 4, 1, R_ANALYSIS.good_contrast);
        bitwrite(&quality_bitfield, 5, 1, R_ANALYSIS.good_black_level);
        // bit 6 is PY-determined rotating
        bitwrite(&quality_bitfield, 7, 1, grid_rotation_rate > 5);
        tp << quality_bitfield;

        uint8_t count1 = py_image_counter;
        uint8_t count2 = roll_image_counter;
        tp << count1 << count2;
        py_image_counter = 0;
        roll_image_counter = 0;

        uint8_t num_fiducials = PY_ANALYSIS.nfid;
        tp << num_fiducials;

        float offset_pitch = 0, rotated_offset_pitch = 0, uncert_pitch = 0;
        float offset_yaw = 0, rotated_offset_yaw = 0, uncert_yaw = 0;
        // Assume that if there are two Suns, then the brighter one is the main Sun
        if (PY_ANALYSIS.there[1]) {
            offset_pitch = -(PY_ANALYSIS.xp[0] - current_settings.screen_center_x) * current_settings.arcsec_per_pixel_x / 3600. * M_PI / 180.;
            offset_yaw = (PY_ANALYSIS.yp[0] - current_settings.screen_center_y) * current_settings.arcsec_per_pixel_y / 3600. * M_PI / 180.;
            rotated_offset_pitch = offset_pitch * cos(current_settings.screen_rotation * M_PI / 180) +
                                   - offset_yaw * sin(current_settings.screen_rotation * M_PI / 180);
            rotated_offset_yaw = offset_pitch * sin(current_settings.screen_rotation * M_PI / 180) +
                                 offset_yaw * cos(current_settings.screen_rotation * M_PI / 180);
            rotated_offset_yaw *= -1; // hack to get into Pascal's coordinate system
        }
        tp << rotated_offset_pitch << uncert_pitch << rotated_offset_yaw << uncert_yaw;

        float new_grid_orientation = PY_ANALYSIS.theta;
        float delta_grid_orientation = new_grid_orientation - old_grid_orientation;
        if (delta_grid_orientation < 0) delta_grid_orientation += 360.;
        tp << new_grid_orientation << delta_grid_orientation;
        old_grid_orientation = new_grid_orientation;

        tp << grid_rotation_rate;

        uint8_t py_histo[16];
        make_histo(py_histo, PY_ANALYSIS.histogram);
        tp.append_bytes(py_histo, 16);

        uint8_t roll_histo[16];
        make_histo(roll_histo, R_ANALYSIS.histogram);
        tp.append_bytes(roll_histo, 16);

        //the three Sun centers in pixel coordinates
        for (int i = 0; i < 3; i++) {
            tp << (uint16_t)(PY_ANALYSIS.xp[i] * 10) << (uint16_t)(PY_ANALYSIS.yp[i] * 10);
        }

        for (int i = 0; i < 4; i++) {
            if(num_fiducials > 1) {
                int index = ((tm_frame_sequence_number + i) % num_fiducials);
                tp << (uint16_t)(PY_ANALYSIS.xfid[index] * 10) << (uint16_t)(PY_ANALYSIS.yfid[index] * 10);
            } else {
                tp << (uint16_t)0 << (uint16_t)0;
            }
        }

        pthread_mutex_unlock(&mutexAnalysis);

        tm_packet_queue << tp;
    }

    printf("TelemetryScience thread #%ld exiting\n", tid);
    started[tid] = false;
    pthread_exit( NULL );
}
*/

void *CommandListenerThread(void *threadargs)
{
    long tid = (long)((struct Thread_data *)threadargs)->thread_id;
    printf("CommandListener thread #%ld!\n", tid);

    tid_listen = tid;

    CommandReceiver comReceiver( (unsigned short) PORT_CMD);
    comReceiver.init_connection();

    /*
    // If talking to the flight computer, synchronize the clock on the odds & ends board
    // This code is here to make sure that we are ready to receive the imminent sync command
    if (strncmp(ip_tm, IP_FC, 20) == 0) {
        printLogTimestamp();
        std::cout << "Sending synchronization command to flight computer\n";
        CommandSender cmdSender(IP_FC, PORT_CMD);
        CommandPacket cp(0x0A, 0x51, 0xF0);
        cmdSender.send( &cp );
    }
    */

    while(!stop_message[tid])
    {
        int packet_length;

        usleep_force(USLEEP_UDP_LISTEN);
        packet_length = comReceiver.listen( );
        if (packet_length <= 0) continue;
        printLogTimestamp();
        printf("CommandListenerThread: %i bytes, ", packet_length);
        uint8_t *packet;
        packet = new uint8_t[packet_length];
        comReceiver.get_packet( packet );

        CommandPacket command_packet( packet, packet_length );

        if (command_packet.valid()){
            printf("valid checksum, ");

            command_sequence_number = command_packet.getCounter();

            // update the command count
            printf("command sequence number %i", command_sequence_number);

            cm_packet_queue << command_packet;
        } else {
            printf("INVALID checksum");
            queue_cmd_proc_ack_tmpacket(ACK_BADCRC, 0xFFFFFFFF);
        }
        printf("\n");

        delete packet;
    }

    printf("CommandListener thread #%ld exiting\n", tid);
    comReceiver.close_connection();
    started[tid] = false;
    pthread_exit( NULL );
}

void queue_cmd_proc_ack_tmpacket( uint8_t error_code, uint64_t response )
{
    TelemetryPacket ack_tp(SYS_ID_MM, TM_ACK, command_sequence_number, current_monotonic_time());
    ack_tp << error_code << response;
    tm_packet_queue << ack_tp;
}

/*
void queue_settings_tmpacket()
{
    static uint16_t counter = 0;
    TelemetryPacket tp(SYS_ID_ASP, TM_SETTINGS, counter, oeb_get_clock());

    tp << (uint16_t)current_table;

    uint8_t py_fps = current_settings.PY_rate;
    uint8_t py_gain = current_settings.PY_gain;
    uint16_t py_exposure = current_settings.PY_exposure;
    tp << py_fps << py_gain << py_exposure;

    uint8_t roll_fps = current_settings.R_rate;
    uint8_t roll_gain = current_settings.R_gain;
    uint16_t roll_exposure = current_settings.R_exposure;
    tp << roll_fps << roll_gain << roll_exposure;

    tm_packet_queue << tp;
}
*/

void *CommandHandlerThread(void *threadargs)
{
    long tid = (long)((struct Thread_data *)threadargs)->thread_id;
    struct Thread_data *my_data;
    uint8_t error_code = 0xFF; //command not implemented
    uint64_t response = 0;
    my_data = (struct Thread_data *) threadargs;

    uint64_t value = *(uint64_t *)(my_data->payload);
    uint8_t *bytes = (uint8_t *)(my_data->payload);

    switch(my_data->system_id)
    {
        case SYS_ID_MM:
            switch(my_data->command_key)
            {
                case KEY_MANUAL_SBD_COMM1:
                    if (system_id_to_device_id[SYS_ID_SIP] != 255) {
                        send_sbd_packet(system_id_to_device_id[SYS_ID_SIP]);
                        error_code = 0;
                    } else {
                        error_code = ACK_NOACTION;
                    }
                    break;
                case KEY_MANUAL_SBD_COMM2:
                    if (system_id_to_device_id[SYS_ID_SIP+1] != 255) {
                        send_sbd_packet(system_id_to_device_id[SYS_ID_SIP+1]);
                        error_code = 0;
                    } else {
                        error_code = ACK_NOACTION;
                    }
                    break;
                case KEY_DESTINATION_IP:
                    // payload should be the four bytes of the IP
                    if (my_data->payload_size == 4) {
                        char new_ip_tm[16];
                        sprintf(new_ip_tm, "%d.%d.%d.%d", bytes[0], bytes[1], bytes[2], bytes[3]);
                        std::cout << "Destination IP for telemetry will be changed from " << ip_tm << " to " << new_ip_tm << std::endl;
                        strcpy(ip_tm, new_ip_tm);
                        SIGNAL_RESET_TELEMETRYSENDER = true;
                        error_code = 0;
                    } else {
                        error_code = ACK_BADVALUE;
                    }
                    break;
                case KEY_ENABLE_TM_FULL:
                    if(!MODE_TM_FULL) {
                        std::cout << "Turning ON full telemetry on port " << PORT_TM << std::endl;
                        MODE_TM_FULL = true;
                        error_code = 0;
                    } else {
                        error_code = ACK_NOACTION;
                    }
                    break;
                case KEY_DISABLE_TM_FULL:
                    if(MODE_TM_FULL) {
                        std::cout << "Turning OFF full telemetry on port " << PORT_TM << std::endl;
                        MODE_TM_FULL = false;
                        error_code = 0;
                    } else {
                        error_code = ACK_NOACTION;
                    }
                    break;
                case KEY_SET_EVTM_CAP:
                    value &= 0xFFFFFFFF;
                    if(value > 0) {
                        if(set_if_different(evtm_bps_cap, (uint32_t)value)) {
                            std::cout << "Setting EVTM bps cap to " << evtm_bps_cap << std::endl;
                            error_code = 0;
                        } else {
                            error_code = ACK_NOACTION;
                        }
                    } else {
                        error_code = ACK_BADVALUE;
                    }
                    break;
                default:
                    error_code = ACK_BADCOM; //unknown command
                    response = my_data->command_key;
            } //switch for command key
            break;
        case SYS_ID_MAG:
            switch(my_data->command_key & 0xF)
            {
                default:
                    error_code = ACK_BADCOM; //unknown command
                    response = my_data->command_key;
            } //switch for command key
            break;
        case SYS_ID_IMG:
            switch(my_data->command_key & 0xF)
            {
                default:
                    error_code = ACK_BADCOM; //unknown command
                    response = my_data->command_key;
            } //switch for command key
            break;
        case SYS_ID_NAI:
            switch(my_data->command_key & 0xF)
            {
                default:
                    error_code = ACK_BADCOM; //unknown command
                    response = my_data->command_key;
            } //switch for command key
            break;
        default:
            error_code = ACK_BADSYS; //unknown system ID
            response = my_data->system_id;
    } //switch for system ID

    switch(error_code) {
        case 0:
            std::cout << "Command processed successfully\n";
            break;
        case ACK_BADSYS:
            std::cerr << "Unknown system ID\n";
            break;
        case ACK_BADCOM:
            std::cerr << "Unknown command\n";
            break;
        case ACK_NOACTION:
            std::cerr << "Command ignored\n";
            break;
        case ACK_BADVALUE:
            std::cerr << "Command had a bad value\n";
            break;
        case 0xFF:
        default:
            std::cerr << "Command not yet implemented\n";
            break;
    }

    queue_cmd_proc_ack_tmpacket( error_code, response );

    started[tid] = false;
    pthread_exit(NULL);
}


void *SerialListenerThread(void *threadargs)
{
    Thread_data *my_data = (Thread_data *) threadargs;
    int tid = my_data->thread_id;

    uint8_t device_id = system_id_to_device_id[my_data->system_id];
    if (device_id == 255) {
        std::cerr << "Unknown system ID for serial listening\n";
        started[tid] = false;
        pthread_exit(NULL);
    }

    printf("SerialListener thread #%d [system 0x%02X]\n", tid, my_data->system_id);

    switch(my_data->system_id & 0xF0) {
        case SYS_ID_SIP:
            device_fd[device_id] = setup_serial_port(device_id, B1200);  // TODO: close this
            break;
        case SYS_ID_GPS:
            device_fd[device_id] = setup_serial_port(device_id, B4800);  // TODO: close this
            break;
        case SYS_ID_MAG:
            device_fd[device_id] = setup_serial_port(device_id, B9600);  // TODO: close this
            break;
        case SYS_ID_IMG:
            device_fd[device_id] = setup_serial_port(device_id, B230400);  // TODO: close this
            break;
        case SYS_ID_NAI:
            device_fd[device_id] = setup_serial_port(device_id, B38400);  // TODO: close this
            break;
        default:
            std::cerr << "Invalid system ID for serial listening\n";
            started[tid] = false;
            pthread_exit(NULL);
    }

    printLogTimestamp();
    printf("Device /dev/ttyS%d opened for system 0x%02X\n", device_id, my_data->system_id);

    switch(my_data->system_id & 0xF0) {
        case SYS_ID_SIP:
            start_thread(SIPParserThread, my_data);
            break;
        case SYS_ID_GPS:
            start_thread(GPSParserThread, my_data);
            break;
        case SYS_ID_MAG:
            start_thread(MagnetometerCommanderThread, my_data);
            start_thread(MagnetometerParserThread, my_data);
            break;
        case SYS_ID_IMG:
            start_thread(ImagerParserThread, my_data);
            break;
        case SYS_ID_NAI:
            start_thread(SpectrometerParserThread, my_data);
            break;
        default:
            fprintf(stderr, "Unknown system ID (%02X) for serial device /dev/tty%d\n", my_data->system_id, device_id);
    }

    char read_buffer[1024];

    struct pollfd serial_poll;
    serial_poll.fd = device_fd[device_id];
    serial_poll.events = POLLIN;

    while(!stop_message[tid])
    {
        int c = polled_read(device_fd[device_id], &serial_poll, &read_buffer, sizeof(read_buffer));

        if(c > 0) {
            if(MODE_VERBOSE) {
                printLogTimestamp();
                printf("Read %d bytes from device %d (starts with 0x%02X)\n", c, device_id, read_buffer[0]);
            }
            ring_buffer[device_id].append(read_buffer, c);

            if (my_data->system_id != SYS_ID_GPS) imager_bytes[my_data->system_id & 0x07] += c;
        }
    }

    printf("SerialListener thread #%d [device %d] exiting\n", tid, device_id);
    started[tid] = false;
    pthread_exit( NULL );
}


void *MagnetometerCommanderThread(void *threadargs)
{
    Thread_data *my_data = (Thread_data *) threadargs;
    int tid = my_data->thread_id;
    uint8_t device_id = system_id_to_device_id[my_data->system_id];

    printf("MagnetometerCommander thread #%d [system 0x%02X]\n", tid, my_data->system_id);

    uint8_t mag_device = system_id_to_device_id[SYS_ID_MAG];

    struct pollfd serial_poll;
    serial_poll.fd = device_fd[mag_device];
    serial_poll.events = POLLOUT;

    while(!stop_message[tid])
    {
        // Request magnetometer information
        polled_write(serial_poll.fd, &serial_poll, "\xBF", 1);
        usleep_force(USLEEP_MAGNETOMETER);
    }

    printf("MagnetometerCommander thread #%d [device %d] exiting\n", tid, device_id);
    started[tid] = false;
    pthread_exit( NULL );
}


void *MagnetometerParserThread(void *threadargs)
{
    Thread_data *my_data = (Thread_data *) threadargs;
    int tid = my_data->thread_id;
    uint8_t device_id = system_id_to_device_id[my_data->system_id];

    printf("MagnetometerParser thread #%d [system 0x%02X]\n", tid, my_data->system_id);

    char packet_buffer[1024];

    TelemetryPacket tp_maggroup(NULL);
    int counter = 0;

    while(!stop_message[tid])
    {
        int packet_size;
        while((packet_size = ring_buffer[device_id].smart_pop_magnetometer(packet_buffer)) != 0) {
            if(packet_size == -1) {
                fprintf(stderr, "Skipping a byte on device %d\n", device_id);
                continue;
            }

            if(counter == 0) {
                tp_maggroup = TelemetryPacket(my_data->system_id, TM_MAGGROUP, 0, current_monotonic_time());  // TODO: needs counter
            }

            //for(int i=0; i<packet_size; i++) printf("%02X ", (uint8_t)packet_buffer[i]);
            //printf("\n");
            counter++;
            tp_maggroup.append_bytes(packet_buffer, packet_size);

            if(counter == 4) {
                tm_packet_queue << tp_maggroup;
                memcpy(latest_magnetometer_packet, packet_buffer, packet_size);
                counter = 0;
            }
        }

        usleep_force(USLEEP_SERIAL_PARSER);
    }

    printf("MagnetometerParser thread #%d [device %d] exiting\n", tid, device_id);
    started[tid] = false;
    pthread_exit( NULL );
}


void *ImagerParserThread(void *threadargs)
{
    Thread_data *my_data = (Thread_data *) threadargs;
    int tid = my_data->thread_id;
    uint8_t device_id = system_id_to_device_id[my_data->system_id];

    printf("ImagerParser thread #%d [system 0x%02X]\n", tid, my_data->system_id);

    char packet_buffer[1024];

    TelemetryPacket tp_eventgroup(my_data->system_id, TM_EVENTGROUP, 0, current_monotonic_time());  // TODO: needs counter
    int events_in_group = 0;

    int last_number = -1;

    while(!stop_message[tid])
    {
        int packet_size;
        while((packet_size = ring_buffer[device_id].smart_pop_imager(packet_buffer)) != 0) {
            if(packet_size == -1) {
                fprintf(stderr, "Skipping a byte on device %d\n", device_id);
                imager_bad_bytes[my_data->system_id & 0b111]++;
                continue;
            }

            if(MODE_VERBOSE) {
                printLogTimestamp();
                printf("Parsed an imager packet of %d bytes from device %d\n", packet_size, device_id);
            }

            uint8_t packet_type = serial_packet_type(packet_buffer);
            if(packet_type == 0b000) {  // event packets are grouped
                events_in_group++;
                imager_counts[my_data->system_id & 0b111]++;
                tp_eventgroup.append_bytes(packet_buffer, packet_size);

                if(MODE_SIMULATED_DATA) { // verify the incrementing pattern in simulated data
                    int number;
                    memcpy(&number, packet_buffer + 2, sizeof(number));

                    if(number != last_number + 1) {
                        fprintf(stderr, "Desync with %d dropped packets (%d, %d)\n", number - last_number - 1, last_number, number);
                        imager_bad_bytes[my_data->system_id & 0b111] += packet_size;
                    }
                    last_number = number;
                }

                // Force out an event-group packet if there are 100 events
                if(events_in_group == 100) {
                    tm_packet_queue << tp_eventgroup;
                    tp_eventgroup = TelemetryPacket(my_data->system_id, TM_EVENTGROUP, 0, current_monotonic_time());  // TODO: needs counter
                    events_in_group = 0;
                }
            } else if(packet_type <= 0b111) {  // housekeeping packets are passed through
                TelemetryPacket tp_hk(my_data->system_id, packet_type + 8, 0, current_monotonic_time());  // TODO: needs counter
                tp_hk.append_bytes(packet_buffer, packet_size);
                tm_packet_queue << tp_hk;

                switch (packet_type) {
                    case 1:
                    case 2:
                    case 3:
                    case 4:
                        memcpy(latest_imager_housekeeping + 22 * (my_data->system_id & 0b111) + 2 * (packet_type - 1), packet_buffer + 4, 2);
                        break;
                    case 6:
                        memcpy(latest_imager_housekeeping + 22 * (my_data->system_id & 0b111) + 8, packet_buffer + 2, 4);
                        memcpy(latest_imager_housekeeping + 22 * (my_data->system_id & 0b111) + 12, packet_buffer + 16, 2);
                        memcpy(latest_imager_housekeeping + 22 * (my_data->system_id & 0b111) + 14, packet_buffer + 8, 8);
                        break;
                }
            } else {
                fprintf(stderr, "Unknown imager packet of type 0b%d%d%d\n", packet_type & 0b100 >> 2, packet_type & 0b010 >> 1, packet_type & 0b001);
                imager_bad_bytes[my_data->system_id & 0b111] += packet_size;
            }
        }

        // Reset the event-group packet if the packet is more than 1 second old
        if(current_monotonic_time() - tp_eventgroup.getSystemTime() > 10000000) {
            if(events_in_group > 0) {
                tm_packet_queue << tp_eventgroup;
                events_in_group = 0;
            }
            tp_eventgroup = TelemetryPacket(my_data->system_id, TM_EVENTGROUP, 0, current_monotonic_time());  // TODO: needs counter
        }
            
        usleep_force(USLEEP_SERIAL_PARSER);
    }

    printf("ImagerParser thread #%d [device %d] exiting\n", tid, device_id);
    started[tid] = false;
    pthread_exit( NULL );
}


void *SpectrometerParserThread(void *threadargs)
{
    Thread_data *my_data = (Thread_data *) threadargs;
    int tid = my_data->thread_id;
    uint8_t device_id = system_id_to_device_id[my_data->system_id];

    printf("SpectrometerParser thread #%d [system 0x%02X]\n", tid, my_data->system_id);

    char packet_buffer[1024];

    TelemetryPacket tp_minorgroup(my_data->system_id, TM_MINORGROUP, 0, current_monotonic_time());  // TODO: needs counter

    while(!stop_message[tid])
    {
        int packet_size;
        while((packet_size = ring_buffer[device_id].smart_pop_spectrometer(packet_buffer)) != 0) {
            if(packet_size == -1) {
                fprintf(stderr, "Skipping a byte on device %d\n", device_id);
                spectrometer_bad_bytes[my_data->system_id & 0b111]++;
                continue;
            }

            if(MODE_VERBOSE) {
                printLogTimestamp();
                printf("Parsed a spectrometer packet of %d bytes from device %d\n", packet_size, device_id);
            }

            uint32_t minor_frame_counter = ((uint8_t)packet_buffer[3] << 16) | ((uint8_t)packet_buffer[4] << 8) | (uint8_t)packet_buffer[5];

            tp_minorgroup.append_bytes(packet_buffer, packet_size);

            // Group every four spectrometer packets
            if ((minor_frame_counter & 0b11) == 3) {
                tm_packet_queue << tp_minorgroup;
                tp_minorgroup = TelemetryPacket(my_data->system_id, TM_MINORGROUP, 0, current_monotonic_time());  // TODO: needs counter
            }

            // Housekeeping
            switch (minor_frame_counter & 0b11111) {
                case 0:
                    memcpy(latest_spectrometer_housekeeping + 18 * (my_data->system_id & 0b11) + 10, packet_buffer+206, 2);
                    memcpy(latest_spectrometer_housekeeping + 18 * (my_data->system_id & 0b11) + 14, packet_buffer+208, 2);
                    break;
                case 1:
                    memcpy(latest_spectrometer_housekeeping + 18 * (my_data->system_id & 0b11) + 12, packet_buffer+206, 2);
                    memcpy(latest_spectrometer_housekeeping + 18 * (my_data->system_id & 0b11) + 16, packet_buffer+208, 2);
                    break;
                case 2:
                    memcpy(latest_spectrometer_housekeeping + 18 * (my_data->system_id & 0b11) + 4, packet_buffer+206, 4);
                    break;
                case 3:
                    memcpy(latest_spectrometer_housekeeping + 18 * (my_data->system_id & 0b11) + 8, packet_buffer+206, 2);
                    break;
                case 4:
                    memcpy(latest_spectrometer_housekeeping + 18 * (my_data->system_id & 0b11), packet_buffer+206, 4);
                    break;
            }

        }

        usleep_force(USLEEP_SERIAL_PARSER);
    }

    printf("SpectrometerParser thread #%d [device %d] exiting\n", tid, device_id);
    started[tid] = false;
    pthread_exit( NULL );
}


bool verify_nmea_checksum(char *buf, int buf_size)
{
    uint8_t xor_checksum = buf[1];
    for (int i=2; i<buf_size-5; i++) xor_checksum ^= buf[i];
    char xor_checksum_in_hex[3];
    sprintf(xor_checksum_in_hex, "%02X", xor_checksum);
    return strncmp(xor_checksum_in_hex, &buf[buf_size-4], 2) == 0;
}


void commastring_to_argv(char *string, char **argv, int max_args)
{
    char **substring;
    for (substring = argv; (*substring = strsep(&string, ",")) != NULL;) {
        //printf("> %s\n", *substring);
        if (++substring >= &argv[max_args])
            break;
    }
}


void *GPSParserThread(void *threadargs)
{
    Thread_data *my_data = (Thread_data *) threadargs;
    int tid = my_data->thread_id;

    printf("GPSParser thread #%d\n", tid);

    char packet_buffer[1024], saved_buffer[1024];
    char *argv[30];

    uint8_t gps_device = system_id_to_device_id[SYS_ID_GPS];

    while(!stop_message[tid])
    {
        int packet_size;
        while((packet_size = ring_buffer[gps_device].smart_pop_nmea(packet_buffer)) != 0) {
            if(packet_size == -1) {
                fprintf(stderr, "Skipping a byte on device %d\n", gps_device);
                continue;
            }

            if(MODE_VERBOSE) {
                printLogTimestamp();
                printf("Parsed a GPS packet of %d bytes from device %d\n", packet_size, gps_device);
            }

            // Check the NMEA checksum
            if (packet_buffer[0] != '$' || packet_buffer[packet_size-5] != '*' ||
                !verify_nmea_checksum(packet_buffer, packet_size)) {
                packet_buffer[packet_size] = 0;
                std::cerr << "Invalid NMEA checksum for packet: " << packet_buffer << std::endl;
                continue;
            }

            packet_buffer[packet_size] = 0;
            strncpy(saved_buffer, packet_buffer, packet_size+1);

            if (strncmp(packet_buffer, "$GPGGA", 6) == 0) {
                // GPS position packet
                commastring_to_argv(packet_buffer, argv, 30);

                TelemetryPacket tp_gps_pos(SYS_ID_GPS, TM_GPS_POSITION, 0, current_monotonic_time());  // TODO: needs counter

                int8_t hour = -1, minute = -1, second = -1, frac_second = -1;
                std::string hms_s(argv[1]);
                if (hms_s.length() > 0) {
                    hour = std::stoi(hms_s.substr(0, 2));
                    minute = std::stoi(hms_s.substr(2, 2));
                    second = std::stoi(hms_s.substr(4, 2));
                    frac_second = std::stoi(hms_s.substr(7, 2));
                }
                tp_gps_pos << hour << minute << second << frac_second;

                double latitude = -1;
                std::string lat_s(argv[2]), lat_ns_s(argv[3]);
                if ((lat_s.length() > 0) && (lat_ns_s.length() > 0)) {
                    latitude = std::stoi(lat_s.substr(0, 2)) +
                               std::stod(lat_s.substr(2, 8)) / 60;
                    if (lat_ns_s[0] == 'S') latitude *= -1;
                }
                tp_gps_pos << latitude;

                double longitude = -1;
                std::string lon_s(argv[4]), lon_ew_s(argv[5]);
                if ((lon_s.length() > 0) && (lon_ew_s.length() > 0)) {
                    longitude = std::stoi(lon_s.substr(0, 3)) +
                                std::stod(lon_s.substr(3, 8)) / 60;
                    if (lon_ew_s[0] == 'W') longitude *= -1;
                }
                tp_gps_pos << longitude;

                uint8_t quality = atoi(argv[6]);
                tp_gps_pos << quality;

                uint8_t num_sat = atoi(argv[7]);
                tp_gps_pos << num_sat;

                float hdop = -1;
                if (strlen(argv[8]) > 0) hdop = atof(argv[8]);
                tp_gps_pos << hdop;

                uint16_t altitude = 65535;
                if (strlen(argv[9]) > 0) altitude = atoi(argv[9]);
                tp_gps_pos << altitude;

                int16_t geoidal = -32768;
                if (strlen(argv[11]) > 0) geoidal = atoi(argv[11]);
                tp_gps_pos << geoidal;

                tm_packet_queue << tp_gps_pos;

                gps_pos.hour = hour;
                gps_pos.minute = minute;
                gps_pos.second = second;
                gps_pos.latitude = float(latitude);
                gps_pos.longitude = float(longitude);
                gps_pos.altitude = altitude;

                if (hour == -1) {
                    printLogTimestamp();
                    printf("GPS packet missing information: %s", saved_buffer);
                }

                if ((quality != 0) && (frac_second == 0)) {
                    obtaining_gps_fixes = true;
                    gps_fix_received = true;
                }
            } else if (strncmp(packet_buffer, "$GPZDA", 6) == 0) {
                // GPS time & date packet
                commastring_to_argv(packet_buffer, argv, 30);

                if (gps_fix_received) {
                    int8_t hour = -1, minute = -1, second = -1;
                    std::string hms_s(argv[1]);
                    if (hms_s.length() > 0) {
                        hour = std::stoi(hms_s.substr(0, 2));
                        minute = std::stoi(hms_s.substr(2, 2));
                        second = std::stoi(hms_s.substr(4, 2));
                    }

                    int8_t day_of_month = -1, month = -1;
                    int16_t year = -1;
                    if (strlen(argv[2]) > 0) day_of_month = atoi(argv[2]);
                    if (strlen(argv[3]) > 0) month = atoi(argv[3]);
                    if (strlen(argv[4]) > 0) year = atoi(argv[4]);

                    if (day_of_month != -1 && gps_for_pps.day_of_month != 255 && day_of_month != gps_for_pps.day_of_month) {  // day rollover
                        gps_for_pps.day_offset++;
                    }
                    gps_for_pps.hour = hour;
                    gps_for_pps.minute = minute;
                    gps_for_pps.second = second;
                    gps_for_pps.second_offset = 1;
                    gps_for_pps.day_of_month = day_of_month;
                    gps_for_pps.month = month;
                    gps_for_pps.year = year;

                    if (!gps_time_received && hour != -1) {
                        gps_time_received = true;
                        SIGNAL_RESET_TELEMETRYSENDER = true;
                    }
                }

            } else if (strncmp(packet_buffer, "$GPVTG", 6) == 0) {
                // GPS velocity packet
                commastring_to_argv(packet_buffer, argv, 30);

                TelemetryPacket tp_gps_vel(SYS_ID_GPS, TM_GPS_VELOCITY, 0, current_monotonic_time());  // TODO: needs counter

                float tmg_true = -1, tmg_mag = -1, sog_kph = -1;
                if (strlen(argv[1]) > 0) tmg_true = atof(argv[1]);
                if (strlen(argv[3]) > 0) tmg_mag = atof(argv[3]);
                if (strlen(argv[7]) > 0) sog_kph = atof(argv[7]);
                tp_gps_vel << tmg_true << tmg_mag << sog_kph;

                uint8_t mode = argv[9][0], reserved = 0;
                tp_gps_vel << mode << reserved;

                tm_packet_queue << tp_gps_vel;

                if (tmg_true < 0) {
                    printLogTimestamp();
                    printf("GPS packet missing information: %s", saved_buffer);
                }
            } else {
                packet_buffer[packet_size] = 0;
                printLogTimestamp();
                printf("GPS packet not handled: %s", packet_buffer);
            }
        }

        usleep_force(USLEEP_SERIAL_PARSER);
    }

    printf("GPSParser thread #%d exiting\n", tid);
    started[tid] = false;
    pthread_exit( NULL );
}


void send_sbd_packet(uint8_t device_id)
{
    sending_sbd_packets = true;

    static uint64_t counter = 0;

    uint8_t sbd_packet[259];

    sbd_packet[0] = 0x10;
    sbd_packet[1] = 0x53;
    sbd_packet[2] = 255;

    for (int i=0; i<255; i++) sbd_packet[i+3] = (counter++ % 254);

    // 21 bytes from latest housekeeping packet
    memcpy(sbd_packet+3, latest_housekeeping_packet+8, 2);  // sequence number
    memcpy(sbd_packet+5, latest_housekeeping_packet+16, 1);  // flag bitfield
    memcpy(sbd_packet+6, latest_housekeeping_packet+18, 2+2+4);  // CPU usage, disk usage, and uptime
    memcpy(sbd_packet+14, latest_housekeeping_packet+27, 1+4+2+1+1);  // bytes from subsystems and board temperature
    memcpy(sbd_packet+23, latest_housekeeping_packet+38, 1);  // CPU temperature

    // 14 bytes from GPS information
    memcpy(sbd_packet+24, &gps_pos.latitude, 4);
    memcpy(sbd_packet+28, &gps_pos.longitude, 4);
    memcpy(sbd_packet+32, &gps_pos.altitude, 2);
    sbd_packet[34] = gps_for_pps.day_offset;
    sbd_packet[35] = gps_pos.hour;
    sbd_packet[36] = gps_pos.minute;
    sbd_packet[37] = gps_pos.second;

    // 154 bytes = 22 bytes per imager times 7 imagers
    memcpy(sbd_packet+3+21+14, latest_imager_housekeeping, 154);
    //memset(latest_imager_housekeeping, 0, 154);

    // 54 bytes = 18 bytes per spectrometer pair times 3 spectrometer pairs
    memcpy(sbd_packet+3+21+14+154, latest_spectrometer_housekeeping, 54);
    //memset(latest_spectrometer_housekeeping, 0, 54);

    // 12 bytes of magnetometer information (excluding ADC offset)
    memcpy(sbd_packet+3+21+14+154+54, latest_magnetometer_packet+2, 12);
    //memset(latest_magnetometer_packet, 0, 18);

    sbd_packet[258] = 0x03;

    printf("Sending SBD packet to /dev/ttyS%d: ", device_id);
    for (int i=0; i<3; i++) printf("%02X ", sbd_packet[i]);
    printf("\n");
    for (int i=0; i<21; i++) printf("%02X ", sbd_packet[i+3]);
    printf("\n");
    for (int i=0; i<14; i++) printf("%02X ", sbd_packet[i+3+21]);
    printf("\n");
    for (int j=0; j<7; j++) {
        for (int i=0; i<22; i++) printf("%02X ", sbd_packet[i+3+21+14+j*22]);
        printf("\n");
    }
    for (int j=0; j<3; j++) {
        for (int i=0; i<18; i++) printf("%02X ", sbd_packet[i+3+21+14+154+j*18]);
        printf("\n");
    }
    for (int i=0; i<12; i++) printf("%02X ", sbd_packet[i+3+21+14+154+54]);
    printf("\n");

    struct pollfd serial_poll;
    serial_poll.fd = device_fd[device_id];
    serial_poll.events = POLLOUT;
    polled_write(serial_poll.fd, &serial_poll, sbd_packet, 259);
}


void *SIPParserThread(void *threadargs)
{
    Thread_data *my_data = (Thread_data *) threadargs;
    int tid = my_data->thread_id;
    uint8_t device_id = system_id_to_device_id[my_data->system_id];

    printf("SIPParser thread #%d [system 0x%02X]\n", tid, my_data->system_id);

    char packet_buffer[1024], cmd_buffer[1024];
    RingBuffer cmd_ring_buffer;

    while(!stop_message[tid])
    {
        int packet_size;
        while((packet_size = ring_buffer[device_id].smart_pop_sip(packet_buffer)) != 0) {
            if(packet_size == -1) {
                fprintf(stderr, "Skipping a byte on device %d\n", device_id);
                continue;
            }

            if(MODE_VERBOSE) {
                printLogTimestamp();
                printf("Parsed a SIP packet of %d bytes from device %d\n", packet_size, device_id);
                printf("SIP packet: ");
                for (int i=0; i<packet_size; i++) {
                    printf("0x%02X ", packet_buffer[i]);
                }
                printf("\n");
            }

            switch(packet_buffer[1]) {
                case 0x13:
                    printf("SIP has requested science data\n");
                    send_sbd_packet(device_id);

                    break;
                case 0x14:
                    cmd_ring_buffer.append(packet_buffer+3, packet_buffer[2]);
                    int cmd_packet_size;
                    while((cmd_packet_size = cmd_ring_buffer.smart_pop_booms_cmd(cmd_buffer)) == -1) {}
                    if(cmd_packet_size > 0) {
                        CommandPacket cp = CommandPacket((uint8_t *)cmd_buffer, cmd_packet_size);
                        cm_packet_queue << cp;
                    }
                    break;
            }
        }

        usleep_force(USLEEP_SERIAL_PARSER);
    }

    printf("SIPParser thread #%d exiting\n", tid);
    started[tid] = false;
    pthread_exit( NULL );
}


void pps_tick()
{
    int RTS_flag;
    RTS_flag = TIOCM_RTS;

    // Assert the RTS line
    for(int i = 0; i < 32; i++) {
        int device_id = system_id_to_device_id[SYS_ID_IMG + i]; // SYS_ID_NAI comes after SYS_ID_IMG
        if (device_id != 255) {
            ioctl(device_fd[device_id], TIOCMBIS, &RTS_flag);
        }
    }

    usleep_force(5000);

    // Clear the RTS line
    for(int i = 0; i < 32; i++) {
        int device_id = system_id_to_device_id[SYS_ID_IMG + i]; // SYS_ID_NAI comes after SYS_ID_IMG
        if (device_id != 255) {
            ioctl(device_fd[device_id], TIOCMBIC, &RTS_flag);
        }
    }
}


void *InternalPPSThread(void *threadargs)
{
    Thread_data *my_data = (Thread_data *) threadargs;
    int tid = my_data->thread_id;

    printf("InternalPPS thread #%d\n", tid);

    timespec assert_time, remaining;
    clock_gettime(CLOCK_REALTIME, &assert_time);
    assert_time.tv_sec += 1;

    uint32_t ticks = 0;

    timespec now;

    while(!stop_message[tid])
    {
        while(clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &assert_time, &remaining) == -1) {};

        if(MODE_TIMING) {
            clock_gettime(CLOCK_REALTIME, &now);
        }

        if (use_fake_pps) pps_tick();

        if(MODE_TIMING) {
            printf("PPS trigger start for %ld s (%ld us) was delayed by %f us\n",
                   assert_time.tv_sec,
                   assert_time.tv_nsec / 1000,
                   (now.tv_nsec - assert_time.tv_nsec) / 1000.);
        }

        assert_time.tv_sec += 1;

        TelemetryPacket tp_fake_pps(SYS_ID_GPS, TM_GPS_PPS, 0, current_monotonic_time());  // TODO: needs counter

        ticks++;

        tp_fake_pps << (uint8_t)(0) << (uint8_t)(0) << (uint8_t)(0) << (uint8_t)(0) << (uint32_t)(0);
        tp_fake_pps << ticks;

        if (use_fake_pps) tm_packet_queue << tp_fake_pps;
    }

    printf("InternalPPS thread #%d exiting\n", tid);
    started[tid] = false;
    pthread_exit( NULL );
}


void pps_handler(isr_info_t info)
{
    // TODO: check contents of info

    pps_tick();

    pps_received = true;

    TelemetryPacket tp_gps_pps(SYS_ID_GPS, TM_GPS_PPS, 0, current_monotonic_time());  // TODO: needs counter

    uint8_t day_offset = 0;
    uint8_t hour = 0, minute = 0, second = 0;
    int32_t clock_difference = -1000000;

    if (gps_time_received) {  // we've received at least one GPS position packet with time
        struct timespec now;
        struct tm now_tm, pps_tm, temp_tm;

        clock_gettime(CLOCK_REALTIME, &now);

        // The code below is a bit crazy
        // We use the mktime/localtime_r loop to take advantage of machinery to add time
        // We falsely treat all UTC times as local times, and we have to be careful about DST
        // Since we care about differences, all of this craziness cancels out

        // Record whether Daylight Savings Time (DST) is in effect
        localtime_r(&now.tv_sec, &now_tm);
        int isdst = now_tm.tm_isdst;

        gmtime_r(&now.tv_sec, &now_tm);
        now_tm.tm_isdst = isdst;  // UTC time would normally always not be in DST
        memcpy(&temp_tm, &now_tm, sizeof(tm));  // make a copy because mktime() can modify its argument
        time_t false_sbc_time = mktime(&temp_tm);  // false conversion as if the UTC time were local time

        // Populate the tm structure with the GPS timestamp
        memcpy(&pps_tm, &now_tm, sizeof(tm));
        pps_tm.tm_hour = gps_for_pps.hour;
        pps_tm.tm_min = gps_for_pps.minute;
        pps_tm.tm_sec = gps_for_pps.second + gps_for_pps.second_offset;

        // Run it through mktime/localtime_r to do the adding of time
        time_t false_pps_time = mktime(&pps_tm);  // false conversion as if the UTC time were local time
        localtime_r(&false_pps_time, &pps_tm);  // false conversion is reversed

        // Account for the addition of second_offset resulting in day change
        day_offset = gps_for_pps.day_offset + (pps_tm.tm_mday != now_tm.tm_mday);

        hour = pps_tm.tm_hour;
        minute = pps_tm.tm_min;
        second = pps_tm.tm_sec;

        // False conversions cancel out, but the result can be off by a full day
        int32_t diff_seconds = false_sbc_time - false_pps_time;
        if (diff_seconds > 43200) diff_seconds -= 86400;
        if (diff_seconds < -43200) diff_seconds += 86400;
        clock_difference = diff_seconds * 1e6 + now.tv_nsec / 1e3;
    }

    tp_gps_pps << day_offset;
    tp_gps_pps << hour << minute << second;
    tp_gps_pps << clock_difference;
    tp_gps_pps << gps_for_pps.second_offset;

    tm_packet_queue << tp_gps_pps;

    // Protection in case we lose GPS position packets but are still getting PPS
    gps_for_pps.second_offset++;
}


void configure_gps_device()
{
    uint8_t gps_device = system_id_to_device_id[SYS_ID_GPS];

    struct pollfd serial_poll;
    serial_poll.fd = device_fd[gps_device];
    serial_poll.events = POLLOUT;

    // Set the PPS pulse width to 500 ms
    char cmd_pps[32] = "$PTNLSPS,1,5000000,1,0*53\r\n";
    polled_write(serial_poll.fd, &serial_poll, cmd_pps, strlen(cmd_pps));

    // Set the dynamics mode to "air"
    char cmd_dynamics[64] = "$PTNLSCR,0.60,5.00,12.00,6.00,0.0000020,0,3,1,1*71\r\n";
    polled_write(serial_poll.fd, &serial_poll, cmd_dynamics, strlen(cmd_dynamics));

    // Set the NMEA packet output to GPGGA+GPVTG+GPZDA
    char cmd_output[23] = "$PTNLSNM,0025,01*50\r\n";
    polled_write(serial_poll.fd, &serial_poll, cmd_output, strlen(cmd_output));
}


void *ExternalPPSThread(void *threadargs)
{
    Thread_data *my_data = (Thread_data *) threadargs;
    int tid = my_data->thread_id;

    printf("ExternalPPS thread #%d\n", tid);

    DeviceHandle aDIO_Device;
    int aDIO_ReturnVal;
    uint8_t IntMode;

    uint32_t ticks = 0;

    // Open aDIO device
    aDIO_ReturnVal = OpenDIO_aDIO(&aDIO_Device, 0);
    if (aDIO_ReturnVal) {
        std::cerr << "ExternalPPS ERROR:  OpenDIO_aDIO(0) FAILED\n";
    } else {
        usleep_force(USLEEP_ADIO);

        // Install handler for PPS interrupts
        aDIO_ReturnVal = InstallISR_aDIO(aDIO_Device, pps_handler, SCHED_FIFO, 99);
        if (aDIO_ReturnVal) {
            std::cerr << "ExternalPPS ERROR:  InstallISR_aDIO() FAILED\n";
        } else {
            // Enable strobe mode
            aDIO_ReturnVal = EnableInterrupts_aDIO(aDIO_Device, STROBE_INT_MODE);

            usleep_force(USLEEP_ADIO);

            if (aDIO_ReturnVal) {
                RemoveISR_aDIO(aDIO_Device);
                std::cerr << "ExternalPPS ERROR:  EnableInterrupts_aDIO() FAILED\n";
            } else {
                // Check for strobe mode
                GetInterruptMode_aDIO(aDIO_Device, &IntMode);

                if (IntMode != STROBE_INT_MODE) {
                    RemoveISR_aDIO(aDIO_Device);
                    std::cerr << "ExternalPPS ERROR:  GetInterruptMode_aDIO() FAILED\n";
                } else {
                    configure_gps_device();

                    while(!stop_message[tid])
                    {
                        usleep_force(USLEEP_ADIO);

                        if (pps_received) {
                            pps_received = false;
                            ticks = 0;  // restart the watchdog timer

                            if (use_fake_pps) {
                                printLogTimestamp();
                                std::cout << "ExternalPPS: PPS pulse detected; blocking the fake PPS\n";

                                use_fake_pps = false;
                            }
                        } else {
                            if (ticks++ > 10000000 / USLEEP_ADIO) {
                                ticks = 0;  // restart the watchdog timer
                                if (!use_fake_pps) {
                                    printLogTimestamp();
                                    std::cout << "ExternalPPS: watchdog tripped; unblocking the fake PPS\n";

                                    use_fake_pps = true;
                                } else {
                                    printLogTimestamp();
                                    std::cout << "ExternalPPS: reconfiguring the GPS device\n";

                                    configure_gps_device();
                                }
                            }
                        }
                    }
                }

                // Disable interrupts
                EnableInterrupts_aDIO(aDIO_Device, DISABLE_INT_MODE);

                usleep_force(USLEEP_ADIO);
            }

            // Remove handler for PPS interrupts
            RemoveISR_aDIO(aDIO_Device);
        }

        // Close the aDIO device
        aDIO_ReturnVal = CloseDIO_aDIO(aDIO_Device);
        if (aDIO_ReturnVal) {
            std::cerr << "Error while closing ADIO = " << aDIO_ReturnVal << std::endl;
        }
    }

    use_fake_pps = true;

    printf("ExternalPPS thread #%d exiting\n", tid);
    started[tid] = false;
    pthread_exit( NULL );
}


void start_thread(void *(*routine) (void *), const Thread_data *tdata)
{
    pthread_mutex_lock(&mutexStartThread);
    int i = 0;
    while (started[i] == true) {
        i++;
        if (i == MAX_THREADS) return; //should probably thrown an exception
    }

    //Copy the thread data to a global to prevent deallocation
    if (tdata != NULL) memcpy(&thread_data[i], tdata, sizeof(Thread_data));
    thread_data[i].thread_id = i;

    stop_message[i] = false;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    int rc = pthread_create(&threads[i], &attr, routine, &thread_data[i]);
    if (rc != 0) {
        printf("ERROR; return code from pthread_create() is %d\n", rc);
    } else started[i] = true;

    pthread_attr_destroy(&attr);

    pthread_mutex_unlock(&mutexStartThread);

    return;
}

void cmd_process_command(CommandPacket &cp)
{
    std::cout << cp << std::endl;

    Thread_data tdata;
    memset(&tdata, 0, sizeof(Thread_data));
    tdata.system_id = cp.getSystemID();
    tdata.command_key = cp.getCmdType();
    cp.setReadIndex(8);
    tdata.payload_size = cp.remainingBytes();
    if (tdata.payload_size > 0) {
        cp.readNextTo_bytes(tdata.payload, tdata.payload_size);
    }

    switch(tdata.command_key)
    {
        case KEY_NULL:
            break;
        case KEY_RESTART_WORKERS:
            kill_all_workers();
            start_all_workers();
            break;
        case KEY_RESTART_ALL:
            kill_all_threads();
            start_thread(CommandListenerThread, NULL);
            start_all_workers();
            break;
        case KEY_EXIT:
            g_running = 0;
            break;
        case KEY_SHUTDOWN:
            kill_all_threads();
            usleep(USLEEP_KILL);
            system("shutdown -h now");
            break;
        default:
            start_thread(CommandHandlerThread, &tdata);
            return; //skip sending an additional ACK packet here
    } //switch
    queue_cmd_proc_ack_tmpacket(0, 0);
}

void start_all_workers()
{
    start_thread(TelemetryHousekeepingThread, NULL);
    //start_thread(TelemetryScienceThread, NULL);
    start_thread(TelemetrySenderThread, NULL);

    Thread_data tdata;
    memset(&tdata, 0, sizeof(Thread_data));

    // Open serial devices as needed
    for(int j = 0; j < 20; j++) {
        tdata.system_id = device_id_to_system_id[j];
        if(tdata.system_id != 255) {
            start_thread(SerialListenerThread, &tdata);
        }
    }

    start_thread(InternalPPSThread, NULL);
    start_thread(ExternalPPSThread, NULL);
}

int main(int argc, char *argv[])
{
    strncpy(ip_tm, IP_TM, 20);

    for(int i = 1; i < argc; i++) {
        if(argv[i][0] == '-') {
            for(int j = 1; argv[i][j] != 0; j++) {
                switch(argv[i][j]) {
                    case 'i':
                        strncpy(ip_tm, &argv[i][j+1], 20);
                        ip_tm[19] = 0;
                        j = strlen(&argv[i][0]) - 1;
                        break;
                    case 'n':
                        std::cout << "Network diagnostics mode\n";
                        MODE_NETWORK = true;
                        break;
                    case 's':
                        std::cout << "Simulated data mode\n";
                        MODE_SIMULATED_DATA = true;
                        break;
                    case 't':
                        std::cout << "Timing mode\n";
                        MODE_TIMING = true;
                        break;
                    case 'v':
                        std::cout << "Verbose mode\n";
                        MODE_VERBOSE = true;
                        break;
                    case '?':
                        std::cout << "Command-line options:\n";
                        std::cout << "-i<ip>  Send telemetry packets to this IP (instead of the FC's IP)\n";
                        std::cout << "-n      Display network packets (can be crazy!)\n";
                        std::cout << "-s      Verify simulated data in the detector packets\n";
                        std::cout << "-t      Display timing information\n";
                        std::cout << "-v      Verbose messages\n";
                        return -1;
                    default:
                        std::cerr << "Unknown option, use -? to list options\n";
                        return -1;
                }
            }
        }
    }

    // to catch a Ctrl-C or termination signal and clean up
    signal(SIGINT, &sig_handler);
    signal(SIGTERM, &sig_handler);

    // elevate privilege level
    if(iopl(3) != 0) {
        std::cerr << "Need to run with root permissions (e.g., sudo)\n";
        return -1;
    }

    // Turn off buffering of stdout
    setbuf(stdout, NULL);

    // Create the inverse map of system ID to device ID
    memset(&system_id_to_device_id, 255, sizeof(system_id_to_device_id));
    for(int j = 0; j < 20; j++) {
        if(device_id_to_system_id[j] != 255) {
            system_id_to_device_id[device_id_to_system_id[j]] = j;
        }
    }

    printLogTimestamp();
    std::cout << "Sending telemetry to " << ip_tm << std::endl;

    pthread_mutex_init(&mutexStartThread, NULL);

    /* Create worker threads */
    printLogTimestamp();
    printf("In main: creating threads\n");

    for(int i = 0; i < MAX_THREADS; i++ ){
        started[0] = false;
    }

    // start the listen for commands thread right away
    start_thread(CommandListenerThread, NULL);
    start_all_workers();

    while(g_running){
        usleep_force(USLEEP_MAIN);

        // check if new commands have been added to command queue and service them
        if (!cm_packet_queue.empty()){
            //printf("size of queue: %zu\n", cm_packet_queue.size());
            CommandPacket cp(NULL);
            cm_packet_queue >> cp;

            latest_system_id = cp.getSystemID();
            latest_command_key = cp.getCmdType();

            printLogTimestamp();
            printf("Received system ID/command key 0x%02X/0x%02X\n", latest_system_id, latest_command_key);
            cmd_process_command(cp);
        }
    }

    /* Last thing that main() should do */
    printLogTimestamp();
    printf("Quitting and cleaning up.\n");

    /* wait for threads to finish */
    kill_all_threads();
    pthread_mutex_destroy(&mutexStartThread);

    printLogTimestamp();
    printf("Done\n");

    pthread_exit(NULL);
    return 0;
}
