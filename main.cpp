#define MAX_THREADS 50
#define LOG_PACKETS true

#define LOG_LOCATION "."

//Sleep settings (microseconds)
#define USLEEP_KILL            3000000 // how long to wait before terminating threads
#define USLEEP_TM_SEND            1000 // period for popping off the telemetry queue
#define USLEEP_UDP_LISTEN         1000 // safety measure in case UDP listening is changed to non-blocking
#define USLEEP_MAIN               5000 // period for checking for new commands in the queue
#define USLEEP_SERIAL_PARSER     10000 // wait time for polling a ring buffer

//IP addresses
#define IP_FC "192.168.2.100"
#define IP_LOOPBACK "127.0.0.1"
#define IP_TM IP_LOOPBACK //default IP address unless overridden on the command line

//UDP ports, aside from PORT_IMAGE, which is TCP
#define PORT_CMD      50501 // commands, FC (receive)
#define PORT_TM       60501 // send telemetry to FC

//Acknowledgement error codes
#define ACK_NOERROR 0x00
#define ACK_BADCRC  0x03
#define ACK_BADSYS  0x04
#define ACK_BADCOM  0x05
#define ACK_NOACTION 0x10
#define ACK_BADVALUE 0x11

//BOOMS system ID
#define SYS_ID_FC  0x00
#define SYS_ID_MM  0xA0
#define SYS_ID_BGO 0xB0
#define SYS_ID_IMG 0xC0
#define SYS_ID_NAI 0xD0

//BOOMS telemetry types
#define TM_ACK          0x01

#define TM_EVENTGROUP   0xC0

//BOOMS commands, shared
#define KEY_NULL                 0x00

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

#include "UDPSender.hpp"
#include "UDPReceiver.hpp"
#include "Command.hpp"
#include "Telemetry.hpp"
#include "ring.hpp"
#include "serial.h"

//#include "main.hpp"

// global declarations
uint8_t command_sequence_number = -1;
uint8_t latest_system_id = 0xFF;
uint8_t latest_command_key = 0xFF;
float temp_py = 0, temp_roll = 0, temp_mb = 0;
char ip_tm[20];

// global mode variables
bool MODE_NETWORK = false;
bool MODE_TIMING = false;
bool MODE_VERBOSE = false;

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

//void *CommandListenerThread(void *threadargs);
//void cmd_process_command(CommandPacket &cp);
//void *CommandHandlerThread(void *threadargs);
//void queue_cmd_proc_ack_tmpacket( uint8_t error_code, uint64_t response );
//void queue_settings_tmpacket();

void *SerialListenerThread(void *threadargs);
void *SerialParserThread(void *threadargs);

void writeCurrentUT(char *buffer);
void printLogTimestamp();

template <class T>
bool set_if_different(T& variable, T value); //returns true if the value is different


uint16_t imager_counts[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint32_t imager_bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};

int device_fd[32];
RingBuffer ring_buffer[32];

uint8_t imager_to_device_map[16] = {4,  5,  6,  7,  8,  9, 10, 11};

uint8_t system_id_to_device_id(uint8_t system_id)
{
    switch(system_id & 0xF8)
    {
        case SYS_ID_IMG:
            return imager_to_device_map[system_id & 0x07];
        default:
            fprintf(stderr, "Unknown system ID\n");
            return 255;
    }
}


void sig_handler(int signum)
{
    if ((signum == SIGINT) || (signum == SIGTERM))
    {
        if (signum == SIGINT) std::cerr << "Keyboard interrupt received\n";
        if (signum == SIGTERM) std::cerr << "Termination signal received\n";
        g_running = 0;
    }
}

void writeCurrentUT(char *buffer)
{
    time_t now;
    time(&now);
    struct tm *now_tm;
    now_tm = gmtime(&now);
    strftime(buffer, 14, "%y%m%d_%H%M%S", now_tm);
}

void printLogTimestamp()
{
    time_t now;
    time(&now);
    struct tm *now_tm;
    now_tm = gmtime(&now);
    char timestamp[23];
    strftime(timestamp, 23, "%Y-%m-%d %H:%M:%S UT", now_tm);
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


uint8_t serial_packet_type(const char *ptr)
{
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

    char timestamp[14];
    char filename[128];
    std::ofstream log;

    if (LOG_PACKETS) {
        writeCurrentUT(timestamp);
        sprintf(filename, "%s/tm_%s.bin", LOG_LOCATION, timestamp);
        filename[128 - 1] = '\0';
        printf("Creating telemetry log file %s\n",filename);
        log.open(filename, std::ofstream::binary);
    }

    TelemetrySender telSender(ip_tm, (unsigned short) PORT_TM);

    while(!stop_message[tid])
    {
        usleep_force(USLEEP_TM_SEND);

        if( !tm_packet_queue.empty() ){
            TelemetryPacket tp(NULL);
            tm_packet_queue >> tp;
            telSender.send( &tp );
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

    if (LOG_PACKETS && log.is_open()) log.close();

    started[tid] = false;
    pthread_exit( NULL );
}

void *TelemetryHousekeepingThread(void *threadargs)
{
    long tid = (long)((struct Thread_data *)threadargs)->thread_id;
    printf("TelemetryHousekeeping thread #%ld!\n", tid);

    uint32_t tm_frame_sequence_number = 0;

    /*
    FILE* fp = fopen("/proc/stat", "r");
    long user1, nice1, system1, idle1;
    fscanf(fp, "%*s %ld %ld %ld %ld", &user1, &nice1, &system1, &idle1);
    fclose(fp);

    long user2, nice2, system2, idle2;

    double free0, free1, free2, min_free = 0, min_free_previous = 0, days = -1;
    */

    while(!stop_message[tid])
    {
        usleep_force(1000000);
        tm_frame_sequence_number++;

        uint16_t local_imager_counts[8], buffer_sizes[8];
        uint32_t local_imager_bytes[8];

        memcpy(&local_imager_counts, imager_counts, sizeof(imager_counts));
        memset(&imager_counts, 0, sizeof(imager_counts));
        memcpy(&local_imager_bytes, imager_bytes, sizeof(imager_bytes));
        memset(&imager_bytes, 0, sizeof(imager_bytes));

        uint8_t i;
        printLogTimestamp();
        printf("\nImager bytes:");
        for (i = 0; i < 8; i++) printf(" %d", local_imager_bytes[i]);
        printf("\nImager counts:");
        for (i = 0; i < 8; i++) printf(" %d", local_imager_counts[i]);
        printf("\nBuffer sizes:");
        for (i = 0; i < 8; i++) {
            buffer_sizes[i] = ring_buffer[imager_to_device_map[i]].size();
            printf(" %d", buffer_sizes[i]);
        }
        printf("\n");

	/*
        TelemetryPacket tp(SYS_ID_ASP, TM_HOUSEKEEPING, tm_frame_sequence_number, oeb_get_clock());

        uint8_t status_bitfield = 0;
        bitwrite(&status_bitfield, 0, 1, CAMERAS[0].Handle != NULL);
        bitwrite(&status_bitfield, 1, 1, CAMERAS[1].Handle != NULL);
        // bit 2 is connection to IR sensor
        // bit 3 is TBD
        bitwrite(&status_bitfield, 4, 1, CAMERAS[0].WantToSave);
        bitwrite(&status_bitfield, 5, 1, CAMERAS[1].WantToSave);
        bitwrite(&status_bitfield, 6, 1, MODE_DECIMATE);
        bitwrite(&status_bitfield, 7, 1, MODE_POINTING);
        tp << status_bitfield << latest_command_key;

        // Temperatures
        tp << (int16_t)(temp_py * 100) << (int16_t)(temp_roll * 100);
        
        // Estimate of days remaining of disk space
        struct statvfs vfs;
        statvfs("/data0", &vfs);
        free0 = (double)vfs.f_bavail / vfs.f_blocks;
        statvfs("/data1", &vfs);
        free1 = (double)vfs.f_bavail / vfs.f_blocks;
        statvfs("/data2", &vfs);
        free2 = (double)vfs.f_bavail / vfs.f_blocks;
        if ((tm_frame_sequence_number % 10) == 0) {
            min_free_previous = min_free;
            min_free = MIN(MIN(free0, free1), free2);
            if (min_free_previous != 0) {
                days = (10. * current_settings.cadence_housekeeping) / (min_free_previous - min_free) / 86400.;
            }
        }
        tp << (int16_t)(days * 100);

        // CPU usage
        fp = fopen("/proc/stat", "r");
        fscanf(fp, "%*s %ld %ld %ld %ld", &user2, &nice2, &system2, &idle2);
        fclose(fp);
        tp << (uint16_t)(100 * 100 * (1 - (double)(idle2 - idle1) / (user2 + nice2 + system2 + idle2 - user1 - nice1 - system1 - idle1)));
        user1 = user2;
        nice1 = nice2;
        system1 = system2;
        idle1 = idle2;

        // Disk usage
        tp << (uint16_t)(100 * 100 * (1 - free0));
        tp << (uint16_t)(100 * 100 * (1 - free1));
        tp << (uint16_t)(100 * 100 * (1 - free2));

        // Uptime
        fp = fopen("/proc/uptime", "r");
        float uptime;
        fscanf(fp, "%f", &uptime);
        fclose(fp);
        tp << (uint32_t)uptime;

        tm_packet_queue << tp;
        */
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

void *CommandListenerThread(void *threadargs)
{
    long tid = (long)((struct Thread_data *)threadargs)->thread_id;
    printf("CommandListener thread #%ld!\n", tid);

    tid_listen = tid;

    CommandReceiver comReceiver( (unsigned short) PORT_CMD);
    comReceiver.init_connection();

    // If talking to the flight computer, synchronize the clock on the odds & ends board
    // This code is here to make sure that we are ready to receive the imminent sync command
    if (strncmp(ip_tm, IP_FC, 20) == 0) {
        printLogTimestamp();
        std::cout << "Sending synchronization command to flight computer\n";
        CommandSender cmdSender(IP_FC, PORT_CMD);
        CommandPacket cp(0x0A, 0x51, 0xF0);
        cmdSender.send( &cp );
    }

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
    TelemetryPacket ack_tp(SYS_ID_ASP, TM_ACK, command_sequence_number, oeb_get_clock());
    ack_tp << error_code << response;
    tm_packet_queue << ack_tp;
}

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

void *CommandHandlerThread(void *threadargs)
{
    long tid = (long)((struct Thread_data *)threadargs)->thread_id;
    struct Thread_data *my_data;
    uint8_t error_code = 0xFF; //command not implemented
    uint64_t response = 0;
    my_data = (struct Thread_data *) threadargs;

    uint64_t value = *(uint64_t *)(my_data->payload);

    switch(my_data->system_id)
    {
        case SYS_ID_ASP:
            switch(my_data->command_key)
            {
                case KEY_POINTING_OFF: //Turn OFF pointing mode
                    if(MODE_POINTING) {
                        std::cout << "Turning OFF pointing mode\n";
                        MODE_POINTING = false;
                        error_code = 0;
                    } else {
                        error_code = ACK_NOACTION;
                    }
                    break;
                case KEY_POINTING_ON: //Turn ON pointing mode
                    if(!MODE_POINTING) {
                        std::cout << "Turning ON pointing mode\n";
                        MODE_POINTING = true;
                        error_code = 0;
                    } else {
                        error_code = ACK_NOACTION;
                    }
                    break;
                case KEY_SET_CLOCK_FOR_SYNC: //Set clock value to sync to
                    value &= 0xFFFFFFFFFFFF; //keep only 6 bytes
                    oeb_set_clock(value);
                    std::cout << "Setting clock value to sync to " << value << std::endl;
                    error_code = 0;
                    break;
                case KEY_PYC_SAVE_OFF: //Turn OFF saving of pitch-yaw images
                    CAMERAS[0].WantToSave = false;
                    std::cout << "Turning OFF saving of pitch-yaw images\n";
                    error_code = 0;
                    break;
                case KEY_PYC_SAVE_ON: //Turn ON saving of pitch-yaw images
                    CAMERAS[0].WantToSave = true;
                    std::cout << "Turning ON saving of pitch-yaw images\n";
                    error_code = 0;
                    break;
                case KEY_RC_SAVE_OFF: //Turn OFF saving of roll images
                    CAMERAS[1].WantToSave = false;
                    std::cout << "Turning OFF saving of roll images\n";
                    error_code = 0;
                    break;
                case KEY_RC_SAVE_ON: //Turn ON saving of roll images
                    CAMERAS[1].WantToSave = true;
                    std::cout << "Turning ON saving of roll images\n";
                    error_code = 0;
                    break;
                case KEY_DECIMATE_OFF: //Turn OFF image decimation
                    if(MODE_DECIMATE) {
                        std::cout << "Turning OFF image decimation\n";
                        MODE_DECIMATE = false;
                        error_code = 0;
                    } else {
                        error_code = ACK_NOACTION;
                    }
                    break;
                case KEY_DECIMATE_ON: //Turn ON image decimation
                    if(!MODE_DECIMATE) {
                        std::cout << "Turning ON image decimation\n";
                        MODE_DECIMATE = true;
                        error_code = 0;
                    } else {
                        error_code = ACK_NOACTION;
                    }
                    break;
                case KEY_TM_SEND_SETTINGS: //Request settings telemetry packet
                    queue_settings_tmpacket();
                    std::cout << "Sending settings telemetry packet\n";
                    error_code = 0;
                    break;
                case KEY_TM_CADENCE_HK: //Set cadence of housekeeping packet
                    value &= 0xFF;
                    if(value > 0) {
                        if(set_if_different(current_settings.cadence_housekeeping, (uint8_t)value)) {
                            std::cout << "Setting cadence of housekeeping packet to "
                                      << (int)current_settings.cadence_housekeeping << " s\n";
                            save_settings();
                            error_code = 0;
                        } else {
                            error_code = ACK_NOACTION;
                        }
                    } else {
                        error_code = ACK_BADVALUE;
                    }
                    break;
                case KEY_TM_CADENCE_A2D: //Set cadence of A2D temperatures packet
                    value &= 0xFF;
                    if(value > 0) {
                        if(set_if_different(current_settings.cadence_a2d, (uint8_t)value)) {
                            std::cout << "Setting cadence of A2D temperatures packet to "
                                      << (int)current_settings.cadence_a2d << " s\n";
                            save_settings();
                            error_code = 0;
                        } else {
                            error_code = ACK_NOACTION;
                        }
                    } else {
                        error_code = ACK_BADVALUE;
                    }
                    break;
                case KEY_TM_CADENCE_SCIENCE: //Set cadence of science packet
                    value &= 0xFF;
                    if(value > 0) {
                        if(set_if_different(current_settings.cadence_science, (uint8_t)value)) {
                            std::cout << "Setting cadence of science packet to "
                                      << (int)current_settings.cadence_science << " s\n";
                            save_settings();
                            error_code = 0;
                        } else {
                            error_code = ACK_NOACTION;
                        }
                    } else {
                        error_code = ACK_BADVALUE;
                    }
                    break;
                case KEY_LOAD_PARAMETERS: //Load parameter table
                    value &= 0xFF;
                    if(value != current_table) {
                        std::cout << "Loading table " << (int)value << std::endl;
                        if(load_settings(value) == 0) {
                            queue_settings_tmpacket();
                            synchronize_settings();
                            error_code = arm_timer();
                        } else {
                            error_code = ACK_BADVALUE;
                        }
                    } else {
                        error_code = ACK_NOACTION;
                    }
                    //note that settings are not saved at this point to enable an undo
                    break;
                default:
                    error_code = ACK_BADCOM; //unknown command
                    response = my_data->command_key;
            } //switch for command key
            break;
        case SYS_ID_PYC:
            switch(my_data->command_key & 0xF)
            {
                case KEY_CAMERA_FPS: //Set FPS
                    value &= 0xFF;
                    if(value > 0 && value <= 10) {
                        if(set_if_different(current_settings.PY_rate, (uint8_t)value)) {
                            synchronize_settings();
                            std::cout << "Setting pitch-yaw camera rate to "
                                      << (int)current_settings.PY_rate << " Hz\n";
                            error_code = arm_timer();
                            if(error_code == 0) save_settings();
                        } else {
                            error_code = ACK_NOACTION;
                        }
                    } else {
                        error_code = ACK_BADVALUE;
                    }
                    break;
                case KEY_CAMERA_GAIN: //Set gain
                    value &= 0xFF;
                    if(set_if_different(current_settings.PY_gain, (uint8_t)value)) {
                        synchronize_settings();
                        std::cout << "Setting pitch-yaw camera gain to "
                                  << (int)current_settings.PY_gain << " dB\n";
                        save_settings();
                        error_code = 0;
                    } else {
                        error_code = ACK_NOACTION;
                    }
                    break;
                case KEY_CAMERA_EXPOSURE: //Set exposure
                    value &= 0xFFFF;
                    if(set_if_different(current_settings.PY_exposure, (uint16_t)value)) {
                        synchronize_settings();
                        std::cout << "Setting pitch-yaw camera exposure to "
                                  << (int)current_settings.PY_exposure << " us\n";
                        save_settings();
                        error_code = 0;
                    } else {
                        error_code = ACK_NOACTION;
                    }
                    break;
                case KEY_CAMERA_SEND_LAST: //Send latest image
                    TRANSMIT_NEXT_PY_IMAGE = true;
                    std::cout << "Sending latest pitch-yaw image\n";
                    error_code = 0;
                    break;
                case KEY_CAMERA_SEND_SPECIFIC: //Send specific image
                    break;
                default:
                    error_code = ACK_BADCOM; //unknown command
                    response = my_data->command_key;
            } //switch for command key
            if(error_code == 0) queue_settings_tmpacket();
            break;
        case SYS_ID_RC:
            switch(my_data->command_key & 0xF)
            {
                case KEY_CAMERA_FPS: //Set FPS
                    value &= 0xFF;
                    if(value > 0 && value <= 10) {
                        current_settings.R_rate = value;
                        synchronize_settings();
                        std::cout << "Setting roll camera rate to "
                                  << (int)current_settings.R_rate << " Hz\n";
                        error_code = arm_timer();
                        if(error_code == 0) save_settings();
                    } else {
                        error_code = ACK_BADVALUE;
                    }
                    break;
                case KEY_CAMERA_GAIN: //Set gain
                    value &= 0xFF;
                    if(set_if_different(current_settings.R_gain, (uint8_t)value)) {
                        synchronize_settings();
                        std::cout << "Setting roll camera gain to "
                                  << (int)current_settings.R_gain << " dB\n";
                        save_settings();
                        error_code = 0;
                    } else {
                        error_code = ACK_NOACTION;
                    }
                    break;
                case KEY_CAMERA_EXPOSURE: //Set exposure
                    value &= 0xFFFF;
                    if(set_if_different(current_settings.R_exposure, (uint16_t)value)) {
                        synchronize_settings();
                        std::cout << "Setting roll camera exposure to "
                                  << (int)current_settings.R_exposure << " us\n";
                        save_settings();
                        error_code = 0;
                    } else {
                        error_code = ACK_NOACTION;
                    }
                    break;
                case KEY_CAMERA_SEND_LAST: //Send latest image
                    TRANSMIT_NEXT_R_IMAGE = true;
                    std::cout << "Sending latest roll image\n";
                    error_code = 0;
                    break;
                case KEY_CAMERA_SEND_SPECIFIC: //Send specific image
                    break;
                default:
                    error_code = ACK_BADCOM; //unknown command
                    response = my_data->command_key;
            } //switch for command key
            if(error_code == 0) queue_settings_tmpacket();
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
*/

void *SerialListenerThread(void *threadargs)
{
    Thread_data *my_data = (Thread_data *) threadargs;
    int tid = my_data->thread_id;
    uint8_t device_id = system_id_to_device_id(my_data->system_id);

    printf("SerialListener thread #%d [system 0x%02X]\n", tid, my_data->system_id);

    device_fd[device_id] = setup_serial_port(device_id);  // TODO: close this

    printLogTimestamp();
    printf("Device /dev/ttyS%d opened for system 0x%02X\n", device_id, my_data->system_id);

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
                printf("Read %d bytes from device %d\n", c, device_id);
            }
            ring_buffer[device_id].append(read_buffer, c);

            imager_bytes[my_data->system_id & 0x07] += c;
        }
    }

    printf("SerialListener thread #%d [device %d] exiting\n", tid, device_id);
    started[tid] = false;
    pthread_exit( NULL );
}


void *SerialParserThread(void *threadargs)
{
    Thread_data *my_data = (Thread_data *) threadargs;
    int tid = my_data->thread_id;
    uint8_t device_id = system_id_to_device_id(my_data->system_id);

    printf("SerialParser thread #%d [system 0x%02X]\n", tid, my_data->system_id);

    char packet_buffer[1024];

    TelemetryPacket tp_eventgroup(my_data->system_id, TM_EVENTGROUP, 0, current_monotonic_time());  // TODO: needs counter
    int events_in_group = 0;

    int last_number = -1;

    while(!stop_message[tid])
    {
        int packet_size;
        while((packet_size = ring_buffer[device_id].smart_pop(packet_buffer)) != 0) {
            if(packet_size == -1) {
                fprintf(stderr, "Skipping a byte on device %d\n", device_id);
                continue;
            }

            if(MODE_VERBOSE) {
                printLogTimestamp();
                printf("Parsed a serial packet of %d bytes from device %d\n", packet_size, device_id);
            }

            uint8_t packet_type = serial_packet_type(packet_buffer);
            if(packet_type == 0b000) {  // event packets are grouped
                events_in_group++;
                imager_counts[my_data->system_id & 0b111]++;
                tp_eventgroup.append_bytes(packet_buffer, packet_size);

                int number;
                memcpy(&number, packet_buffer + 2, sizeof(number));

                if(number != last_number + 1) {
                    fprintf(stderr, "Desync with %d dropped packets (%d, %d)\n", number - last_number - 1, last_number, number);
                }
                last_number = number;

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
            } else {
                fprintf(stderr, "Unknown serial packet of type 0b%d%d%d\n", packet_type & 0b100 >> 2, packet_type & 0b010 >> 1, packet_type & 0b001);
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

    printf("SerialParser thread #%d [device %d] exiting\n", tid, device_id);
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

/*
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
*/

void start_all_workers()
{
    start_thread(TelemetryHousekeepingThread, NULL);
    //start_thread(TelemetryScienceThread, NULL);
    start_thread(TelemetrySenderThread, NULL);

    Thread_data tdata;
    memset(&tdata, 0, sizeof(Thread_data));

    // Imager threads
    for(int i = 0; i < 8; i++) {
        tdata.system_id = SYS_ID_IMG + i;
        start_thread(SerialListenerThread, &tdata); 
        start_thread(SerialParserThread, &tdata);
    }
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
                        std::cout << "-t      Perform timing tests\n";
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
    //start_thread(CommandListenerThread, NULL);
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
            //cmd_process_command(cp);
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
