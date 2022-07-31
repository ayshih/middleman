#ifndef _RING_HPP_
#define _RING_HPP_

#include <stdint.h>

#define BUFFER_SIZE 100000

extern uint8_t packet_size_by_id[8];

class RingBuffer {
    private:
        uint8_t buffer[BUFFER_SIZE];
        uint32_t read_index, write_index;

    public:
        RingBuffer();

        void append(const void *ptr, uint16_t num);
        int32_t pop(void *ptr, uint16_t num);
        int32_t peek(void *ptr, uint16_t num);
        uint32_t size();

        // UW instruments
        int32_t smart_pop_imager(void *ptr);
        int32_t smart_pop_spectrometer(void *ptr);

        // GPS device
        int32_t smart_pop_nmea(void *ptr);

        // SIP command packets
        int32_t smart_pop_sip(void *ptr);

        // Reconstructed BOOMS command packets
        int32_t smart_pop_booms_cmd(void *ptr);

        // MAG device
        int32_t smart_pop_magnetometer(void *ptr);

        void clear();
};

#endif
