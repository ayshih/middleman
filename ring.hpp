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
        uint32_t size();

	int32_t smart_pop(void *ptr);

        void clear();
};

#endif
