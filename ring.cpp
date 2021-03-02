#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ring.hpp"

uint8_t packet_size_by_id[8] = {7, 11, 11, 11, 11, 8, 18, 10};

RingBuffer::RingBuffer()
{
    this->clear();
}

void RingBuffer::append(const void *ptr, uint16_t num)
{
    int32_t num_overflow = (this->write_index + num) - BUFFER_SIZE;
    if (num_overflow < 0) {
        memcpy(this->buffer + this->write_index, ptr, num);
        this->write_index += num;
    } else {
        memcpy(this->buffer + this->write_index, ptr, num - num_overflow);
        memcpy(this->buffer, (uint8_t *)ptr + num - num_overflow, num_overflow);
        this->write_index = num_overflow;
    }

    // TODO: detect lapping the read pointer
}

int32_t RingBuffer::peek(void *ptr, uint16_t num)
{
    int32_t num_overflow = (this->read_index + num) - BUFFER_SIZE;
    if (num_overflow < 0) {
        memcpy(ptr, this->buffer + this->read_index, num);
    } else {
        memcpy(ptr, this->buffer + this->read_index, num - num_overflow);
        memcpy((uint8_t *)ptr + num - num_overflow, this->buffer, num_overflow);
    }

    // TODO: detect lapping the write pointer

    return num;
}

int32_t RingBuffer::pop(void *ptr, uint16_t num)
{
    int32_t result = peek(ptr, num);

    if (result == num) {
        int32_t num_overflow = (this->read_index + num) - BUFFER_SIZE;
	if (num_overflow < 0) {
            this->read_index += num;
        } else {
            this->read_index = num_overflow;
	}
    }

    return result;
}

int32_t RingBuffer::smart_pop(void *ptr)
{
    // local copy since there may be active writing
    uint32_t this_write_index = this->write_index;

    uint32_t this_size = (BUFFER_SIZE + this_write_index - this->read_index) % BUFFER_SIZE;

    if(this_size == 0) return 0;

    //printf("R/W: %d %d\n", this->read_index, this_write_index);

    uint8_t byte1 = this->buffer[this->read_index];
    uint8_t byte2 = this->buffer[(this->read_index + 1) % BUFFER_SIZE];

    if((byte1 & 0b11111100) == 0b10101100) {
        uint8_t id = ((byte1 & 0b11) << 1) | (byte2 >> 7);
        if(id < 8) {
            uint8_t packet_size = packet_size_by_id[id];
            if(this_size < packet_size) return 0;
            return pop(ptr, packet_size);
       }
    }

    // Not a valid header, so advance one byte and return an error
    this->read_index = (this->read_index + 1) % BUFFER_SIZE;
    return -1;
}

int32_t RingBuffer::smart_pop_nmea(void *ptr)
{
    // local copy since there may be active writing
    uint32_t this_write_index = this->write_index;

    uint32_t this_size = (BUFFER_SIZE + this_write_index - this->read_index) % BUFFER_SIZE;

    if(this_size == 0) return 0;

    uint8_t new_buffer[BUFFER_SIZE];
    peek(new_buffer, this_size);

    //printf("R/W: %d %d\n", this->read_index, this_write_index);

    if (new_buffer[0] == '$') {
	for(uint32_t i = 1; i < this_size; i++) {
	    switch(new_buffer[i]) {
		case '\n': // found end of NMEA packet
		    return pop(ptr, i+1);
		case '$': // found start of next NMEA packet
		    this->read_index = (this->read_index + i) % BUFFER_SIZE;
		    return -1;
            }
	}
	return 0; // still waiting for end of NMEA packet
    }

    // Not the start of a NMEA packet, so advance one byte and return an error
    this->read_index = (this->read_index + 1) % BUFFER_SIZE;
    return -1;
}

uint32_t RingBuffer::size()
{
    return (uint32_t)((this->write_index - this->read_index) % BUFFER_SIZE);
}

void RingBuffer::clear()
{
    this->read_index = 0;
    this->write_index = 0;
}
