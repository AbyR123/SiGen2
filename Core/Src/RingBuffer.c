#include "RingBuffer.h"

void RingBuffer_write( ring_buffer_t * rb, unsigned char data )
{
	uint16_t write_cursor;
	write_cursor = rb->write_cursor;
    rb->data[write_cursor++] = data;
    if ( write_cursor >= RING_BUFFER_SIZE ) {
    	write_cursor = 0;
    }

    rb->write_cursor = write_cursor;
}

unsigned char RingBuffer_read( ring_buffer_t * rb )
{
    uint16_t read_cursor;
    unsigned char ret;

    read_cursor = rb->read_cursor;

    ret = rb->data[read_cursor++];
    if (read_cursor >= RING_BUFFER_SIZE)
    {
    	read_cursor = 0;
    }
    rb->read_cursor = read_cursor;

    return ret;
}

unsigned char RingBuffer_isEmpty( ring_buffer_t *rb )
{
	if(rb->write_cursor == rb->read_cursor){
		return 1;
	}
	else{
		return 0;
	}
}

void RingBuffer_init( ring_buffer_t *rb )
{
    rb->write_cursor = 0;
    rb->read_cursor  = 0;
}
