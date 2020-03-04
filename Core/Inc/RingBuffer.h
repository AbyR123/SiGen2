#include  <stdint.h>

#define RING_BUFFER_SIZE 256

typedef struct
{
    volatile uint16_t write_cursor;
    volatile uint16_t read_cursor;
    volatile unsigned char data[RING_BUFFER_SIZE];
} ring_buffer_t;

void RingBuffer_write( ring_buffer_t *rb, unsigned char data );
unsigned char RingBuffer_read( ring_buffer_t *rb );
unsigned char RingBuffer_isEmpty( ring_buffer_t *rb );
void RingBuffer_init( ring_buffer_t *rb );

