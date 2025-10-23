#ifndef __RINGBUF_H
#define __RINGBUF_H

#include <stdbool.h>
#include <stdint.h>

#define RINGBUF_DEFINE(name, capacity, locked)      \
    static uint8_t name##_backing_buffer[capacity]; \
    static ringbuf_t name = {                       \
        .buffer = name##_backing_buffer,            \
        .buffer_capacity = capacity,                \
        .use_lock = locked                          \
    };

typedef struct {
    uint8_t* buffer;
    uint32_t buffer_capacity;
    bool use_lock;
    uint32_t write_index;
    uint32_t read_index;
    bool full;
} ringbuf_t;

void ringbuf_init(ringbuf_t* rb);
void ringbuf_reset(ringbuf_t* rb);

bool ringbuf_empty(ringbuf_t* rb);
bool ringbuf_full(ringbuf_t* rb);
uint32_t ringbuf_bytesFree(ringbuf_t* rb);
uint32_t ringbuf_bytesUsed(ringbuf_t* rb);

uint32_t ringbuf_put(ringbuf_t* rb, const uint8_t* data, uint32_t data_length);
uint8_t* ringbuf_contiguousPeek(ringbuf_t* rb, uint32_t* data_length);
void ringbuf_pop(ringbuf_t* rb, uint32_t bytes_to_pop);

#endif // __RINGBUF_H
