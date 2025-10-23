#include "ringbuf.h"

#include <string.h>

#include "lock.h"

#define LOCK(rb) \
    bool was_locked = lock(rb)

#define UNLOCK(rb) \
    unlock(rb, was_locked)

static bool lock(ringbuf_t* rb)
{
    if (rb->use_lock) {
        return lock_enter();
    }

    return false;
}

static void unlock(ringbuf_t* rb, bool interrupts_were_disabled)
{
    if (rb->use_lock) {
        lock_exit(interrupts_were_disabled);
    }
}

void ringbuf_init(ringbuf_t* rb)
{
    ringbuf_reset(rb);
}

void ringbuf_reset(ringbuf_t* rb)
{
    LOCK(rb);
    {
        rb->write_index = 0;
        rb->read_index = 0;
        rb->full = false;
    }
    UNLOCK(rb);
}

bool ringbuf_empty(ringbuf_t* rb)
{
    bool empty;

    LOCK(rb);
    {
        empty = (rb->write_index == rb->read_index) && !rb->full;
    }
    UNLOCK(rb);

    return empty;
}

bool ringbuf_full(ringbuf_t* rb)
{
    bool full;

    LOCK(rb);
    {
        full = rb->full;
    }
    UNLOCK(rb);

    return full;
}

uint32_t ringbuf_bytesFree(ringbuf_t* rb)
{
    uint32_t bytes_free = 0;

    LOCK(rb);
    {
        if (ringbuf_full(rb)) {
            goto unlock;
        }

        if (rb->write_index >= rb->read_index) {
            bytes_free = rb->buffer_capacity - (rb->write_index - rb->read_index);
        } else {
            bytes_free = rb->read_index - rb->write_index;
        }
    }
unlock:
    UNLOCK(rb);

    return bytes_free;
}

uint32_t ringbuf_bytesUsed(ringbuf_t* rb)
{
    uint32_t bytes_used;

    LOCK(rb);
    {
        bytes_used = rb->buffer_capacity - ringbuf_bytesFree(rb);
    }
    UNLOCK(rb);

    return bytes_used;
}

static void copyDataToBuffer(ringbuf_t* rb, const uint8_t* data, uint32_t data_length)
{
    memcpy(&rb->buffer[rb->write_index], data, data_length);
    rb->write_index = (rb->write_index + data_length) % rb->buffer_capacity;
}

uint32_t ringbuf_put(ringbuf_t* rb, const uint8_t* data, uint32_t data_length)
{
    uint32_t bytes_to_write = data_length;

    LOCK(rb);
    {
        uint32_t bytes_free = ringbuf_bytesFree(rb);
        if (bytes_to_write > bytes_free) {
            bytes_to_write = bytes_free;
        }

        if (bytes_to_write == 0) {
            goto unlock;
        }

        uint32_t first_write_length;
        if (rb->write_index >= rb->read_index) {
            first_write_length = rb->buffer_capacity - rb->write_index;
            if (first_write_length > bytes_to_write) {
                first_write_length = bytes_to_write;
            }
        } else {
            first_write_length = bytes_to_write;
        }

        copyDataToBuffer(rb, data, first_write_length);

        if (first_write_length < bytes_to_write) {
            uint32_t second_write_length = bytes_to_write - first_write_length;
            copyDataToBuffer(rb, &data[first_write_length], second_write_length);
        }

        if (rb->write_index == rb->read_index) {
            rb->full = true;
        }
    }
unlock:
    UNLOCK(rb);

    return bytes_to_write;
}

uint8_t* ringbuf_contiguousPeek(ringbuf_t* rb, uint32_t *data_length)
{
    uint8_t *data;

    LOCK(rb);
    {
        data = &rb->buffer[rb->read_index];

        if (ringbuf_empty(rb)) {
            *data_length = 0;
        } else if (rb->read_index < rb->write_index) {
            *data_length = rb->write_index - rb->read_index;
        } else {
            *data_length = rb->buffer_capacity - rb->read_index;
        }
    }
    UNLOCK(rb);

    return data;
}

void ringbuf_pop(ringbuf_t* rb, uint32_t bytes_to_pop)
{
    LOCK(rb);
    {
        if (bytes_to_pop > ringbuf_bytesUsed(rb)) {
            goto unlock;
        }

        rb->read_index = (rb->read_index + bytes_to_pop) % rb->buffer_capacity;
        rb->full = false;
    }
unlock:
    UNLOCK(rb);
}
