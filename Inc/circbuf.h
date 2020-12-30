#ifndef SAMLIB_CIRCBUF_H
#define SAMLIB_CIRCBUF_H 

#include <stdint.h>

// Header for circular buffer data structure

typedef struct{
    uint32_t head;  // Next free space
    uint32_t tail;  // Current element, to be popped
    uint32_t capacity; // Max buffer size
    uint8_t full;   // Whether or not the buffer is full (0 or 1)
    uint8_t* data;  // Actual buffer
} circbuf;

// Make a typedef for a handler to a circbuf pointer, so users don't think they're supposed to dereference it
typedef circbuf* circbuf_handler;
typedef uint8_t circbuf_status;

#define CIRCBUF_STATUS_OK           0
#define CIRCBUF_STATUS_FULL         1
#define CIRCBUF_STATUS_EMPTY        2
#define CIRCBUF_STATUS_WARN         3
// Warn = last byte written, now buffer is full

// Implementation functions
void init_circbuf(circbuf_handler cb, uint8_t* buf, uint32_t size); // Initialize circular buffer with buffer and size
void circbuf_reset(circbuf_handler cb); // Clear buffer, set head = tail
circbuf_status circbuf_push(circbuf_handler cb, uint8_t wbyte); // Push len bytes to buffer, return status
circbuf_status circbuf_pop(circbuf_handler cb, uint8_t* rbyte); // Pop len bytes from buffer to rbyte, return status
circbuf_status circbuf_get_status(circbuf_handler cb); // Return a status value
uint32_t circbuf_get_capacity(circbuf_handler cb); // Return capacity (full size) of buffer
uint32_t circbuf_get_size(circbuf_handler cb); // Return number of elements in buffer
uint32_t circbuf_get_free_space(circbuf_handler cb); // Return number of free spaces

/* FOR DEBUGGING ONLY */
#ifdef CBUF_DEBUG
    void circbuf_print(circbuf_handler cb);
#endif

#endif /* SAMLIB_CIRCBUF_H */


