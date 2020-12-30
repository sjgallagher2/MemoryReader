#include "circbuf.h"

/* FOR DEBUG ONLY */
#ifdef CBUF_DEBUG
    #include <stdio.h>
    #include <stdint.h>
#endif
/******************/



/* CIRCULAR BUFFER DECLARATIONS */
circbuf _cb1;
circbuf_handler circular_buffer_1 = &_cb1; // Initialize circular buffer


void init_circbuf(circbuf_handler cb, uint8_t* buf, uint32_t size)
{
    cb->data = buf;
    cb->capacity = (uint32_t)size;
    circbuf_reset(cb);
}

void circbuf_reset(circbuf_handler cb)
{
    uint32_t i;
    for(i=0;i<cb->capacity;i++)
        cb->data[i] = 0;
    cb->head = 0;
    cb->tail = 0;
}

circbuf_status circbuf_push(circbuf_handler cb, uint8_t wbyte)
{
    // Need to determine whether or not the next space is available, 
    // updating if we go over the capacity
    uint32_t next_head = (cb->head+1) % cb->capacity;
    if(cb->full)
    {
        return CIRCBUF_STATUS_FULL;
    }
    else if(next_head == cb->tail)
    { // If we're going to overlap the tail next step (overflow)
        cb->full = 1;
        cb->data[cb->head] = wbyte;
        cb->head = next_head;
        return CIRCBUF_STATUS_WARN;
    }
    else
    { // We aren't overlapping the tail (no overflow condition)
        cb->full = 0;
        cb->data[cb->head] = wbyte;
        cb->head = next_head;
        return CIRCBUF_STATUS_OK;
    }
}

circbuf_status circbuf_pop(circbuf_handler cb, uint8_t* rbyte)
{
    uint32_t next_tail = (cb->tail+1) % cb->capacity;
    if(cb->tail == cb->head && !cb->full)
    { // Tail is caught up to head (empty condition)
        return CIRCBUF_STATUS_EMPTY;
    }
    else
    {
        *rbyte = cb->data[cb->tail];
        cb->tail = next_tail;
        if(cb->full)
        {
            cb->full = 0;
        }
        return CIRCBUF_STATUS_OK;
    }
}

circbuf_status circbuf_get_status(circbuf_handler cb)
{
    if(cb->full)
    {
        return CIRCBUF_STATUS_FULL; 
    }
    else if( cb->head == cb->tail )
    {
        return CIRCBUF_STATUS_EMPTY;
    }
    else
    {
        return CIRCBUF_STATUS_OK;
    }
}

uint32_t circbuf_get_capacity(circbuf_handler cb)
{
    return cb->capacity;
}

uint32_t circbuf_get_free_space(circbuf_handler cb)
{
    // head can lead tail, tail can't lead head. If head > tail, take 
    // capacity - separation; if head < tail, take separation
    if(cb->full)
    {
        return 0;
    }
    else if(cb->tail == cb->head)
    {
        return cb->capacity;
    }
    else if(cb->head > cb->tail)
    {
        return ( cb->capacity - (cb->head - cb->tail) );
    }
    else if(cb->head < cb->tail)
    {
        return (cb->tail - cb->head);
    }

    return 0; // Default
}

uint32_t circbuf_get_size(circbuf_handler cb)
{
    return cb->capacity - circbuf_get_free_space(cb);
}

#ifdef CBUF_DEBUG
void circbuf_print(circbuf_handler cb)
{
    uint32_t i;
    printf("CIRCULAR BUFFER CONTENTS\n");
    for(i=0; i<cb->capacity; i++)
    {
        printf("0x%.2x\t",cb->data[i]);
    }
    printf("\n");
    for(i=0; i<cb->capacity; i++)
    {
        if(i == cb->head && i != cb->tail)
            printf("H\t");
        else if(i == cb->tail && i != cb->head)
            printf("T\t");
        else if(i == cb->tail && i == cb->head)
            printf("T/H\t");
        else
            printf("\t");
    }
    printf("\n");
    
}
#endif


