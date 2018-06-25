
#include <stdlib.h>
#include <memory.h>

#include "queue.h"

typedef struct _Queue {
    unsigned int max_len, item_size, item_count;
    unsigned int head, tail;
    byte* data;
};

Queue* queue_init(unsigned int max_len,  unsigned int item_size) {
    Queue* queue = malloc(sizeof(*queue));
    queue->max_len = max_len;
    queue->item_size = item_size;
    queue-> head = 0;
    queue->tail = 0;
    queue->item_count = 0;
    queue->data = malloc(max_len * item_size);
    return queue;
}

int queue_delete(Queue* self) {
    free(self->data);
    free(self);
}

int queue_push(Queue* self, byte* data) {
    if(self->item_count < self->max_len) {
        memcpy(self->data + self->head, data, self->item_size);
        self->item_count++;
        self->head += self->item_size;
        self->head %= self->max_len * self->item_size;
        return SUCCESS;
    }
    else return ERROR;
}

int queue_pop(Queue* self, byte* data) {
    if(self->item_count > 0) {
        memcpy(data, self->data + self->tail, self->item_size);
        self->item_count--;
        self->tail += self->item_size;
        self->tail %= self->max_len * self->item_size;
        return SUCCESS;
    }
    else return ERROR;
}

int queue_count(Queue* self) {
    return (int)self->item_count;
}
