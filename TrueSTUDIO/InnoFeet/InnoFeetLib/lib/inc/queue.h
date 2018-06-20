

#ifndef INNOFEET_QUEUE_H
#define INNOFEET_QUEUE_H

#include "globals.h"


typedef struct _Queue Queue;

Queue* queue_init(unsigned int max_len, unsigned int item_size);
int queue_delete(Queue*);

int queue_push(Queue*, byte* data);
int queue_pop(Queue*, byte* data);
int queue_count(Queue*);

#endif //INNOFEET_QUEUE_H
