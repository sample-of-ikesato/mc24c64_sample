#ifndef _queue_h_
#define _queue_h_

typedef struct Queue_t {
  unsigned char *head;  // enqueue position
  unsigned char *tail;  // dequeue position
  unsigned char *buffer;
  int buffer_size;      // need store size + 1
} Queue;

// initilize Queue object
void queue_init(Queue *queue, void *buffer, int max_size);

// get current queue size, less equal than buffer_size-1
int queue_size(Queue *queue);

// enqueu buffer
// return 0:success -1:buffer over flow
int queue_enqueue(Queue *queue, void *buffer, int size);

// dequeue buffer
// return readed buffer size (less than argument size)
int queue_dequeue(Queue *queue, void *buffer, int size);

// peek queue
// return queue data
unsigned char queue_peek(Queue *queue, int pos);

// clear queue
void queue_clear(Queue *queue);

#endif//_queue_h_
