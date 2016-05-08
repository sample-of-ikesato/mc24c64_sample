#include <stdio.h>
#include "queue.h"


void queue_init(Queue *q, void *buffer, int max_size)
{
  q->buffer = buffer;
  q->buffer_size = max_size;
  q->head = buffer;
  q->tail = buffer;
}

int queue_size(Queue *q)
{
  if (q->head >= q->tail)
    return q->head - q->tail;
  else
    return q->buffer_size - (q->tail - q->head);
}

int queue_enqueue(Queue *q, void *buffer, int size)
{
  unsigned char *bp = buffer;
  int i;
  int ret = 0;

  for (i=0; i<size; i++) {
    *q->head++ = *bp++;
    if (q->head >= q->buffer + q->buffer_size) {
      q->head = q->buffer;
    }
    if (q->head == q->tail) {
      ret = -1;
      if (q->tail < q->buffer + q->buffer_size -1)
        q->tail++;
      else
        q->tail = q->buffer;
    }
  }
  return ret;
}

int queue_dequeue(Queue *q, void *buffer, int size)
{
  unsigned char *bp = buffer;
  int i;
  for (i=0; i<size; i++) {
    if (q->tail == q->head)
      break;
    if (bp != 0)
      bp[i] = *q->tail;
    q->tail++;
    if (q->tail >= q->buffer + q->buffer_size) {
      q->tail = q->buffer;
    }
  }
  return i;
}

unsigned char queue_peek(Queue *q, int pos)
{
  unsigned char *qp = q->tail + pos;
  if (qp >= q->buffer + q->buffer_size) {
    qp -= q->buffer_size;
  }
  return *qp;
}

// clear queue
void queue_clear(Queue *q)
{
  q->tail = q->head;
}
