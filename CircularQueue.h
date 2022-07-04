/*
 * CircularQueue.h
 *
 *  Created on: 2022. 4. 15.
 *      Author: hc.ro
 */

#ifndef CIRCULARQUEUE_H_
#define CIRCULARQUEUE_H_

#define TRUE 1
#define FALSE 0


#define QUE_LEN 11                  // Packet byte (data length)
typedef unsigned char Data;

typedef struct _cQueue
{
    int front;
    int rear;
    Data queArr[QUE_LEN];
    int state;                      // Queue state Empty or full
} CQueue;

typedef CQueue Queue;

void QueueInit(Queue *pq);
int QIsEmpty(Queue *pq);

void Enqueue(Queue *pq, Data data);
Data Dequeue(Queue *pq);
Data QPeek(Queue *pq);

#endif /* CIRCULARQUEUE_H_ */
