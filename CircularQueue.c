/*
 * CircularQueue.c
 *
 *  Created on: 2022. 4. 15.
 *      Author: hc.ro
 */

#ifndef CIRCULARQUEUE_C_
#define CIRCULARQUEUE_C_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "CircularQueue.h"

void QueueInit(Queue *pq)
{
    pq->front = 0;
    pq->rear = 0;
    pq->state = 0;
    //pq->[QUE_LEN] = {0, };
}

int QIsEmpty(Queue *pq)
{
    if (pq->front == pq->rear)
        return TRUE;

    else
        return FALSE;
}

int NextPosIdx(int pos)
{
    if (pos == QUE_LEN - 1)
        return 0;

    else
        return pos + 1;
}

void Enqueue(Queue *pq, Data data)
{
    if (NextPosIdx(pq->rear) == pq->front)
    {
        pq->state = 1;                          // Queue is full
    }

    pq->rear = NextPosIdx(pq->rear);
    pq->queArr[pq->rear] = data;
}

Data Dequeue(Queue *pq)
{
    if (QIsEmpty(pq))
    {
        pq->state = 0;
    }

    pq->front = NextPosIdx(pq->front);
    return pq->queArr[pq->front];
}

Data QPeek(Queue *pq)
{
    if (QIsEmpty(pq))
    {
        pq->state = 0;
    }

    return pq->queArr[NextPosIdx(pq->front)];
}

#endif /* CIRCULARQUEUE_C_ */
