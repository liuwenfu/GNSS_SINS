#pragma once
#include "rawdecode.h"

typedef IMUdata Elemtype;

typedef struct Node
{
	Elemtype imudata;
	struct Node *next;          // 链式队列中结点元素的指针
} QNode, *QueuePtr;

typedef struct                   //观测值队列
{
	QueuePtr front;             // 队列头指针
	QueuePtr rear;              // 队列尾指针
} LinkQueue;

bool InitQueue(LinkQueue *Q);
void DestroyQueue(LinkQueue *Q);
bool QueueEmpty(LinkQueue Q);
int QueueLength(LinkQueue Q);
bool GetHead(LinkQueue Q, Elemtype *e);
void QueueTraverse(LinkQueue QO, void(*fp)(Elemtype));
void ClearQueue(LinkQueue *Q);
bool EnQueue(LinkQueue *Q, Elemtype e);
bool DeQueue(LinkQueue *Q, Elemtype *e);