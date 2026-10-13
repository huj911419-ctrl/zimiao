#ifndef TEST_MESSAGE_CENTER_H
#define TEST_MESSAGE_CENTER_H
#include <stdint.h>
typedef struct { uint8_t size; } Publisher_t;
Publisher_t *PubRegister(char *name, uint8_t size);
uint8_t PubPushMessage(Publisher_t *pub, void *data);
typedef struct { uint8_t size; } Subscriber_t;
Subscriber_t *SubRegister(char *name, uint8_t size);
uint8_t SubGetMessage(Subscriber_t *sub, void *data);
#endif
