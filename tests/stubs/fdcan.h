#ifndef TEST_FDCAN_H
#define TEST_FDCAN_H
#include "main.h"
typedef struct { int unused; } FDCAN_TxHeaderTypeDef;
typedef struct { uint32_t BusOff; } FDCAN_ProtocolStatusTypeDef;
typedef struct { uint32_t TxErrorCnt, RxErrorCnt; } FDCAN_ErrorCountersTypeDef;
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *bus, FDCAN_ProtocolStatusTypeDef *status);
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef *bus, FDCAN_ErrorCountersTypeDef *errors);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *bus);
#endif
