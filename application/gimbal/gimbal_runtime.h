/** @file gimbal_runtime.h @brief Shared manual-gimbal robot runtime. */
#ifndef GIMBAL_RUNTIME_H
#define GIMBAL_RUNTIME_H
#include <stdint.h>
void GimbalRuntimeInit(void);
void GimbalRuntimeControl(void);
void GimbalRuntimeComm(void);
/** @brief Task-context query: optional USB text reports may be sent in this mode only. */
uint8_t GimbalRuntimeTextIsStreaming(void);
#endif
