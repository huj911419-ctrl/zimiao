/** @file auto_aim.h @brief Tracking-only CBoard application; no shooter dependencies. */
#ifndef AUTO_AIM_H
#define AUTO_AIM_H
#include "gimbal_control.h"
#include <stdint.h>
typedef struct {
    uint32_t good_frames, bad_frames, ignored_shoot, last_rx_ms;
    uint8_t mode, target_active;
} AutoAim_Diagnostics_s;
void AutoAimInit(void);
/** Communication-task only. Timestamp is the USB receive time, not parse time. */
void AutoAimReceiveByte(uint8_t byte, uint32_t received_ms);
/** Communication-task only: discard a broken stream and revoke cached target. */
void AutoAimInvalidate(void);
/** Control-task only. Mutates AUTO into HOLD if takeover is unavailable.
 * OFF, existing faults, stage, calibration and fresh USB all dominate AUTO.
 */
void AutoAimSelect(Gimbal_Request_s *request, const Gimbal_Control_s *control,
                   uint32_t faults, uint8_t stage, uint32_t now);
/** Control-task only, after the controller has applied all guards. */
void AutoAimControlResult(const Gimbal_Control_s *control);
/** Comm-task only. Binary frames exclusively, nonblocking and freshest sample only. */
void AutoAimComm(void);
void AutoAimGetDiagnostics(AutoAim_Diagnostics_s *diagnostics);
#endif
