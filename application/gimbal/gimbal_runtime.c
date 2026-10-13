/** @file gimbal_runtime.c
 * @brief USB CDC IMU text/VOFA+ telemetry and bounded ASCII tuning commands.
 * USB ISR only copies bytes. Parsing/transmission run in the low-priority comm task;
 * the control task alone changes control configuration.
 */
#include "gimbal_runtime.h"
#include "2yaw_gimbal.h"
#include "gimbal_manual_config.h"
#include "gimbal_imu_text.h"
#include "dt7_cmd.h"
#include "remote_control.h"
#include "bsp_usb.h"
#include "main.h"
#include "fdcan.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

#define RX_CAPACITY 512U
#define COMMAND_QUEUE_LENGTH 8U
#define COMMAND_IDLE_TIMEOUT_MS 250U

typedef struct {
    Vofa_Command_s command;
    uint32_t sequence;
} Queued_Command_s;

typedef struct {
    uint32_t sequence;
    Vofa_Result_e result;
    float value;
    uint8_t streaming;
    uint16_t rate_hz;
    uint8_t imu_text;
} Vofa_Reply_s;

static uint8_t rx_bytes[RX_CAPACITY];
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t rx_overflow;
static volatile uint8_t rx_gap;
static volatile uint32_t rx_dropped;
static volatile uint32_t rx_last_ms;
static uint32_t queue_dropped, tx_skipped;
static QueueHandle_t command_queue;
static Vofa_Reply_s reply = {
    .streaming = GIMBAL_VOFA_STREAM_ENABLED,
    .rate_hz = GIMBAL_VOFA_DEFAULT_RATE_HZ,
    .imu_text = GIMBAL_USB_IMU_TEXT_DEFAULT,
};

uint8_t GimbalRuntimeTextIsStreaming(void)
{
    taskENTER_CRITICAL();
    uint8_t enabled = reply.imu_text && reply.streaming;
    taskEXIT_CRITICAL();
    return enabled;
}

static void UsbReceive(uint16_t length)
{
    const uint8_t *bytes = USBGetRxBuffer();
    if (!bytes || !length) return;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - rx_last_ms) > COMMAND_IDLE_TIMEOUT_MS) rx_gap = 1U;
    rx_last_ms = now;
    for (uint16_t i = 0U; i < length; ++i)
    {
        uint16_t next = (rx_head + 1U) % RX_CAPACITY;
        if (next == rx_tail)
        {
            rx_overflow = 1U;
            ++rx_dropped;
            return; /* main task discards the corrupted line, never executes its suffix */
        }
        rx_bytes[rx_head] = bytes[i];
        __DMB();
        rx_head = next;
    }
}

void GimbalRuntimeInit(void)
{
    HAL_GPIO_WritePin(POWER_24V_1_GPIO_Port, POWER_24V_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(POWER_24V_2_GPIO_Port, POWER_24V_2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(POWER_5V_GPIO_Port, POWER_5V_Pin, GPIO_PIN_SET);
    GimbalInit();
    RobotCMDInit();
    command_queue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(Queued_Command_s));
    configASSERT(command_queue != NULL);
    USB_Init_Config_s usb = {.tx_cbk = NULL, .rx_cbk = UsbReceive};
    (void)USBInit(usb);
}

void GimbalRuntimeControl(void)
{
    RobotCMDTask();
    GimbalTask();
    Queued_Command_s queued;
    /* Apply at most one command per control tick; no parsing or float formatting here. */
    if (xQueueReceive(command_queue, &queued, 0U) != pdPASS) return;
    const Vofa_Command_s *cmd = &queued.command;
    Vofa_Result_e result = VOFA_RESULT_SYNTAX;
    float value = 0.0f;
    uint8_t streaming = reply.streaming;
    uint16_t rate = reply.rate_hz;
    uint8_t imu_text = reply.imu_text;
    switch (cmd->type)
    {
    case VOFA_CMD_IMU_TEXT:
        if (cmd->value == 0.0f || cmd->value == 1.0f)
        {
            imu_text = (uint8_t)cmd->value;
            streaming = 1U;
            result = VOFA_RESULT_OK;
            value = cmd->value;
        }
        else result = VOFA_RESULT_RANGE;
        break;
    case VOFA_CMD_STREAM:
        if (cmd->value == 0.0f || cmd->value == 1.0f)
        {
            streaming = (uint8_t)cmd->value;
            result = VOFA_RESULT_OK;
            value = cmd->value;
        }
        else result = VOFA_RESULT_RANGE;
        break;
    case VOFA_CMD_RATE:
        if (cmd->value == 10.0f || cmd->value == 20.0f || cmd->value == 25.0f ||
            cmd->value == 50.0f || cmd->value == 100.0f)
        {
            rate = (uint16_t)cmd->value;
            result = VOFA_RESULT_OK;
            value = cmd->value;
        }
        else result = VOFA_RESULT_RANGE;
        break;
    default:
        result = GimbalApplyVofaCommand(cmd, &value);
        break;
    }
    taskENTER_CRITICAL();
    reply = (Vofa_Reply_s){.sequence = queued.sequence, .result = result, .value = value,
        .streaming = streaming, .rate_hz = rate, .imu_text = imu_text};
    taskEXIT_CRITICAL();
}

static void QueueLine(const char *text, uint8_t rejected)
{
    static uint32_t sequence;
    Queued_Command_s queued = {0};
    sequence = (sequence + 1U) & 0x00ffffffU; /* exact in float32 telemetry */
    queued.sequence = sequence;
    if (!rejected && !VofaParseCommand(text, &queued.command)) queued.command.type = VOFA_CMD_INVALID;
    /* STOP/STAGE 0 cancel older pending tuning commands, including a full queue. */
    if (queued.command.type == VOFA_CMD_STAGE && queued.command.value == 0.0f)
    {
        xQueueReset(command_queue);
        if (xQueueSendToFront(command_queue, &queued, 0U) != pdPASS) ++queue_dropped;
    }
    else if (xQueueSend(command_queue, &queued, 0U) != pdPASS) ++queue_dropped;
}

static void ReceiveCommands(void)
{
    static Vofa_Line_s line;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t overflow = rx_overflow;
    uint8_t gap = rx_gap;
    rx_gap = 0U;
    if (overflow)
    {
        rx_tail = rx_head;
        rx_overflow = 0U;
    }
    uint8_t empty = rx_tail == rx_head;
    uint32_t last_rx = rx_last_ms;
    __set_PRIMASK(primask);
    if (overflow) VofaLineDiscard(&line);
    if (gap && line.length) VofaLineDiscard(&line);
    if (empty && line.length && (uint32_t)(HAL_GetTick() - last_rx) > COMMAND_IDLE_TIMEOUT_MS)
        VofaLineDiscard(&line);
    for (uint16_t budget = 0U; budget < 128U; ++budget)
    {
        primask = __get_PRIMASK();
        __disable_irq();
        if (rx_overflow)
        {
            rx_tail = rx_head;
            rx_overflow = 0U;
            __set_PRIMASK(primask);
            VofaLineDiscard(&line);
            break;
        }
        if (rx_tail == rx_head)
        {
            __set_PRIMASK(primask);
            break;
        }
        uint8_t byte = rx_bytes[rx_tail];
        rx_tail = (rx_tail + 1U) % RX_CAPACITY;
        __set_PRIMASK(primask);
        int status = VofaLineFeed(&line, byte);
        if (status) QueueLine(line.text, status < 0);
    }
}

void GimbalRuntimeComm(void)
{
    static uint32_t last_ms;
    static uint32_t sent_sequence;
    static uint8_t frame[1024U]; /* USB BSP owns a 1024-byte asynchronous staging buffer. */
    static uint8_t previous_text_mode = GIMBAL_USB_IMU_TEXT_DEFAULT, header_sent;
    static uint32_t last_header_ms;
    static uint32_t last_motor_diag_ms;
    static uint8_t dual_diag_axis;
    ReceiveCommands();
    if (!USBIsReady()) return;
    Vofa_Reply_s r;
    Gimbal_Debug_Data_s d;
    taskENTER_CRITICAL();
    r = reply;
    GimbalGetDebugData(&d); /* nested critical section keeps reply/config from the same control tick */
    taskEXIT_CRITICAL();
    uint32_t now = HAL_GetTick();
    uint8_t response_pending = sent_sequence != r.sequence;
    if (previous_text_mode != r.imu_text)
    {
        previous_text_mode = r.imu_text;
        header_sent = 0U;
    }
    if (r.imu_text)
    {
        size_t length;
        uint8_t send_header = 0U;
        uint8_t send_motor_diag = 0U;
        if (response_pending)
        {
            char value[20];
            GimbalImuTextFormatValue(r.value, value);
            int count = snprintf((char *)frame, sizeof(frame),
                "# ACK seq=%lu result=%u value=%s stage=%u stream=%u\r\n",
                (unsigned long)r.sequence, (unsigned)r.result, value,
                (unsigned)d.stage, (unsigned)r.streaming);
            length = count > 0 && (size_t)count < sizeof(frame) ? (size_t)count : 0U;
        }
        else
        {
            if (!r.streaming) return;
            send_header = !header_sent ||
                (uint32_t)(now - last_header_ms) >= GIMBAL_IMU_TEXT_HEADER_PERIOD_MS;
            if (send_header)
                length = GimbalImuTextHeader("USB", (char *)frame, sizeof(frame));
            else
            {
                if ((uint32_t)(now - last_ms) < GIMBAL_IMU_TEXT_PERIOD_MS) return;
                length = GimbalImuTextFormat(&d, tx_skipped, (char *)frame, sizeof(frame));
                if (length && (uint32_t)(now - last_motor_diag_ms) >= 250U)
                {
                    FDCAN_HandleTypeDef *bus[2] = {GIMBAL_YAW_CAN_HANDLE, GIMBAL_PITCH_CAN_HANDLE};
                    /* Read diagnostic registers only in the communication task, at 4 Hz. */
                    for (uint8_t i = 0U; i < 2U; ++i)
                    {
                        FDCAN_ProtocolStatusTypeDef protocol = {0};
                        FDCAN_ErrorCountersTypeDef errors = {0};
                        HAL_StatusTypeDef ps = HAL_FDCAN_GetProtocolStatus(bus[i], &protocol);
                        HAL_StatusTypeDef es = HAL_FDCAN_GetErrorCounters(bus[i], &errors);
                        int count = snprintf((char *)frame + length, sizeof(frame) - length,
                            "# MOTOR axis=%s t_ms=%lu i_raw=%d rpm=%d q_ok=%lu q_fail=%lu can_ok=%u bo=%lu tec=%lu rec=%lu tx_free=%lu temp_c=%u\r\n",
                            i == 0U ? "yaw" : "pitch", (unsigned long)d.tick_ms,
                            (int)d.motor_current_raw[i], (int)d.motor_rpm_raw[i],
                            (unsigned long)d.tx_enqueue_ok[i], (unsigned long)d.tx_enqueue_fail[i],
                            (unsigned)(ps == HAL_OK && es == HAL_OK),
                            (unsigned long)protocol.BusOff, (unsigned long)errors.TxErrorCnt,
                            (unsigned long)errors.RxErrorCnt,
                            (unsigned long)HAL_FDCAN_GetTxFifoFreeLevel(bus[i]), (unsigned)d.temperature[i]);
                        if (count <= 0 || (size_t)count >= sizeof(frame) - length) break;
                        length += (size_t)count;
                        if (i == 1U) send_motor_diag = 1U;
                    }
                    if (send_motor_diag)
                    {
                        /* Alternate dual-axis diagnostics within the USB staging limit. */
                        uint8_t axis = d.command.request.axis_mask == 3U ? dual_diag_axis :
                            (d.command.request.axis_mask == 2U ? 1U : 0U);
                        uint8_t recovering = d.stage != 0U && d.config[axis].soft_recovery_speed > 0.0f &&
                            (d.feedback[axis].joint < d.feedback[axis].soft_min ||
                             d.feedback[axis].joint > d.feedback[axis].soft_max);
                        char target_rate[20], actual_rate[20], integral[20], speed_max[20], accel[20], angle_error[20], filtered_rate[20], manual_rate[20];
                        GimbalImuTextFormatValue(d.control.axis[axis].rate_target / GIMBAL_RAD_PER_DEG, target_rate);
                        GimbalImuTextFormatValue(d.feedback[axis].rate / GIMBAL_RAD_PER_DEG, actual_rate);
                        GimbalImuTextFormatValue(d.control.axis[axis].filtered_rate / GIMBAL_RAD_PER_DEG, filtered_rate);
                        GimbalImuTextFormatValue(d.control.axis[axis].manual_rate / GIMBAL_RAD_PER_DEG, manual_rate);
                        GimbalImuTextFormatValue(d.control.axis[axis].integral, integral);
                        GimbalImuTextFormatValue(d.config[axis].speed_limit / GIMBAL_RAD_PER_DEG, speed_max);
                        GimbalImuTextFormatValue(d.config[axis].acceleration_limit / GIMBAL_RAD_PER_DEG, accel);
                        GimbalImuTextFormatValue((d.control.axis[axis].target - d.feedback[axis].angle) / GIMBAL_RAD_PER_DEG, angle_error);
                        int count = snprintf((char *)frame + length, sizeof(frame) - length,
                            "# CTRL axis=%s t_ms=%lu target_dps=%s actual_dps=%s i_term=%s speed_max_dps=%s accel_dps2=%s out_max=%u recover=%u angle_err_deg=%s filtered_dps=%s manual_dps=%s\r\n",
                            axis == 0U ? "yaw" : "pitch", (unsigned long)d.tick_ms,
                            target_rate, actual_rate, integral, speed_max, accel,
                            (unsigned)d.config[axis].output_limit, (unsigned)recovering, angle_error, filtered_rate, manual_rate);
                        if (count > 0 && (size_t)count < sizeof(frame) - length) length += (size_t)count;
                        Remote_Control_Diagnostics_s rc;
                        RemoteControlGetDiagnostics(&rc);
                        count = snprintf((char *)frame + length, sizeof(frame) - length,
                            "# RC age_ms=%lu ok=%lu bad=%lu len=%u why=%u uart_err=%lu err=0x%lx retry=%lu start_fail=%lu\r\n",
                            (unsigned long)rc.age_ms, (unsigned long)rc.good_frames,
                            (unsigned long)rc.bad_frames, (unsigned)rc.last_length, (unsigned)rc.bad_reason,
                            (unsigned long)rc.uart_errors, (unsigned long)rc.last_uart_error,
                            (unsigned long)rc.receive_retries, (unsigned long)rc.start_failures);
                        if (count > 0 && (size_t)count < sizeof(frame) - length) length += (size_t)count;
                    }
                }
            }
        }
        /* USBTransmit copies into its own protected staging buffer; do not wait for USB. */
        if (length && USBTransmit(frame, (uint16_t)length))
        {
            last_ms = now;
            if (response_pending) sent_sequence = r.sequence;
            if (send_motor_diag)
            {
                last_motor_diag_ms = now;
                if (d.command.request.axis_mask == 3U) dual_diag_axis ^= 1U;
            }
            if (send_header)
            {
                header_sent = 1U;
                last_header_ms = now;
            }
        }
        else ++tx_skipped;
        return;
    }
    uint32_t period_ms = 1000U / r.rate_hz;
    if (!response_pending && (!r.streaming || (uint32_t)(now - last_ms) < period_ms)) return;
    const float degrees = 1.0f / GIMBAL_RAD_PER_DEG;
    float channel[VOFA_CHANNEL_COUNT] = {
        d.control.axis[0].target * degrees, d.feedback[0].angle * degrees,
        d.control.axis[0].rate_target * degrees, d.feedback[0].rate * degrees, (float)d.output[0],
        d.control.axis[1].target * degrees, d.feedback[1].angle * degrees,
        d.control.axis[1].rate_target * degrees, d.feedback[1].rate * degrees, (float)d.output[1],
        (d.control.axis[0].target - d.feedback[0].angle) * degrees,
        (d.control.axis[1].target - d.feedback[1].angle) * degrees,
        (float)d.control.state, (float)d.control.faults, (float)d.stage,
        (float)d.command.request.axis_mask, (float)d.command.online, (float)d.imu_calibrated,
        (float)d.imu_age_ms, (float)d.motor_age_ms[0], (float)d.motor_age_ms[1], d.dt * 1000.0f,
        (float)r.sequence, (float)r.result, r.value, (float)(rx_dropped + queue_dropped), (float)tx_skipped,
        d.config[0].angle_kp, d.config[0].speed_kp, d.config[0].speed_ki,
        d.config[1].angle_kp, d.config[1].speed_kp, d.config[1].speed_ki,
        d.config[0].output_limit, d.config[1].output_limit, (float)d.ecd[0], (float)d.ecd[1],
        (float)d.tick_ms * 0.001f, (float)d.temperature[0], (float)d.temperature[1],
        d.raw_roll, d.gyro[0] * degrees, d.gyro[1] * degrees, d.gyro[2] * degrees,
        d.raw_yaw, d.raw_pitch,
        (float)d.command.request.mode, (float)d.command.request.sticks_centered,
    };
    size_t length = VofaPackFrame(channel, frame, sizeof(frame));
    if (length && USBTransmit(frame, (uint16_t)length))
    {
        last_ms = now;
        sent_sequence = r.sequence;
    }
    else ++tx_skipped; /* skip old telemetry instead of blocking control or accumulating delay */
}
