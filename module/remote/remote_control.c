#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
// #include "daemon.h"
#include "bsp_log.h"

#define REMOTE_CONTROL_FRAME_SIZE 18u // 遥控器接收的buffer大小

// 遥控器数据
static RC_ctrl_t rc_ctrl[2];     //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位
static volatile uint8_t rc_rx_valid = 0;
static volatile uint32_t rc_last_rx_tick = 0;
static Remote_Control_Diagnostics_s rc_diagnostics;

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *rc_usart_instance;
// static DaemonInstance *rc_daemon_instance;



/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static void sbus_to_rc(const uint8_t *sbus_buf)
{
    // 摇杆,直接解算时减去偏置
    rc_ctrl[TEMP].rc.rocker_r_ = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;                              //!< Channel 0
    rc_ctrl[TEMP].rc.rocker_r1 = (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 1
    rc_ctrl[TEMP].rc.rocker_l_ = (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET; //!< Channel 2
    rc_ctrl[TEMP].rc.rocker_l1 = (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 3
    rc_ctrl[TEMP].rc.dial = ((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;                                 // 左侧拨轮

    // 开关,0左1右
    rc_ctrl[TEMP].rc.switch_right = ((sbus_buf[5] >> 4) & 0x0003);     //!< Switch right
    rc_ctrl[TEMP].rc.switch_left = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch left

    // 鼠标解析
    rc_ctrl[TEMP].mouse.x = (sbus_buf[6] | (sbus_buf[7] << 8)); //!< Mouse X axis
    rc_ctrl[TEMP].mouse.y = (sbus_buf[8] | (sbus_buf[9] << 8)); //!< Mouse Y axis
    rc_ctrl[TEMP].mouse.press_l = sbus_buf[12];                 //!< Mouse Left Is Press ?
    rc_ctrl[TEMP].mouse.press_r = sbus_buf[13];                 //!< Mouse Right Is Press ?

    //  位域的按键值解算,直接memcpy即可,注意小端低字节在前,即lsb在第一位,msb在最后
    *(uint16_t *)&rc_ctrl[TEMP].key[KEY_PRESS] = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));
    if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (rc_ctrl[TEMP].key[KEY_PRESS].shift) // shift键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    uint16_t key_now = rc_ctrl[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
        key_last = rc_ctrl[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
        key_with_ctrl = rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
        key_with_shift = rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
        key_last_with_ctrl = rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
        key_last_with_shift = rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
            continue;
        // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
        // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }

    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
}

/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void RemoteControlRxCallback()
{
    const uint8_t *b = rc_usart_instance->recv_buff;
    rc_diagnostics.last_length = rc_usart_instance->recv_len;
    /* Discard fragments without decoding or refreshing the last-good deadline.
     * A UART error can leave a short IDLE tail before the next complete frame.
     * Keep the last validated command only within the existing 100 ms watchdog. */
    if (rc_usart_instance->recv_len != REMOTE_CONTROL_FRAME_SIZE)
    {
        ++rc_diagnostics.bad_frames;
        rc_diagnostics.bad_reason = 1U;
        return;
    }
    uint16_t channel[5] = {
        (b[0] | (b[1] << 8)) & 0x7ff,
        ((b[1] >> 3) | (b[2] << 5)) & 0x7ff,
        ((b[2] >> 6) | (b[3] << 2) | (b[4] << 10)) & 0x7ff,
        ((b[4] >> 1) | (b[5] << 7)) & 0x7ff,
        (b[16] | (b[17] << 8)) & 0x7ff,
    };
    /* A short IDLE frame or malformed switch must never refresh the watchdog. */
    if (((b[5] >> 4) & 3U) == 0U || ((b[5] >> 6) & 3U) == 0U)
    {
        ++rc_diagnostics.bad_frames;
        rc_diagnostics.bad_reason = 2U;
        rc_rx_valid = 0U;
        return;
    }
    if (b[12] > 1U || b[13] > 1U)
    {
        ++rc_diagnostics.bad_frames;
        rc_diagnostics.bad_reason = 3U;
        rc_rx_valid = 0U;
        return;
    }
    for (uint8_t i = 0; i < 5U; ++i)
        if (channel[i] < RC_CH_VALUE_MIN || channel[i] > RC_CH_VALUE_MAX)
        {
            ++rc_diagnostics.bad_frames;
            rc_diagnostics.bad_reason = 4U;
            rc_rx_valid = 0U;
            return;
        }
    sbus_to_rc(rc_usart_instance->recv_buff); // 进行协议解析
    rc_last_rx_tick = HAL_GetTick();
    ++rc_diagnostics.good_frames;
    rc_rx_valid = 1;
}



RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    // Daemon_Init_Config_s daemon_conf = {
    //     .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
    //     .callback = RCLostCallback,
    //     .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    // };
    // rc_daemon_instance = DaemonRegister(&daemon_conf);

    rc_init_flag = 1;
    return rc_ctrl;
}

uint8_t RemoteControlIsOnline()
{
    /*
     * DT7/DR16 normally refreshes at about 70 Hz. 100 ms without a complete
     * frame is treated as offline. Unsigned subtraction is wrap-safe.
     */
    if (!rc_init_flag || !rc_rx_valid)
        return 0;

    return ((uint32_t)(HAL_GetTick() - rc_last_rx_tick) <= 100U) ? 1U : 0U;
}

uint8_t RemoteControlGetSnapshot(RC_ctrl_t *sample)
{
    return RemoteControlGetSnapshotWithAge(sample, NULL);
}

uint8_t RemoteControlGetSnapshotWithAge(RC_ctrl_t *sample, uint32_t *age_ms)
{
    if (sample == NULL) return 0U;
    if (rc_init_flag) USARTEnsureReceive(rc_usart_instance);
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t online = RemoteControlIsOnline();
    memcpy(sample, &rc_ctrl[TEMP], sizeof(*sample));
    if (age_ms) *age_ms = (uint32_t)(HAL_GetTick() - rc_last_rx_tick);
    __set_PRIMASK(primask);
    return online;
}

void RemoteControlGetDiagnostics(Remote_Control_Diagnostics_s *diagnostics)
{
    if (!diagnostics) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *diagnostics = rc_diagnostics;
    diagnostics->age_ms = (uint32_t)(HAL_GetTick() - rc_last_rx_tick);
    diagnostics->online = RemoteControlIsOnline();
    if (rc_usart_instance)
    {
        diagnostics->uart_errors = rc_usart_instance->error_count;
        diagnostics->last_uart_error = rc_usart_instance->last_error;
        diagnostics->receive_retries = rc_usart_instance->receive_retries;
        diagnostics->start_failures = rc_usart_instance->start_failures;
    }
    __set_PRIMASK(primask);
}
