#include "vofa_protocol.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void TestLines(void)
{
    Vofa_Line_s line = {0};
    const char *text = "set yaw speed_kp 1500\r\nget pitch angle_kp\n";
    int completed = 0;
    for (size_t i = 0; text[i]; ++i)
    {
        int result = VofaLineFeed(&line, (uint8_t)text[i]);
        if (result == 1)
        {
            Vofa_Command_s cmd;
            assert(VofaParseCommand(line.text, &cmd));
            assert(cmd.type == (completed ? VOFA_CMD_GET : VOFA_CMD_SET));
            ++completed;
        }
    }
    assert(completed == 2);
    for (size_t i = 0; i < VOFA_LINE_SIZE + 20; ++i) assert(VofaLineFeed(&line, 'x') == 0);
    assert(VofaLineFeed(&line, '\n') == -1);
    assert(VofaLineFeed(&line, 's') == 0 && VofaLineFeed(&line, '\n') == 1);
    assert(strcmp(line.text, "s") == 0);
    assert(VofaLineFeed(&line, 0) == 0 && VofaLineFeed(&line, '\n') == -1);
    assert(VofaLineFeed(&line, '\n') == 0);
}

static void TestCommands(void)
{
    Vofa_Command_s cmd;
    assert(VofaParseCommand(" SET YAW SPEED_KP 1.5e3 ", &cmd));
    assert(cmd.type == VOFA_CMD_SET && cmd.axis == 0 && cmd.parameter == VOFA_PARAM_SPEED_KP && cmd.value == 1500);
    assert(VofaParseCommand("stage 2", &cmd) && cmd.type == VOFA_CMD_STAGE && cmd.value == 2);
    assert(VofaParseCommand("stop", &cmd) && cmd.type == VOFA_CMD_STAGE && cmd.value == 0);
    assert(VofaParseCommand("rate 100", &cmd) && cmd.type == VOFA_CMD_RATE);
    assert(VofaParseCommand("stream 0", &cmd) && cmd.type == VOFA_CMD_STREAM);
    assert(VofaParseCommand("imu 0", &cmd) && cmd.type == VOFA_CMD_IMU_TEXT && cmd.value == 0);
    assert(VofaParseCommand("imu 1", &cmd) && cmd.type == VOFA_CMD_IMU_TEXT && cmd.value == 1);
    assert(VofaParseCommand("T", &cmd) && cmd.value == 1);
    assert(VofaParseCommand("defaults", &cmd) && cmd.type == VOFA_CMD_DEFAULTS);
    const char *bad[] = {"", "stage", "stage 2 extra", "set pitch speed_kp", "set roll speed_kp 1",
        "set yaw speed_kd 1", "set yaw speed_kp nan", "set yaw speed_kp inf", "set yaw speed_kp 1e99",
        "set yaw speed_kp 1200junk", "get yaw speed_kp extra", "stage1", "foobar",
        "imu", "imu 1 extra", "imu nan"};
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) assert(!VofaParseCommand(bad[i], &cmd));
    Gimbal_Axis_Config_s config = {0};
    const char *names[] = {"angle_kp", "speed_kp", "speed_ki", "integral_limit", "output_limit", "speed_dps", "accel_dps2", "follow_error_deg"};
    const float values[] = {4, 1500, 50, 500, 2000, 10, 60, 5};
    for (size_t i = 0; i < 8; ++i)
    {
        char text[80];
        snprintf(text, sizeof(text), "set pitch %s %.1f", names[i], values[i]);
        assert(VofaParseCommand(text, &cmd));
        assert(VofaValidateParameter(&cmd, 5000, 15, 360) == VOFA_RESULT_OK);
        assert(fabsf(VofaSetParameter(&config, &cmd) - values[i]) < 0.001f);
        assert(fabsf(VofaGetParameter(&config, cmd.parameter) - values[i]) < 0.001f);
    }
    assert(fabsf(config.speed_limit - 10 * GIMBAL_RAD_PER_DEG) < 1e-6f);
    assert(VofaParseCommand("set yaw output_limit 5001", &cmd));
    assert(VofaValidateParameter(&cmd, 5000, 30, 720) == VOFA_RESULT_RANGE);
    assert(VofaParseCommand("set pitch speed_dps 16", &cmd));
    assert(VofaValidateParameter(&cmd, 5000, 15, 360) == VOFA_RESULT_RANGE);
    assert(VofaParseCommand("set yaw speed_kp -1", &cmd));
    assert(VofaValidateParameter(&cmd, 5000, 30, 720) == VOFA_RESULT_RANGE);
    assert(VofaParseCommand("set yaw accel_dps2 720", &cmd));
    assert(VofaValidateParameter(&cmd, 15000, 180, 720) == VOFA_RESULT_OK);
    assert(VofaParseCommand("set yaw accel_dps2 721", &cmd));
    assert(VofaValidateParameter(&cmd, 15000, 180, 720) == VOFA_RESULT_RANGE);
    assert(VofaParseCommand("set pitch accel_dps2 361", &cmd));
    assert(VofaValidateParameter(&cmd, 5000, 15, 360) == VOFA_RESULT_RANGE);
}

static void TestFraming(void)
{
    float channels[VOFA_CHANNEL_COUNT] = {1.0f, -2.0f, NAN, INFINITY};
    uint8_t frame[VOFA_FRAME_SIZE + 2];
    memset(frame, 0xaa, sizeof(frame));
    assert(!VofaPackFrame(channels, frame, VOFA_FRAME_SIZE - 1));
    assert(VofaPackFrame(channels, frame, sizeof(frame)) == VOFA_FRAME_SIZE);
    const uint8_t expected[] = {0, 0, 0x80, 0x3f, 0, 0, 0, 0xc0, 0, 0, 0, 0, 0, 0, 0, 0};
    assert(memcmp(frame, expected, sizeof(expected)) == 0);
    const uint8_t tail[] = {0, 0, 0x80, 0x7f};
    assert(memcmp(frame + VOFA_CHANNEL_COUNT * 4, tail, 4) == 0);
    assert(frame[VOFA_FRAME_SIZE] == 0xaa);
}

int main(void)
{
    TestLines(); TestCommands(); TestFraming();
    puts("PASS: VOFA command syntax, fragmented/batched lines, bounds, unit conversion, JustFloat byte order/tail and finite payload");
    return 0;
}
