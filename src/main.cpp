// Minimal PRIZM PacketSerial firmware
// Responsibilities:
//  - Receive velocity commands (PKT_CMD_VELOCITY) via PacketSerial (COBS)
//  - Apply a simple fixed-slope ramp to motor outputs
//  - Publish wheel joint pose (degrees mod 0..359) at 20Hz
//  - Publish status (battery cV, uptime_ms) at 1Hz
//  - Keep PacketSerial usage and edubot_protocol unchanged

#include <Arduino.h>
#include <PRIZM.h>
#include <PacketSerial.h>
#include "edubot_protocol.hpp"

// Serial
static const uint32_t SERIAL_BAUD = 115200;

// Control
static const uint32_t CONTROL_LOOP_PERIOD_MS = 10; // 100Hz control tick
static const uint16_t COMMAND_WATCHDOG_MS = 100;   // stop motors if command stale

// Ramp
static const float RAMP_TIME_S = 1.0f;
static const int16_t MAX_MOTOR_SPEED_DPS = 720;

// Kinematics
static const float WHEEL_RADIUS_M = 0.0508f;
static const float TRACK_WIDTH_M = 0.28636f;

// Signs
static const int8_t MOTOR_LEFT_SIGN = -1;
static const int8_t MOTOR_RIGHT_SIGN = +1;
static const int8_t ENCODER_LEFT_SIGN = -1;
static const int8_t ENCODER_RIGHT_SIGN = +1;

// State
static PacketSerial packet_serial;
static PRIZM prizm;

static volatile int16_t commanded_linear_x_mmps = 0;
static volatile int16_t commanded_angular_z_mrps = 0;
static volatile bool commanded_enable = false;
static volatile uint32_t last_command_time_ms = 0;

static float output_left_speed_dps = 0.0f;
static float output_right_speed_dps = 0.0f;

static uint32_t boot_time_ms = 0;

// Helpers
static inline float rps_to_dps(float rps) { return rps * (180.0f / 3.14159265358979323846f); }
static inline void body_to_wheel_rps(float linear_x_mps, float angular_z_rps, float &l, float &r)
{
    const float half_track = 0.5f * TRACK_WIDTH_M;
    l = (linear_x_mps - angular_z_rps * half_track) / max(WHEEL_RADIUS_M, 1e-6f);
    r = (linear_x_mps + angular_z_rps * half_track) / max(WHEEL_RADIUS_M, 1e-6f);
}
static inline float ramp_fixed_slope(float current, float target)
{
    const float slope = (2.0f * (float)MAX_MOTOR_SPEED_DPS) / max(RAMP_TIME_S, 1e-6f);
    const float step = slope * (CONTROL_LOOP_PERIOD_MS / 1000.0f);
    const float err = target - current;
    if (err > step)
        return current + step;
    if (err < -step)
        return current - step;
    return target;
}
static inline uint16_t wrap_degrees_0_359(long deg)
{
    long m = deg % 360;
    if (m < 0)
        m += 360;
    return (uint16_t)m;
}

// Protocol handler
static void on_packet_received(const uint8_t *buf, size_t len)
{
    int16_t lin = 0, ang = 0;
    bool en = false;
    if (parse_cmd_velocity(buf, len, lin, ang, en))
    {
        noInterrupts();
        commanded_linear_x_mmps = lin;
        commanded_angular_z_mrps = ang;
        commanded_enable = en;
        last_command_time_ms = millis();
        interrupts();
    }
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    packet_serial.setStream(&Serial);
    packet_serial.setPacketHandler(&on_packet_received);

    prizm.PrizmBegin();
    prizm.resetEncoders();

    boot_time_ms = millis();
    last_command_time_ms = boot_time_ms;
}

void loop()
{
    // Service PacketSerial frequently
    packet_serial.update();

    static uint32_t last_loop = 0;
    uint32_t now = millis();
    if (now - last_loop < CONTROL_LOOP_PERIOD_MS)
        return;
    last_loop = now;

    // Watchdog
    const bool command_fresh = (now - last_command_time_ms) <= COMMAND_WATCHDOG_MS;
    const bool command_nonzero = (abs(commanded_linear_x_mmps) > 10 || abs(commanded_angular_z_mrps) > 10);
    const bool motion_enabled = command_fresh && (commanded_enable || command_nonzero);

    float left_rps = 0.0f, right_rps = 0.0f;
    if (motion_enabled)
    {
        const float v = ((float)commanded_linear_x_mmps) / 1000.0f;
        const float wz = ((float)commanded_angular_z_mrps) / 1000.0f;
        body_to_wheel_rps(v, wz, left_rps, right_rps);
    }

    const float left_target = constrain((float)MOTOR_LEFT_SIGN * rps_to_dps(left_rps), -MAX_MOTOR_SPEED_DPS, MAX_MOTOR_SPEED_DPS);
    const float right_target = constrain((float)MOTOR_RIGHT_SIGN * rps_to_dps(right_rps), -MAX_MOTOR_SPEED_DPS, MAX_MOTOR_SPEED_DPS);

    const float desired_left = motion_enabled ? left_target : 0.0f;
    const float desired_right = motion_enabled ? right_target : 0.0f;

    output_left_speed_dps = ramp_fixed_slope(output_left_speed_dps, desired_left);
    output_right_speed_dps = ramp_fixed_slope(output_right_speed_dps, desired_right);

    prizm.setMotorSpeeds((int)output_left_speed_dps, (int)output_right_speed_dps);

    // Wheels telemetry @20Hz (every 50ms)
    static uint32_t last_wheels = 0;
    if (now - last_wheels >= 50)
    {
        const long raw_l = prizm.readEncoderDegrees(1);
        const long raw_r = prizm.readEncoderDegrees(2);
        const long nl = (long)ENCODER_LEFT_SIGN * raw_l;
        const long nr = (long)ENCODER_RIGHT_SIGN * raw_r;
        uint8_t payload[7];
        const size_t n = build_tel_wheels_deg_mod(payload, wrap_degrees_0_359(nl), wrap_degrees_0_359(nr));
        packet_serial.send(payload, n);
        last_wheels = now;
    }

    // Status telemetry @1Hz
    static uint32_t last_status = 0;
    if (now - last_status >= 1000)
    {
        const float battery_V = prizm.readBatteryVoltage() / 100.0f;
        const uint16_t battery_cV = (uint16_t)lroundf(battery_V * 100.0f);
        uint8_t payload[9];
        const size_t n = build_tel_status(payload, battery_cV, now - boot_time_ms);
        packet_serial.send(payload, n);
        last_status = now;
    }
}
