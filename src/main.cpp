// src/main.cpp
// PRIZM motor controller with PacketSerial (COBS) framing and CRC16.
// Responsibilities:
//   • Receive velocity commands (mm/s, mrad/s) with a 100 ms watchdog.
//   • Apply a fixed-time linear ramp so the motor outputs reach any new target in proportion to the remaining percentage (e.g., 0→100% in 1.0 s).
//   • Publish per-wheel joint pose wrapped to 0..359 deg at 50 Hz and battery/uptime at 1 Hz for diagnostics.
//   • Normalize motor and encoder signs in software so positive linear velocity drives both wheels forward and increases both joint poses.
//
// Important: Do not print text to Serial during operation; the same UART is used for framed binary packets.

#include <Arduino.h>
#include <PRIZM.h>
#include <PacketSerial.h>
#include "edubot_protocol.hpp"

// ------------------------
// Configuration constants
// ------------------------

// Serial transport settings
static const uint32_t SERIAL_BAUD = 115200;

// Loop and safety
static const uint32_t CONTROL_LOOP_PERIOD_MS = 10; // 100 Hz
static const uint16_t COMMAND_WATCHDOG_MS = 100;   // Stop if no command in 100 ms

// Fixed-time ramp settings
// The slope is chosen so a full-scale change (from -MAX to +MAX) completes in RAMP_TIME_S seconds.
// Any smaller change completes proportionally (e.g., half-scale in RAMP_TIME_S/2), including deceleration to zero.
static const float RAMP_TIME_S = 1.0f; // 0→100% change in ~1.0 s
static const float LOOP_DT_S = (CONTROL_LOOP_PERIOD_MS / 1000.0f);

// Kinematics and motor limits
static const float WHEEL_RADIUS_M = 0.0508f; // 4" diameter
static const float TRACK_WIDTH_M = 0.28636f;
static const int16_t MAX_MOTOR_SPEED_DPS = 720; // Clamp motor command

// Per-wheel sign normalization
// Choose these so positive linear_x produces forward rotation on each wheel.
// With these defaults, both motors drive forward for positive linear_x and joint poses increase.
static const int8_t MOTOR_LEFT_SIGN = -1;
static const int8_t MOTOR_RIGHT_SIGN = +1;
static const int8_t ENCODER_LEFT_SIGN = -1;
static const int8_t ENCODER_RIGHT_SIGN = +1;

// ------------------------
// Controller state
// ------------------------

static PacketSerial packet_serial;
static PRIZM prizm;

// Latched command (from host)
static volatile int16_t commanded_linear_x_mmps = 0;
static volatile int16_t commanded_angular_z_mrps = 0;
static volatile bool commanded_enable = false;
static uint32_t last_command_time_ms = 0;

// Output speeds after ramping (deg/s)
static float output_left_speed_dps = 0.0f;
static float output_right_speed_dps = 0.0f;

// For joint pose (0..359 deg per wheel)
static long raw_left_degrees_last = 0;
static long raw_right_degrees_last = 0;

static uint32_t boot_time_ms = 0;

// ------------------------
// Helper functions
// ------------------------

// Convert wheel angular velocity from rad/s to deg/s
static inline float rps_to_dps(float rps)
{
    return rps * (180.0f / 3.14159265358979323846f);
}

// Compute per-wheel angular rates (rad/s) from body velocities
static inline void body_to_wheel_rps(float linear_x_mps, float angular_z_rps, float &left_wheel_rps, float &right_wheel_rps)
{
    const float half_track = 0.5f * TRACK_WIDTH_M;
    left_wheel_rps = (linear_x_mps - angular_z_rps * half_track) / max(WHEEL_RADIUS_M, 1e-6f);
    right_wheel_rps = (linear_x_mps + angular_z_rps * half_track) / max(WHEEL_RADIUS_M, 1e-6f);
}

// Constant-slope ramp: move current toward target at fixed slope so time-to-target is |Δ| / slope.
// Slope is derived from the configured ramp time and the command full-scale (MAX_MOTOR_SPEED_DPS).
static inline float ramp_fixed_slope(float current, float target)
{
    // Slope in deg/s^2: full-scale change (2*MAX) over RAMP_TIME_S ⇒ slope = (2*MAX)/RAMP_TIME_S.
    const float slope_dps_per_s = (2.0f * (float)MAX_MOTOR_SPEED_DPS) / max(RAMP_TIME_S, 1e-6f);
    const float step = slope_dps_per_s * LOOP_DT_S;
    const float error = target - current;
    if (error > step)
        return current + step;
    if (error < -step)
        return current - step;
    return target;
}

// Wrap signed degrees into 0..359 for joint pose
static inline uint16_t wrap_degrees_0_359(long signed_degrees)
{
    long mod = signed_degrees % 360;
    if (mod < 0)
        mod += 360;
    return (uint16_t)mod;
}

// PacketSerial callback: parse velocity command
static void on_packet_received(const uint8_t *buffer, size_t size)
{
    int16_t lin_mmps = 0, ang_mrps = 0;
    bool en = false;
    if (parse_cmd_velocity(buffer, size, lin_mmps, ang_mrps, en))
    {
        noInterrupts();
        commanded_linear_x_mmps = lin_mmps;
        commanded_angular_z_mrps = ang_mrps;
        commanded_enable = en;
        interrupts();
        last_command_time_ms = millis();
    }
}

// ------------------------
// Arduino entry points
// ------------------------

void setup()
{
    Serial.begin(SERIAL_BAUD);
    packet_serial.setStream(&Serial);
    packet_serial.setPacketHandler(&on_packet_received);

    prizm.PrizmBegin();
    prizm.resetEncoders(); // Zero once at boot

    raw_left_degrees_last = prizm.readEncoderDegrees(1);
    raw_right_degrees_last = prizm.readEncoderDegrees(2);

    boot_time_ms = millis();
    last_command_time_ms = boot_time_ms;
}

void loop()
{
    // Service framed I/O
    packet_serial.update();

    // Fixed-rate control
    static uint32_t last_loop_ms = 0;
    const uint32_t now_ms = millis();
    if (now_ms - last_loop_ms < CONTROL_LOOP_PERIOD_MS)
        return;
    last_loop_ms = now_ms;

    // Watchdog and enable gating
    const bool command_fresh = (now_ms - last_command_time_ms) <= COMMAND_WATCHDOG_MS;
    const bool command_nonzero = (abs(commanded_linear_x_mmps) > 10 || abs(commanded_angular_z_mrps) > 10);
    const bool motion_enabled = command_fresh && (commanded_enable || command_nonzero);

    // Compute wheel targets (deg/s), normalize motor signs, clamp
    float left_wheel_rps = 0.0f, right_wheel_rps = 0.0f;
    if (motion_enabled)
    {
        const float v_mps = ((float)commanded_linear_x_mmps) / 1000.0f;
        const float wz_rps = ((float)commanded_angular_z_mrps) / 1000.0f;
        body_to_wheel_rps(v_mps, wz_rps, left_wheel_rps, right_wheel_rps);
    }
    const float left_target_dps = constrain((float)MOTOR_LEFT_SIGN * rps_to_dps(left_wheel_rps), -MAX_MOTOR_SPEED_DPS, MAX_MOTOR_SPEED_DPS);
    const float right_target_dps = constrain((float)MOTOR_RIGHT_SIGN * rps_to_dps(right_wheel_rps), -MAX_MOTOR_SPEED_DPS, MAX_MOTOR_SPEED_DPS);

    // If watchdog trips, ramp toward zero using the same fixed slope for a predictable stop
    const float desired_left_dps = motion_enabled ? left_target_dps : 0.0f;
    const float desired_right_dps = motion_enabled ? right_target_dps : 0.0f;

    output_left_speed_dps = ramp_fixed_slope(output_left_speed_dps, desired_left_dps);
    output_right_speed_dps = ramp_fixed_slope(output_right_speed_dps, desired_right_dps);

    prizm.setMotorSpeeds((int)output_left_speed_dps, (int)output_right_speed_dps);

    // Wheels telemetry: joint pose 0..359 deg per wheel @ 50 Hz
    static uint32_t last_wheels_telemetry_ms = 0;
    if (now_ms - last_wheels_telemetry_ms >= 20)
    {
        const long raw_left_deg = prizm.readEncoderDegrees(1);
        const long raw_right_deg = prizm.readEncoderDegrees(2);

        const long norm_left_deg = (long)ENCODER_LEFT_SIGN * raw_left_deg;
        const long norm_right_deg = (long)ENCODER_RIGHT_SIGN * raw_right_deg;

        const uint16_t left_deg_mod = wrap_degrees_0_359(norm_left_deg);
        const uint16_t right_deg_mod = wrap_degrees_0_359(norm_right_deg);

        uint8_t payload[7];
        const size_t n = build_tel_wheels_deg_mod(payload, left_deg_mod, right_deg_mod);
        packet_serial.send(payload, n);

        raw_left_degrees_last = raw_left_deg;
        raw_right_degrees_last = raw_right_deg;
        last_wheels_telemetry_ms = now_ms;
    }

    // Status telemetry @ 1 Hz for diagnostics
    static uint32_t last_status_telemetry_ms = 0;
    if (now_ms - last_status_telemetry_ms >= 1000)
    {
        const float battery_V = prizm.readBatteryVoltage() / 100.0f;
        const uint16_t battery_cV = (uint16_t)lroundf(battery_V * 100.0f);

        uint8_t payload[9];
        const size_t n = build_tel_status(payload, battery_cV, now_ms - boot_time_ms);
        packet_serial.send(payload, n);

        last_status_telemetry_ms = now_ms;
    }
}
