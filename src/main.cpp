// src/main.cpp
// Milestone 1: Receive framed CBOR commands (linear_x, angular_z), compute wheel targets using hardcoded kinematics,
// and use PRIZM PID constant speed control via setMotorSpeeds (deg/s).
// Enable inferred from non-zero velocities; stop if stale (>100ms) or zero vels.
// Uses PRIZM library built-ins: setMotorSpeeds (deg/s, PID), resetEncoders, readBatteryVoltage (/100 for V).

#include <Arduino.h>
#include <PRIZM.h>
#include "edubot_serial.hpp"

// ---- Serial configuration ----
static const uint32_t SERIAL_BAUD_RATE = 115200;

// ---- Control loop timing ----
static const float CONTROL_LOOP_FREQUENCY_HZ = 100.0f;
static const uint32_t CONTROL_LOOP_PERIOD_MS = (uint32_t)(1000.0f / CONTROL_LOOP_FREQUENCY_HZ);

// ---- PRIZM speed limits (degrees/second) ----
static const int16_t PRIZM_SPEED_DPS_LIMIT = 360; // Conservative; max ~720 per docs

// ---- Watchdog ----
static const uint16_t WATCHDOG_MS = 100; // Hardcoded

// ---- Globals ----
PRIZM prizm;
FramedCborReceiver framed_cbor_receiver;
VelocityCommand latest_command;
uint32_t last_command_time_ms = 0;
bool startup_test_done = false;

// Utility: radians/second -> degrees/second
static inline float radiansPerSecondToDegreesPerSecond(float value_rps)
{
    return value_rps * (180.0f / 3.14159265358979323846f);
}

// Compute per-wheel angular velocity in rad/s from v (m/s) and omega (rad/s)
static inline void computeWheelAngularVelocities(
    float linear_x_mps,
    float angular_z_rps,
    float wheel_radius_m,
    float track_width_m,
    float &left_wheel_rps,
    float &right_wheel_rps)
{
    const float half_track = 0.5f * track_width_m;
    left_wheel_rps = (linear_x_mps - angular_z_rps * half_track) / max(wheel_radius_m, 1e-6f);
    right_wheel_rps = (linear_x_mps + angular_z_rps * half_track) / max(wheel_radius_m, 1e-6f);
}

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial)
    { /* wait for USB CDC */
    }
    Serial.println("PRIZM Controller Started");

    prizm.PrizmBegin(); // Initialize PRIZM core and motor subsystems
    prizm.resetEncoders();

    // Battery check (divide by 100 per PRIZM docs)
    float battery_v = prizm.readBatteryVoltage() / 100.0f;
    Serial.print("Battery Voltage: ");
    Serial.print(battery_v);
    Serial.println(" V");

    last_command_time_ms = millis();
}

void loop()
{
    // Ingest and parse serial bytes
    while (Serial.available() > 0)
    {
        const uint8_t incoming_byte = (uint8_t)Serial.read();
        if (framed_cbor_receiver.parseByte(incoming_byte, latest_command))
        {
            last_command_time_ms = millis();
        }
    }

    // Control loop timing
    static uint32_t last_loop_ms = millis();
    const uint32_t now_ms = millis();
    if (now_ms - last_loop_ms < CONTROL_LOOP_PERIOD_MS)
    {
        return;
    }
    last_loop_ms += CONTROL_LOOP_PERIOD_MS;

    // Startup test: Brief forward spin if no commands in 5s
    if (!startup_test_done && (now_ms - last_command_time_ms > 5000))
    {
        prizm.setMotorSpeeds(200, 200);
        delay(2000);
        prizm.setMotorSpeeds(0, 0);
        startup_test_done = true;
        Serial.println("Startup Test Complete");
    }

    // Enforce watchdog and enable (infer if velocities non-zero)
    const bool command_is_fresh = (now_ms - last_command_time_ms) <= WATCHDOG_MS;
    const bool velocities_non_zero = (fabs(latest_command.linear_x_mps) > 0.01f || fabs(latest_command.angular_z_rps) > 0.01f);
    const bool motion_is_enabled = latest_command.enable && command_is_fresh && velocities_non_zero;

    // Compute wheel targets
    float left_wheel_rps = 0.0f;
    float right_wheel_rps = 0.0f;
    computeWheelAngularVelocities(
        latest_command.linear_x_mps,
        latest_command.angular_z_rps,
        latest_command.wheel_radius_m,
        latest_command.track_width_m,
        left_wheel_rps,
        right_wheel_rps);

    // Convert to deg/s and clamp
    float left_wheel_dps = radiansPerSecondToDegreesPerSecond(left_wheel_rps);
    float right_wheel_dps = radiansPerSecondToDegreesPerSecond(right_wheel_rps);
    left_wheel_dps = constrain(left_wheel_dps, -PRIZM_SPEED_DPS_LIMIT, PRIZM_SPEED_DPS_LIMIT);
    right_wheel_dps = constrain(right_wheel_dps, -PRIZM_SPEED_DPS_LIMIT, PRIZM_SPEED_DPS_LIMIT);

    // Apply to PRIZM PID
    if (motion_is_enabled)
    {
        prizm.setMotorSpeeds((int)left_wheel_dps, (int)right_wheel_dps);
    }
    else
    {
        prizm.setMotorSpeeds(0, 0);
    }
}
