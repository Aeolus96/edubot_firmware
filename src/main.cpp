// main.cpp — Minimal PRIZM + PacketSerial pass-through
// Behavior:
// - COBS/PacketSerial receives PKT_CMD_VELOCITY via edubot_protocol.hpp and stores the last command.
// - Every 10 ms: apply the last fresh command immediately; otherwise command zeros.
// - Always publish wheels and status each iteration (no extra throttling).
// - Drain serial garbage on setup and on overflow to resync after reconnects.

#include <Arduino.h>
#include <PRIZM.h>
#include <PacketSerial.h>
#include "edubot_protocol.hpp"

// ----------------------- Serial config -----------------------
static constexpr uint32_t SERIAL_BAUD = 115200;

// ----------------------- Loop timing -------------------------
static constexpr uint16_t LOOP_PERIOD_MS = 10;       // Fixed 10 ms
static constexpr uint16_t COMMAND_WATCHDOG_MS = 100; // Zeros if stale

// ----------------------- Robot geometry ----------------------
static constexpr float WHEEL_RADIUS_M = 0.0508f;
static constexpr float TRACK_WIDTH_M = 0.28636f;

// ----------------------- Sign conventions --------------------
static constexpr int8_t MOTOR_LEFT_SIGN = -1;
static constexpr int8_t MOTOR_RIGHT_SIGN = +1;
static constexpr int8_t ENCODER_LEFT_SIGN = -1;
static constexpr int8_t ENCODER_RIGHT_SIGN = +1;

// ----------------------- Limits ------------------------------
static constexpr int16_t MAX_MOTOR_SPEED_DPS = 720; // PRIZM dps scale

// ----------------------- Globals -----------------------------
static PacketSerial packet_serial;
static PRIZM prizm;

// Last received command (volatile: written from ISR-like handler)
static volatile int16_t commanded_linear_x_mmps = 0;
static volatile int16_t commanded_angular_z_mrps = 0;
static volatile bool commanded_enable = false;
static volatile uint32_t last_command_time_ms = 0;

static uint32_t boot_time_ms = 0;

// ----------------------- Math helpers ------------------------
static inline float rps_to_dps(float rps)
{
    return rps * (180.0f / 3.14159265358979323846f);
}

static inline void body_to_wheel_rps(float linear_x_mps, float angular_z_rps,
                                     float &out_left_rps, float &out_right_rps)
{
    const float half_track = 0.5f * TRACK_WIDTH_M;
    const float denom = max(WHEEL_RADIUS_M, 1e-6f);
    out_left_rps = (linear_x_mps - angular_z_rps * half_track) / denom;
    out_right_rps = (linear_x_mps + angular_z_rps * half_track) / denom;
}

static inline uint16_t wrap_degrees_0_359(long deg)
{
    long m = deg % 360;
    if (m < 0)
        m += 360;
    return static_cast<uint16_t>(m);
}

static void clearSerialInputBuffer(Stream &s)
{
    // Drain any partial bytes so COBS can find the next clean frame boundary
    while (s.available() > 0)
    {
        (void)s.read();
    }
}

// ----------------------- Packet handler ----------------------
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

// ----------------------- Arduino setup/loop ------------------
void setup()
{
    Serial.begin(SERIAL_BAUD);
    // Attach PacketSerial to Serial and register handler
    packet_serial.setStream(&Serial);
    packet_serial.setPacketHandler(&on_packet_received);

    // Proactively drain any garbage after boot before first update
    clearSerialInputBuffer(Serial);

    // Initialize PRIZM and encoders
    prizm.PrizmBegin();
    prizm.resetEncoders();

    boot_time_ms = millis();
    last_command_time_ms = 0; // force zeros until first valid command
}

void loop()
{
    // Always service COBS decoding
    packet_serial.update();

    // Optional: handle overflow, stop and resync
    if (packet_serial.overflow())
    {
        prizm.setMotorSpeeds(0, 0);
        clearSerialInputBuffer(Serial);
    }

    // Fixed 10 ms loop pacing
    static uint32_t last_tick = 0;
    const uint32_t now = millis();
    if (now - last_tick < LOOP_PERIOD_MS)
    {
        delay(1); // light yield until next tick
        return;
    }
    last_tick = now;

    // Fetch the latest command snapshot
    int16_t lin_mmps, ang_mrps;
    bool en;
    uint32_t last_rx;
    noInterrupts();
    lin_mmps = commanded_linear_x_mmps;
    ang_mrps = commanded_angular_z_mrps;
    en = commanded_enable;
    last_rx = last_command_time_ms;
    interrupts();

    // Watchdog: command considered fresh if within window
    const bool fresh = (now - last_rx) <= COMMAND_WATCHDOG_MS;

    // Compute wheel speeds in DPS directly or zero if stale/disabled
    int16_t left_dps = 0;
    int16_t right_dps = 0;
    if (fresh && (en || (abs(lin_mmps) > 10 || abs(ang_mrps) > 10)))
    {
        const float vx = static_cast<float>(lin_mmps) / 1000.0f; // mm/s -> m/s
        const float wz = static_cast<float>(ang_mrps) / 1000.0f; // mrad/s -> rad/s
        float l_rps = 0.0f, r_rps = 0.0f;
        body_to_wheel_rps(vx, wz, l_rps, r_rps);
        const float l_dps_f = constrain(static_cast<float>(MOTOR_LEFT_SIGN) * rps_to_dps(l_rps),
                                        -static_cast<float>(MAX_MOTOR_SPEED_DPS),
                                        static_cast<float>(MAX_MOTOR_SPEED_DPS));
        const float r_dps_f = constrain(static_cast<float>(MOTOR_RIGHT_SIGN) * rps_to_dps(r_rps),
                                        -static_cast<float>(MAX_MOTOR_SPEED_DPS),
                                        static_cast<float>(MAX_MOTOR_SPEED_DPS));
        left_dps = static_cast<int16_t>(lroundf(l_dps_f));
        right_dps = static_cast<int16_t>(lroundf(r_dps_f));
    }

    // Apply immediately (no ramp)
    prizm.setMotorSpeeds(left_dps, right_dps);

    // Wheels telemetry every loop
    {
        const long raw_l = prizm.readEncoderDegrees(1);
        const long raw_r = prizm.readEncoderDegrees(2);
        const long nl = static_cast<long>(ENCODER_LEFT_SIGN) * raw_l;
        const long nr = static_cast<long>(ENCODER_RIGHT_SIGN) * raw_r;
        uint8_t payload[8]; // sized generously
        const size_t n = build_tel_wheels_deg_mod(payload,
                                                  wrap_degrees_0_359(nl),
                                                  wrap_degrees_0_359(nr));
        packet_serial.send(payload, n);
    }

    // Status telemetry every loop
    {
        const float battery_V = prizm.readBatteryVoltage() / 100.0f; // library returns centivolts
        const uint16_t battery_cV = static_cast<uint16_t>(lroundf(battery_V * 100.0f));
        uint8_t payload[10]; // sized generously
        const size_t n = build_tel_status(payload, battery_cV, now - boot_time_ms);
        packet_serial.send(payload, n);
    }

    // Fixed loop delay (always-on behavior)
    delay(LOOP_PERIOD_MS);
}
