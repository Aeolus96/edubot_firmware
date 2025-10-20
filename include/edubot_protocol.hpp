// include/edubot_protocol.hpp
// Binary protocol using PacketSerial (COBS) framing and CRC16-CCITT for integrity.
// Payload format for each packet: [type u8][fields ...][crc16_le u16], CRC computed over [type+fields], appended little-endian.
// All multi-byte fields are little-endian to minimize parsing code on AVR and Python.

#pragma once
#include <Arduino.h>
#include <stdint.h>

// Packet types
static const uint8_t PKT_CMD_VELOCITY = 0x10; // Host->MCU:  [0x10][linear_x_mmps i16][angular_z_mrps i16][enable u8][crc u16]
static const uint8_t PKT_TEL_WHEELS = 0x21;   // MCU->Host:  [0x21][left_deg_mod u16][right_deg_mod u16][crc u16]
static const uint8_t PKT_TEL_STATUS = 0x22;   // MCU->Host:  [0x22][battery_cV u16][uptime_ms u32][crc u16]

// CRC16-CCITT (poly 0x1021, init 0xFFFF)
static inline uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF)
{
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; ++b)
        {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// Little-endian writers
static inline void wr_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}
static inline void wr_u32_le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}
static inline void wr_i16_le(uint8_t *p, int16_t v) { wr_u16_le(p, (uint16_t)v); }

// Wheels telemetry builder (joint pose 0..359 deg)
static inline size_t build_tel_wheels_deg_mod(uint8_t *out, uint16_t left_deg_mod, uint16_t right_deg_mod)
{
    out[0] = PKT_TEL_WHEELS;
    wr_u16_le(out + 1, left_deg_mod);
    wr_u16_le(out + 3, right_deg_mod);
    const uint16_t crc = crc16_ccitt(out, 5);
    wr_u16_le(out + 5, crc);
    return 7;
}

// Status telemetry builder
static inline size_t build_tel_status(uint8_t *out, uint16_t battery_cV, uint32_t uptime_ms)
{
    out[0] = PKT_TEL_STATUS;
    wr_u16_le(out + 1, battery_cV);
    wr_u32_le(out + 3, uptime_ms);
    const uint16_t crc = crc16_ccitt(out, 7);
    wr_u16_le(out + 7, crc);
    return 9;
}

// Velocity command parser
static inline bool parse_cmd_velocity(const uint8_t *buf, size_t len, int16_t &out_lin_mmps, int16_t &out_ang_mrps, bool &out_enable)
{
    if (len != 8 || buf[0] != PKT_CMD_VELOCITY)
        return false;
    const uint16_t crc_exp = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);
    const uint16_t crc_act = crc16_ccitt(buf, 6);
    if (crc_exp != crc_act)
        return false;
    out_lin_mmps = (int16_t)((uint16_t)buf[1] | ((uint16_t)buf[2] << 8));
    out_ang_mrps = (int16_t)((uint16_t)buf[3] | ((uint16_t)buf[4] << 8));
    out_enable = (buf[5] != 0);
    return true;
}
