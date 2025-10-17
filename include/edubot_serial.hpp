// include/edubot_serial.hpp
// Reusable framed-CBOR transport + command decoding for PRIZM controllers.
// Frame: [0xAA 0x55][msg_type:1][length:2 little-endian][CBOR payload][CRC16-CCITT:2 little-endian]
// CBOR payload: {1: linear_x_mmps (int), 2: angular_z_mrps (int), 3: enable (int 0/1)} (enable inferred from non-zero vels)

#pragma once
#include <Arduino.h>
#include <tinycbor.h> // TinyCBOR correct include name on Arduino/PIO

// ----- Framing constants -----
static const uint8_t FRAME_SYNC_0 = 0xAA;
static const uint8_t FRAME_SYNC_1 = 0x55;
static const uint8_t FRAME_TYPE_COMMAND = 0x01;

// ----- Transport tunables -----
static const uint16_t MAX_CBOR_PAYLOAD_BYTES = 256; // sized for UNO-class controllers

// ----- CRC16-CCITT (poly 0x1021, init 0xFFFF) -----
static inline uint16_t crc16_ccitt(const uint8_t *data, uint16_t length, uint16_t crc = 0xFFFF)
{
    for (uint16_t i = 0; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; ++b)
        {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// ----- Command schema keys (CBOR map integer keys) -----
// 1: linear_x_mmps (int, mm/s)
// 2: angular_z_mrps (int, milli-rad/s)
// 3: enable (int 0/1) [optional, inferred from non-zero velocities]

struct VelocityCommand
{
    float linear_x_mps = 0.0f;
    float angular_z_rps = 0.0f;
    bool enable = false;
    uint16_t watchdog_ms = 100; // hardcoded

    // Hardcoded kinematics (no longer from CBOR)
    float wheel_radius_m = 0.0508f; // 4.0 in diameter / 2 = 0.0508 m
    float track_width_m = 0.28636f; // center-to-center
};

class FramedCborReceiver
{
public:
    FramedCborReceiver()
        : parse_state_(State::FindSync0), message_type_(0), message_length_(0),
          payload_bytes_read_(0), crc_received_(0) {}

    bool parseByte(uint8_t byte, VelocityCommand &out_command)
    {
        switch (parse_state_)
        {
        case State::FindSync0:
            parse_state_ = (byte == FRAME_SYNC_0) ? State::FindSync1 : State::FindSync0;
            break;
        case State::FindSync1:
            parse_state_ = (byte == FRAME_SYNC_1) ? State::ReadType : State::FindSync0;
            break;
        case State::ReadType:
            message_type_ = byte;
            parse_state_ = State::ReadLen0;
            break;
        case State::ReadLen0:
            message_length_ = byte;
            parse_state_ = State::ReadLen1;
            break;
        case State::ReadLen1:
            message_length_ |= ((uint16_t)byte << 8);
            if (message_length_ == 0 || message_length_ > MAX_CBOR_PAYLOAD_BYTES)
            {
                resetParser();
                break;
            }
            payload_bytes_read_ = 0;
            parse_state_ = State::ReadPayload;
            break;
        case State::ReadPayload:
            payload_[payload_bytes_read_++] = byte;
            if (payload_bytes_read_ >= message_length_)
            {
                parse_state_ = State::ReadCrc0;
            }
            break;
        case State::ReadCrc0:
            crc_received_ = byte;
            parse_state_ = State::ReadCrc1;
            break;
        case State::ReadCrc1:
        {
            crc_received_ |= ((uint16_t)byte << 8);
            bool ok = validateAndDecode(out_command);
            resetParser();
            return ok;
        }
        default:
            resetParser();
            break;
        }
        return false;
    }

private:
    enum class State : uint8_t
    {
        FindSync0,
        FindSync1,
        ReadType,
        ReadLen0,
        ReadLen1,
        ReadPayload,
        ReadCrc0,
        ReadCrc1
    };

    void resetParser()
    {
        parse_state_ = State::FindSync0;
        message_type_ = 0;
        message_length_ = 0;
        payload_bytes_read_ = 0;
        crc_received_ = 0;
    }

    bool validateAndDecode(VelocityCommand &out_command)
    {
        if (message_type_ != FRAME_TYPE_COMMAND)
            return false;

        uint8_t header[3] = {message_type_, (uint8_t)(message_length_ & 0xFF), (uint8_t)(message_length_ >> 8)};
        uint16_t crc_computed = crc16_ccitt(header, 3);
        crc_computed = crc16_ccitt(payload_, message_length_, crc_computed);

        if (crc_computed != crc_received_)
        {
            return false;
        }
        return decodeCborCommand(payload_, message_length_, out_command);
    }

    static bool decodeCborCommand(const uint8_t *buffer, uint16_t length, VelocityCommand &out_command)
    {
        CborParser parser;
        CborValue root;
        if (cbor_parser_init(buffer, length, 0, &parser, &root) != CborNoError)
            return false;
        if (!cbor_value_is_map(&root))
            return false;

        CborValue it;
        if (cbor_value_enter_container(&root, &it) != CborNoError)
            return false;

        VelocityCommand temp = out_command;
        const float scale_vel = 1000.0f; // mm/s and milli-rad/s

        while (!cbor_value_at_end(&it))
        {
            // Get key
            int64_t key = -1;
            if (!cbor_value_is_integer(&it))
            {
                if (cbor_value_advance(&it) != CborNoError)
                    return false;
                continue;
            }
            if (cbor_value_get_int64(&it, &key) != CborNoError)
            {
                if (cbor_value_advance(&it) != CborNoError)
                    return false;
                continue;
            }
            if (cbor_value_advance(&it) != CborNoError)
                return false; // To value

            // Process value if integer
            if (!cbor_value_is_integer(&it) && !cbor_value_is_unsigned_integer(&it))
            {
                if (cbor_value_advance(&it) != CborNoError)
                    return false;
                continue;
            }
            int64_t val = 0;
            if (cbor_value_get_int64(&it, &val) != CborNoError)
            {
                if (cbor_value_advance(&it) != CborNoError)
                    return false;
                continue;
            }
            if (cbor_value_advance(&it) != CborNoError)
                return false; // To next key

            // Apply by key
            switch (key)
            {
            case 1:
                temp.linear_x_mps = (float)val / scale_vel;
                break;
            case 2:
                temp.angular_z_rps = (float)val / scale_vel;
                break;
            case 3:
                temp.enable = (val != 0);
                break;
            default:
                // Skip unknown
                continue;
            }
        }
        if (cbor_value_leave_container(&root, &it) != CborNoError)
            return false;

        // Infer enable from non-zero velocities if not set
        if (!temp.enable)
        {
            temp.enable = (fabs(temp.linear_x_mps) > 0.01f || fabs(temp.angular_z_rps) > 0.01f);
        }
        out_command = temp;
        return true;
    }

    // Parser state
    State parse_state_;
    uint8_t message_type_;
    uint16_t message_length_;
    uint16_t payload_bytes_read_;
    uint8_t payload_[MAX_CBOR_PAYLOAD_BYTES];
    uint16_t crc_received_;
};
