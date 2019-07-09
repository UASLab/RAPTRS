#pragma once

#include <stdint.h>  // uint8_t, et. al.
#include <string.h>  // memcpy()

static inline int32_t intround(float f) {
    return (int32_t)(f >= 0.0 ? (f + 0.5) : (f - 0.5));
}

static inline uint32_t uintround(float f) {
    return (int32_t)(f + 0.5);
}

// Message id constants
const uint8_t message_mode_command_id = 10;
const uint8_t message_effector_command_id = 12;
const uint8_t message_config_ack_id = 20;
const uint8_t message_config_time_id = 21;
const uint8_t message_config_input_voltage_id = 22;

// max of one byte used to store message len
static const uint8_t message_max_len = 255;

#include <string>
using std::string;

static const uint8_t message_num_effectors = 16;  // nubmer of effector channels

// Message: mode_command (id: 10)
struct message_mode_command_t {
    // public fields
    int8_t mode;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        int8_t mode;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 10;
    uint16_t len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->mode = mode;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        mode = _buf->mode;
        return true;
    }
};

// Message: effector_command (id: 12)
struct message_effector_command_t {
    // public fields
    uint8_t num_active;
    float command[message_num_effectors];

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t num_active;
        int16_t command[message_num_effectors];
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 12;
    uint16_t len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->num_active = num_active;
        for (int _i=0; _i<message_num_effectors; _i++) _buf->command[_i] = intround(command[_i] * 32767);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        num_active = _buf->num_active;
        for (int _i=0; _i<message_num_effectors; _i++) command[_i] = _buf->command[_i] / (float)32767;
        return true;
    }
};

// Message: config_ack (id: 20)
struct message_config_ack_t {
    // public fields
    uint8_t ack_id;
    uint8_t ack_subid;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t ack_id;
        uint8_t ack_subid;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 20;
    uint16_t len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->ack_id = ack_id;
        _buf->ack_subid = ack_subid;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        ack_id = _buf->ack_id;
        ack_subid = _buf->ack_subid;
        return true;
    }
};

// Message: config_time (id: 21)
struct message_config_time_t {
    // public fields
    string output;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t output_len;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 21;
    uint16_t len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        size += output.length();
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->output_len = output.length();
        memcpy(&(payload[len]), output.c_str(), output.length());
        len += output.length();
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        output = string((char *)&(payload[len]), _buf->output_len);
        len += _buf->output_len;
        return true;
    }
};

// Message: config_input_voltage (id: 22)
struct message_config_input_voltage_t {
    // public fields
    string output;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t output_len;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 22;
    uint16_t len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        size += output.length();
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->output_len = output.length();
        memcpy(&(payload[len]), output.c_str(), output.length());
        len += output.length();
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        output = string((char *)&(payload[len]), _buf->output_len);
        len += _buf->output_len;
        return true;
    }
};

