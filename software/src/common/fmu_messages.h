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
const uint8_t message_config_basic_id = 21;
const uint8_t message_config_mpu9250_id = 22;
const uint8_t message_config_bme280_id = 23;
const uint8_t message_config_ublox_id = 24;
const uint8_t message_config_ams5915_id = 25;
const uint8_t message_config_swift_id = 26;
const uint8_t message_config_analog_id = 27;

// max of one byte used to store message len
static const uint8_t message_max_len = 255;

#include <string>
using std::string;

// Constants
static const uint8_t message_num_effectors = 16;  // nubmer of effector channels

// Enums
enum class config_basic {
    time = 0,
    input_voltage = 1,
    regulated_voltage = 2,
    pwm_voltage = 3,
    sbus_voltage = 4,
    internal_bme280 = 5,
    sbus = 6
};

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

// Message: config_basic (id: 21)
struct message_config_basic_t {
    // public fields
    config_basic device;
    string output;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t device;
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
        _buf->device = (uint8_t)device;
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
        device = (config_basic)_buf->device;
        output = string((char *)&(payload[len]), _buf->output_len);
        len += _buf->output_len;
        return true;
    }
};

// Message: config_mpu9250 (id: 22)
struct message_config_mpu9250_t {
    // public fields
    bool internal;
    uint8_t SRD;
    float orientation[9];
    int8_t DLPF_bandwidth_hz;
    string output;
    bool use_spi;
    uint8_t i2c_addr;
    uint8_t cs_pin;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        bool internal;
        uint8_t SRD;
        float orientation[9];
        int8_t DLPF_bandwidth_hz;
        uint8_t output_len;
        bool use_spi;
        uint8_t i2c_addr;
        uint8_t cs_pin;
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
        _buf->internal = internal;
        _buf->SRD = SRD;
        for (int _i=0; _i<9; _i++) _buf->orientation[_i] = orientation[_i];
        _buf->DLPF_bandwidth_hz = DLPF_bandwidth_hz;
        _buf->output_len = output.length();
        _buf->use_spi = use_spi;
        _buf->i2c_addr = i2c_addr;
        _buf->cs_pin = cs_pin;
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
        internal = _buf->internal;
        SRD = _buf->SRD;
        for (int _i=0; _i<9; _i++) orientation[_i] = _buf->orientation[_i];
        DLPF_bandwidth_hz = _buf->DLPF_bandwidth_hz;
        use_spi = _buf->use_spi;
        i2c_addr = _buf->i2c_addr;
        cs_pin = _buf->cs_pin;
        output = string((char *)&(payload[len]), _buf->output_len);
        len += _buf->output_len;
        return true;
    }
};

// Message: config_bme280 (id: 23)
struct message_config_bme280_t {
    // public fields
    bool use_spi;
    uint8_t i2c_addr;
    uint8_t cs_pin;
    uint8_t mosi_pin;
    uint8_t miso_pin;
    uint8_t sck_pin;
    string output;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        bool use_spi;
        uint8_t i2c_addr;
        uint8_t cs_pin;
        uint8_t mosi_pin;
        uint8_t miso_pin;
        uint8_t sck_pin;
        uint8_t output_len;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 23;
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
        _buf->use_spi = use_spi;
        _buf->i2c_addr = i2c_addr;
        _buf->cs_pin = cs_pin;
        _buf->mosi_pin = mosi_pin;
        _buf->miso_pin = miso_pin;
        _buf->sck_pin = sck_pin;
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
        use_spi = _buf->use_spi;
        i2c_addr = _buf->i2c_addr;
        cs_pin = _buf->cs_pin;
        mosi_pin = _buf->mosi_pin;
        miso_pin = _buf->miso_pin;
        sck_pin = _buf->sck_pin;
        output = string((char *)&(payload[len]), _buf->output_len);
        len += _buf->output_len;
        return true;
    }
};

// Message: config_ublox (id: 24)
struct message_config_ublox_t {
    // public fields
    uint8_t uart;
    uint32_t baud;
    string output;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t uart;
        uint32_t baud;
        uint8_t output_len;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 24;
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
        _buf->uart = uart;
        _buf->baud = baud;
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
        uart = _buf->uart;
        baud = _buf->baud;
        output = string((char *)&(payload[len]), _buf->output_len);
        len += _buf->output_len;
        return true;
    }
};

// Message: config_ams5915 (id: 25)
struct message_config_ams5915_t {
    // public fields
    uint8_t i2c_addr;
    string transducer;
    string output;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t i2c_addr;
        uint8_t transducer_len;
        uint8_t output_len;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 25;
    uint16_t len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        size += transducer.length();
        size += output.length();
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->i2c_addr = i2c_addr;
        _buf->transducer_len = transducer.length();
        _buf->output_len = output.length();
        memcpy(&(payload[len]), transducer.c_str(), transducer.length());
        len += transducer.length();
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
        i2c_addr = _buf->i2c_addr;
        transducer = string((char *)&(payload[len]), _buf->transducer_len);
        len += _buf->transducer_len;
        output = string((char *)&(payload[len]), _buf->output_len);
        len += _buf->output_len;
        return true;
    }
};

// Message: config_swift (id: 26)
struct message_config_swift_t {
    // public fields
    uint8_t i2c_bus;
    uint8_t static_i2c_addr;
    uint8_t diff_i2c_addr;
    string diff_transducer;
    string output;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t i2c_bus;
        uint8_t static_i2c_addr;
        uint8_t diff_i2c_addr;
        uint8_t diff_transducer_len;
        uint8_t output_len;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 26;
    uint16_t len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        size += diff_transducer.length();
        size += output.length();
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->i2c_bus = i2c_bus;
        _buf->static_i2c_addr = static_i2c_addr;
        _buf->diff_i2c_addr = diff_i2c_addr;
        _buf->diff_transducer_len = diff_transducer.length();
        _buf->output_len = output.length();
        memcpy(&(payload[len]), diff_transducer.c_str(), diff_transducer.length());
        len += diff_transducer.length();
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
        i2c_bus = _buf->i2c_bus;
        static_i2c_addr = _buf->static_i2c_addr;
        diff_i2c_addr = _buf->diff_i2c_addr;
        diff_transducer = string((char *)&(payload[len]), _buf->diff_transducer_len);
        len += _buf->diff_transducer_len;
        output = string((char *)&(payload[len]), _buf->output_len);
        len += _buf->output_len;
        return true;
    }
};

// Message: config_analog (id: 27)
struct message_config_analog_t {
    // public fields
    uint8_t channel;
    float calibration[4];
    string output;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t channel;
        float calibration[4];
        uint8_t output_len;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 27;
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
        _buf->channel = channel;
        for (int _i=0; _i<4; _i++) _buf->calibration[_i] = calibration[_i];
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
        channel = _buf->channel;
        for (int _i=0; _i<4; _i++) calibration[_i] = _buf->calibration[_i];
        output = string((char *)&(payload[len]), _buf->output_len);
        len += _buf->output_len;
        return true;
    }
};

