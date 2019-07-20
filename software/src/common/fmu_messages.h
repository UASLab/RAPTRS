#pragma once

#include <stdint.h>  // uint8_t, et. al.
#include <string.h>  // memcpy()

#include <string>
using std::string;

namespace message {

static inline int32_t intround(float f) {
    return (int32_t)(f >= 0.0 ? (f + 0.5) : (f - 0.5));
}

static inline uint32_t uintround(float f) {
    return (int32_t)(f + 0.5);
}

// Message id constants
const uint8_t mode_command_id = 10;
const uint8_t command_effectors_id = 11;
const uint8_t config_ack_id = 12;
const uint8_t config_basic_id = 13;
const uint8_t config_mpu9250_id = 14;
const uint8_t config_bme280_id = 15;
const uint8_t config_ublox_id = 16;
const uint8_t config_ams5915_id = 17;
const uint8_t config_swift_id = 18;
const uint8_t config_analog_id = 19;
const uint8_t config_effector_id = 20;
const uint8_t config_mission_id = 21;

// max of one byte used to store message len
static const uint8_t message_max_len = 255;

// Constants
static const uint8_t num_effectors = 16;  // number of effector channels
static const uint8_t max_calibration = 4;  // maximum nubmer of calibration coefficients

// Enums
enum class sensor_type {
    time = 0,
    input_voltage = 1,
    regulated_voltage = 2,
    pwm_voltage = 3,
    sbus_voltage = 4,
    internal_bme280 = 5,
    sbus = 6
};
enum class effector_type {
    motor = 0,
    pwm = 1,
    sbus = 2
};

// Message: mode_command (id: 10)
struct mode_command_t {
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

// Message: command_effectors (id: 11)
struct command_effectors_t {
    // public fields
    uint8_t num_active;
    float command[num_effectors];

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t num_active;
        int16_t command[num_effectors];
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 11;
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
        for (int _i=0; _i<num_effectors; _i++) _buf->command[_i] = intround(command[_i] * 32767);
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
        for (int _i=0; _i<num_effectors; _i++) command[_i] = _buf->command[_i] / (float)32767;
        return true;
    }
};

// Message: config_ack (id: 12)
struct config_ack_t {
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

// Message: config_basic (id: 13)
struct config_basic_t {
    // public fields
    sensor_type sensor;
    string output;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t sensor;
        uint8_t output_len;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 13;
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
        _buf->sensor = (uint8_t)sensor;
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
        sensor = (sensor_type)_buf->sensor;
        output = string((char *)&(payload[len]), _buf->output_len);
        len += _buf->output_len;
        return true;
    }
};

// Message: config_mpu9250 (id: 14)
struct config_mpu9250_t {
    // public fields
    bool internal;
    uint8_t SRD;
    float orientation[9];
    int8_t DLPF_bandwidth_hz;
    string output;
    bool use_spi;
    uint8_t spi_bus;
    uint8_t cs_pin;
    uint8_t mosi_pin;
    uint8_t miso_pin;
    uint8_t sck_pin;
    uint8_t i2c_bus;
    uint8_t i2c_addr;

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
        uint8_t spi_bus;
        uint8_t cs_pin;
        uint8_t mosi_pin;
        uint8_t miso_pin;
        uint8_t sck_pin;
        uint8_t i2c_bus;
        uint8_t i2c_addr;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 14;
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
        _buf->spi_bus = spi_bus;
        _buf->cs_pin = cs_pin;
        _buf->mosi_pin = mosi_pin;
        _buf->miso_pin = miso_pin;
        _buf->sck_pin = sck_pin;
        _buf->i2c_bus = i2c_bus;
        _buf->i2c_addr = i2c_addr;
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
        spi_bus = _buf->spi_bus;
        cs_pin = _buf->cs_pin;
        mosi_pin = _buf->mosi_pin;
        miso_pin = _buf->miso_pin;
        sck_pin = _buf->sck_pin;
        i2c_bus = _buf->i2c_bus;
        i2c_addr = _buf->i2c_addr;
        output = string((char *)&(payload[len]), _buf->output_len);
        len += _buf->output_len;
        return true;
    }
};

// Message: config_bme280 (id: 15)
struct config_bme280_t {
    // public fields
    bool use_spi;
    uint8_t spi_bus;
    uint8_t cs_pin;
    uint8_t mosi_pin;
    uint8_t miso_pin;
    uint8_t sck_pin;
    uint8_t i2c_bus;
    uint8_t i2c_addr;
    string output;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        bool use_spi;
        uint8_t spi_bus;
        uint8_t cs_pin;
        uint8_t mosi_pin;
        uint8_t miso_pin;
        uint8_t sck_pin;
        uint8_t i2c_bus;
        uint8_t i2c_addr;
        uint8_t output_len;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 15;
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
        _buf->spi_bus = spi_bus;
        _buf->cs_pin = cs_pin;
        _buf->mosi_pin = mosi_pin;
        _buf->miso_pin = miso_pin;
        _buf->sck_pin = sck_pin;
        _buf->i2c_bus = i2c_bus;
        _buf->i2c_addr = i2c_addr;
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
        spi_bus = _buf->spi_bus;
        cs_pin = _buf->cs_pin;
        mosi_pin = _buf->mosi_pin;
        miso_pin = _buf->miso_pin;
        sck_pin = _buf->sck_pin;
        i2c_bus = _buf->i2c_bus;
        i2c_addr = _buf->i2c_addr;
        output = string((char *)&(payload[len]), _buf->output_len);
        len += _buf->output_len;
        return true;
    }
};

// Message: config_ublox (id: 16)
struct config_ublox_t {
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
    static const uint8_t id = 16;
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

// Message: config_ams5915 (id: 17)
struct config_ams5915_t {
    // public fields
    uint8_t i2c_bus;
    uint8_t i2c_addr;
    string transducer;
    string output;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t i2c_bus;
        uint8_t i2c_addr;
        uint8_t transducer_len;
        uint8_t output_len;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 17;
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
        _buf->i2c_bus = i2c_bus;
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
        i2c_bus = _buf->i2c_bus;
        i2c_addr = _buf->i2c_addr;
        transducer = string((char *)&(payload[len]), _buf->transducer_len);
        len += _buf->transducer_len;
        output = string((char *)&(payload[len]), _buf->output_len);
        len += _buf->output_len;
        return true;
    }
};

// Message: config_swift (id: 18)
struct config_swift_t {
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
    static const uint8_t id = 18;
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

// Message: config_analog (id: 19)
struct config_analog_t {
    // public fields
    uint8_t channel;
    float calibration[max_calibration];
    string output;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t channel;
        float calibration[max_calibration];
        uint8_t output_len;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 19;
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
        for (int _i=0; _i<max_calibration; _i++) _buf->calibration[_i] = calibration[_i];
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
        for (int _i=0; _i<max_calibration; _i++) calibration[_i] = _buf->calibration[_i];
        output = string((char *)&(payload[len]), _buf->output_len);
        len += _buf->output_len;
        return true;
    }
};

// Message: config_effector (id: 20)
struct config_effector_t {
    // public fields
    effector_type effector;
    string input;
    uint8_t channel;
    float calibration[max_calibration];
    uint8_t safed_command;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t effector;
        uint8_t input_len;
        uint8_t channel;
        float calibration[max_calibration];
        uint8_t safed_command;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 20;
    uint16_t len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        size += input.length();
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->effector = (uint8_t)effector;
        _buf->input_len = input.length();
        _buf->channel = channel;
        for (int _i=0; _i<max_calibration; _i++) _buf->calibration[_i] = calibration[_i];
        _buf->safed_command = safed_command;
        memcpy(&(payload[len]), input.c_str(), input.length());
        len += input.length();
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        effector = (effector_type)_buf->effector;
        channel = _buf->channel;
        for (int _i=0; _i<max_calibration; _i++) calibration[_i] = _buf->calibration[_i];
        safed_command = _buf->safed_command;
        input = string((char *)&(payload[len]), _buf->input_len);
        len += _buf->input_len;
        return true;
    }
};

// Message: config_mission (id: 21)
struct config_mission_t {
    // public fields
    string switch_name;
    string source;
    float gain = 1.0;
    float threshold = 0.5;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t switch_name_len;
        uint8_t source_len;
        float gain;
        float threshold;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 21;
    uint16_t len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        size += switch_name.length();
        size += source.length();
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->switch_name_len = switch_name.length();
        _buf->source_len = source.length();
        _buf->gain = gain;
        _buf->threshold = threshold;
        memcpy(&(payload[len]), switch_name.c_str(), switch_name.length());
        len += switch_name.length();
        memcpy(&(payload[len]), source.c_str(), source.length());
        len += source.length();
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        gain = _buf->gain;
        threshold = _buf->threshold;
        switch_name = string((char *)&(payload[len]), _buf->switch_name_len);
        len += _buf->switch_name_len;
        source = string((char *)&(payload[len]), _buf->source_len);
        len += _buf->source_len;
        return true;
    }
};

} // namespace message
