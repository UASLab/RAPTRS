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
const uint8_t command_mode_id = 10;
const uint8_t command_effectors_id = 11;
const uint8_t config_ack_id = 20;
const uint8_t config_basic_id = 21;
const uint8_t config_mpu9250_id = 22;
const uint8_t config_bme280_id = 23;
const uint8_t config_ublox_id = 24;
const uint8_t config_ams5915_id = 25;
const uint8_t config_swift_id = 26;
const uint8_t config_analog_id = 27;
const uint8_t config_effector_id = 28;
const uint8_t config_mission_id = 29;
const uint8_t config_control_gain_id = 30;
const uint8_t data_time_id = 40;
const uint8_t data_mpu9250_short_id = 41;
const uint8_t data_mpu9250_id = 42;
const uint8_t data_bme280_id = 43;
const uint8_t data_ublox_id = 44;
const uint8_t data_ams5915_id = 45;
const uint8_t data_swift_id = 46;
const uint8_t data_sbus_id = 47;
const uint8_t data_analog_id = 48;
const uint8_t data_compound_id = 49;

// max of one byte used to store message len
static const uint8_t message_max_len = 255;

// Constants
static const uint8_t num_effectors = 16;  // number of effector channels
static const uint8_t max_calibration = 4;  // maximum nubmer of calibration coefficients
static const float accel_scale = 208.82724;  // 1 / (9.807(g) * 16 / 32767.5) (+/-16g)
static const float gyro_scale = 938.71973;  // 1 / (2000.0f/32767.5f * d2r (+/-2000deg/sec)
static const float mag_scale = 300;  // fits range

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

// Message: command_mode (id: 10)
struct command_mode_t {
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
    int len = 0;

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
    int len = 0;

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

// Message: config_ack (id: 20)
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
    static const uint8_t id = 20;
    int len = 0;

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
    static const uint8_t id = 21;
    int len = 0;

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

// Message: config_mpu9250 (id: 22)
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
    static const uint8_t id = 22;
    int len = 0;

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

// Message: config_bme280 (id: 23)
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
    static const uint8_t id = 23;
    int len = 0;

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

// Message: config_ublox (id: 24)
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
    static const uint8_t id = 24;
    int len = 0;

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
    static const uint8_t id = 25;
    int len = 0;

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

// Message: config_swift (id: 26)
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
    static const uint8_t id = 26;
    int len = 0;

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
    static const uint8_t id = 27;
    int len = 0;

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

// Message: config_effector (id: 28)
struct config_effector_t {
    // public fields
    effector_type effector;
    string input;
    uint8_t channel;
    float calibration[max_calibration];
    float safed_command;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t effector;
        uint8_t input_len;
        uint8_t channel;
        float calibration[max_calibration];
        float safed_command;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 28;
    int len = 0;

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

// Message: config_mission (id: 29)
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
    static const uint8_t id = 29;
    int len = 0;

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

// Message: config_control_gain (id: 30)
struct config_control_gain_t {
    // public fields
    string level_name;
    string input;
    string output;
    float gain = 1.0;
    bool has_limits;
    float upper_limit;
    float lower_limit;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t level_name_len;
        uint8_t input_len;
        uint8_t output_len;
        float gain;
        bool has_limits;
        float upper_limit;
        float lower_limit;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 30;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        size += level_name.length();
        size += input.length();
        size += output.length();
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->level_name_len = level_name.length();
        _buf->input_len = input.length();
        _buf->output_len = output.length();
        _buf->gain = gain;
        _buf->has_limits = has_limits;
        _buf->upper_limit = upper_limit;
        _buf->lower_limit = lower_limit;
        memcpy(&(payload[len]), level_name.c_str(), level_name.length());
        len += level_name.length();
        memcpy(&(payload[len]), input.c_str(), input.length());
        len += input.length();
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
        gain = _buf->gain;
        has_limits = _buf->has_limits;
        upper_limit = _buf->upper_limit;
        lower_limit = _buf->lower_limit;
        level_name = string((char *)&(payload[len]), _buf->level_name_len);
        len += _buf->level_name_len;
        input = string((char *)&(payload[len]), _buf->input_len);
        len += _buf->input_len;
        output = string((char *)&(payload[len]), _buf->output_len);
        len += _buf->output_len;
        return true;
    }
};

// Message: data_time (id: 40)
struct data_time_t {
    // public fields
    uint64_t time_us;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint64_t time_us;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 40;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->time_us = time_us;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        time_us = _buf->time_us;
        return true;
    }
};

// Message: data_mpu9250_short (id: 41)
struct data_mpu9250_short_t {
    // public fields
    int8_t ReadStatus;
    float AccelX_mss;
    float AccelY_mss;
    float AccelZ_mss;
    float GyroX_rads;
    float GyroY_rads;
    float GyroZ_rads;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        int8_t ReadStatus;
        int16_t AccelX_mss;
        int16_t AccelY_mss;
        int16_t AccelZ_mss;
        int16_t GyroX_rads;
        int16_t GyroY_rads;
        int16_t GyroZ_rads;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 41;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->ReadStatus = ReadStatus;
        _buf->AccelX_mss = intround(AccelX_mss * accel_scale);
        _buf->AccelY_mss = intround(AccelY_mss * accel_scale);
        _buf->AccelZ_mss = intround(AccelZ_mss * accel_scale);
        _buf->GyroX_rads = intround(GyroX_rads * gyro_scale);
        _buf->GyroY_rads = intround(GyroY_rads * gyro_scale);
        _buf->GyroZ_rads = intround(GyroZ_rads * gyro_scale);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        ReadStatus = _buf->ReadStatus;
        AccelX_mss = _buf->AccelX_mss / (float)accel_scale;
        AccelY_mss = _buf->AccelY_mss / (float)accel_scale;
        AccelZ_mss = _buf->AccelZ_mss / (float)accel_scale;
        GyroX_rads = _buf->GyroX_rads / (float)gyro_scale;
        GyroY_rads = _buf->GyroY_rads / (float)gyro_scale;
        GyroZ_rads = _buf->GyroZ_rads / (float)gyro_scale;
        return true;
    }
};

// Message: data_mpu9250 (id: 42)
struct data_mpu9250_t {
    // public fields
    int8_t ReadStatus;
    float AccelX_mss;
    float AccelY_mss;
    float AccelZ_mss;
    float GyroX_rads;
    float GyroY_rads;
    float GyroZ_rads;
    float MagX_uT;
    float MagY_uT;
    float MagZ_uT;
    float Temperature_C;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        int8_t ReadStatus;
        int16_t AccelX_mss;
        int16_t AccelY_mss;
        int16_t AccelZ_mss;
        int16_t GyroX_rads;
        int16_t GyroY_rads;
        int16_t GyroZ_rads;
        int16_t MagX_uT;
        int16_t MagY_uT;
        int16_t MagZ_uT;
        int16_t Temperature_C;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 42;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->ReadStatus = ReadStatus;
        _buf->AccelX_mss = intround(AccelX_mss * accel_scale);
        _buf->AccelY_mss = intround(AccelY_mss * accel_scale);
        _buf->AccelZ_mss = intround(AccelZ_mss * accel_scale);
        _buf->GyroX_rads = intround(GyroX_rads * gyro_scale);
        _buf->GyroY_rads = intround(GyroY_rads * gyro_scale);
        _buf->GyroZ_rads = intround(GyroZ_rads * gyro_scale);
        _buf->MagX_uT = intround(MagX_uT * mag_scale);
        _buf->MagY_uT = intround(MagY_uT * mag_scale);
        _buf->MagZ_uT = intround(MagZ_uT * mag_scale);
        _buf->Temperature_C = intround(Temperature_C * 100);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        ReadStatus = _buf->ReadStatus;
        AccelX_mss = _buf->AccelX_mss / (float)accel_scale;
        AccelY_mss = _buf->AccelY_mss / (float)accel_scale;
        AccelZ_mss = _buf->AccelZ_mss / (float)accel_scale;
        GyroX_rads = _buf->GyroX_rads / (float)gyro_scale;
        GyroY_rads = _buf->GyroY_rads / (float)gyro_scale;
        GyroZ_rads = _buf->GyroZ_rads / (float)gyro_scale;
        MagX_uT = _buf->MagX_uT / (float)mag_scale;
        MagY_uT = _buf->MagY_uT / (float)mag_scale;
        MagZ_uT = _buf->MagZ_uT / (float)mag_scale;
        Temperature_C = _buf->Temperature_C / (float)100;
        return true;
    }
};

// Message: data_bme280 (id: 43)
struct data_bme280_t {
    // public fields
    int8_t ReadStatus;
    float Pressure_Pa;
    float Temperature_C;
    float Humidity_RH;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        int8_t ReadStatus;
        float Pressure_Pa;
        int16_t Temperature_C;
        float Humidity_RH;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 43;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->ReadStatus = ReadStatus;
        _buf->Pressure_Pa = Pressure_Pa;
        _buf->Temperature_C = intround(Temperature_C * 100);
        _buf->Humidity_RH = Humidity_RH;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        ReadStatus = _buf->ReadStatus;
        Pressure_Pa = _buf->Pressure_Pa;
        Temperature_C = _buf->Temperature_C / (float)100;
        Humidity_RH = _buf->Humidity_RH;
        return true;
    }
};

// Message: data_ublox (id: 44)
struct data_ublox_t {
    // public fields
    bool Fix;
    uint8_t NumberSatellites;
    uint32_t TOW;
    uint16_t Year;
    uint8_t Month;
    uint8_t Day;
    uint8_t Hour;
    uint8_t Min;
    uint8_t Sec;
    double Latitude_rad;
    double Longitude_rad;
    float Altitude_m;
    float NorthVelocity_ms;
    float EastVelocity_ms;
    float DownVelocity_ms;
    float HorizontalAccuracy_m;
    float VerticalAccuracy_m;
    float VelocityAccuracy_ms;
    float pDOP;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        bool Fix;
        uint8_t NumberSatellites;
        uint32_t TOW;
        uint16_t Year;
        uint8_t Month;
        uint8_t Day;
        uint8_t Hour;
        uint8_t Min;
        uint8_t Sec;
        double Latitude_rad;
        double Longitude_rad;
        float Altitude_m;
        int16_t NorthVelocity_ms;
        int16_t EastVelocity_ms;
        int16_t DownVelocity_ms;
        float HorizontalAccuracy_m;
        int16_t VerticalAccuracy_m;
        int16_t VelocityAccuracy_ms;
        int16_t pDOP;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 44;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->Fix = Fix;
        _buf->NumberSatellites = NumberSatellites;
        _buf->TOW = TOW;
        _buf->Year = Year;
        _buf->Month = Month;
        _buf->Day = Day;
        _buf->Hour = Hour;
        _buf->Min = Min;
        _buf->Sec = Sec;
        _buf->Latitude_rad = Latitude_rad;
        _buf->Longitude_rad = Longitude_rad;
        _buf->Altitude_m = Altitude_m;
        _buf->NorthVelocity_ms = intround(NorthVelocity_ms * 100);
        _buf->EastVelocity_ms = intround(EastVelocity_ms * 100);
        _buf->DownVelocity_ms = intround(DownVelocity_ms * 100);
        _buf->HorizontalAccuracy_m = HorizontalAccuracy_m;
        _buf->VerticalAccuracy_m = intround(VerticalAccuracy_m * 100);
        _buf->VelocityAccuracy_ms = intround(VelocityAccuracy_ms * 100);
        _buf->pDOP = intround(pDOP * 100);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        Fix = _buf->Fix;
        NumberSatellites = _buf->NumberSatellites;
        TOW = _buf->TOW;
        Year = _buf->Year;
        Month = _buf->Month;
        Day = _buf->Day;
        Hour = _buf->Hour;
        Min = _buf->Min;
        Sec = _buf->Sec;
        Latitude_rad = _buf->Latitude_rad;
        Longitude_rad = _buf->Longitude_rad;
        Altitude_m = _buf->Altitude_m;
        NorthVelocity_ms = _buf->NorthVelocity_ms / (float)100;
        EastVelocity_ms = _buf->EastVelocity_ms / (float)100;
        DownVelocity_ms = _buf->DownVelocity_ms / (float)100;
        HorizontalAccuracy_m = _buf->HorizontalAccuracy_m;
        VerticalAccuracy_m = _buf->VerticalAccuracy_m / (float)100;
        VelocityAccuracy_ms = _buf->VelocityAccuracy_ms / (float)100;
        pDOP = _buf->pDOP / (float)100;
        return true;
    }
};

// Message: data_ams5915 (id: 45)
struct data_ams5915_t {
    // public fields
    int8_t ReadStatus;
    float Pressure_Pa;
    float Temperature_C;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        int8_t ReadStatus;
        float Pressure_Pa;
        int16_t Temperature_C;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 45;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->ReadStatus = ReadStatus;
        _buf->Pressure_Pa = Pressure_Pa;
        _buf->Temperature_C = intround(Temperature_C * 100);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        ReadStatus = _buf->ReadStatus;
        Pressure_Pa = _buf->Pressure_Pa;
        Temperature_C = _buf->Temperature_C / (float)100;
        return true;
    }
};

// Message: data_swift (id: 46)
struct data_swift_t {
    // public fields
    int8_t static_ReadStatus;
    float static_Pressure_Pa;
    float static_Temperature_C;
    int8_t diff_ReadStatus;
    float diff_Pressure_Pa;
    float diff_Temperature_C;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        int8_t static_ReadStatus;
        float static_Pressure_Pa;
        int16_t static_Temperature_C;
        int8_t diff_ReadStatus;
        float diff_Pressure_Pa;
        int16_t diff_Temperature_C;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 46;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->static_ReadStatus = static_ReadStatus;
        _buf->static_Pressure_Pa = static_Pressure_Pa;
        _buf->static_Temperature_C = intround(static_Temperature_C * 100);
        _buf->diff_ReadStatus = diff_ReadStatus;
        _buf->diff_Pressure_Pa = diff_Pressure_Pa;
        _buf->diff_Temperature_C = intround(diff_Temperature_C * 100);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        static_ReadStatus = _buf->static_ReadStatus;
        static_Pressure_Pa = _buf->static_Pressure_Pa;
        static_Temperature_C = _buf->static_Temperature_C / (float)100;
        diff_ReadStatus = _buf->diff_ReadStatus;
        diff_Pressure_Pa = _buf->diff_Pressure_Pa;
        diff_Temperature_C = _buf->diff_Temperature_C / (float)100;
        return true;
    }
};

// Message: data_sbus (id: 47)
struct data_sbus_t {
    // public fields
    float channels[16];
    bool FailSafe;
    uint32_t LostFrames;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        int16_t channels[16];
        bool FailSafe;
        uint32_t LostFrames;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 47;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        for (int _i=0; _i<16; _i++) _buf->channels[_i] = intround(channels[_i] * 20000);
        _buf->FailSafe = FailSafe;
        _buf->LostFrames = LostFrames;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        for (int _i=0; _i<16; _i++) channels[_i] = _buf->channels[_i] / (float)20000;
        FailSafe = _buf->FailSafe;
        LostFrames = _buf->LostFrames;
        return true;
    }
};

// Message: data_analog (id: 48)
struct data_analog_t {
    // public fields
    float calibrated_value;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        float calibrated_value;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 48;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->calibrated_value = calibrated_value;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        calibrated_value = _buf->calibrated_value;
        return true;
    }
};

// Message: data_compound (id: 49)
struct data_compound_t {
    // public fields

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 49;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        return true;
    }
};

} // namespace message
