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
const uint8_t gps_v2_id = 16;
const uint8_t gps_v3_id = 26;
const uint8_t gps_v4_id = 34;
const uint8_t gps_raw_v1_id = 48;
const uint8_t imu_v3_id = 17;
const uint8_t imu_v4_id = 35;
const uint8_t imu_v5_id = 45;
const uint8_t airdata_v5_id = 18;
const uint8_t airdata_v6_id = 40;
const uint8_t airdata_v7_id = 43;
const uint8_t filter_v3_id = 31;
const uint8_t filter_v4_id = 36;
const uint8_t filter_v5_id = 47;
const uint8_t actuator_v2_id = 21;
const uint8_t actuator_v3_id = 37;
const uint8_t pilot_v2_id = 20;
const uint8_t pilot_v3_id = 38;
const uint8_t ap_status_v4_id = 30;
const uint8_t ap_status_v5_id = 32;
const uint8_t ap_status_v6_id = 33;
const uint8_t ap_status_v7_id = 39;
const uint8_t system_health_v4_id = 19;
const uint8_t system_health_v5_id = 41;
const uint8_t system_health_v6_id = 46;
const uint8_t payload_v2_id = 23;
const uint8_t payload_v3_id = 42;
const uint8_t event_v1_id = 27;
const uint8_t event_v2_id = 44;
const uint8_t command_v1_id = 28;
const uint8_t stream_v1_id = 49;

// max of one byte used to store message len
static const uint8_t message_max_len = 255;

// Constants
static const uint8_t max_raw_sats = 12;  // maximum array size to store satellite raw data

// Message: gps_v2 (id: 16)
struct gps_v2_t {
    // public fields
    uint8_t index;
    double timestamp_sec;
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float vn_ms;
    float ve_ms;
    float vd_ms;
    double unixtime_sec;
    uint8_t satellites;
    uint8_t status;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        double timestamp_sec;
        double latitude_deg;
        double longitude_deg;
        float altitude_m;
        int16_t vn_ms;
        int16_t ve_ms;
        int16_t vd_ms;
        double unixtime_sec;
        uint8_t satellites;
        uint8_t status;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 16;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->latitude_deg = latitude_deg;
        _buf->longitude_deg = longitude_deg;
        _buf->altitude_m = altitude_m;
        _buf->vn_ms = intround(vn_ms * 100);
        _buf->ve_ms = intround(ve_ms * 100);
        _buf->vd_ms = intround(vd_ms * 100);
        _buf->unixtime_sec = unixtime_sec;
        _buf->satellites = satellites;
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        latitude_deg = _buf->latitude_deg;
        longitude_deg = _buf->longitude_deg;
        altitude_m = _buf->altitude_m;
        vn_ms = _buf->vn_ms / (float)100;
        ve_ms = _buf->ve_ms / (float)100;
        vd_ms = _buf->vd_ms / (float)100;
        unixtime_sec = _buf->unixtime_sec;
        satellites = _buf->satellites;
        status = _buf->status;
        return true;
    }
};

// Message: gps_v3 (id: 26)
struct gps_v3_t {
    // public fields
    uint8_t index;
    double timestamp_sec;
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float vn_ms;
    float ve_ms;
    float vd_ms;
    double unixtime_sec;
    uint8_t satellites;
    float horiz_accuracy_m;
    float vert_accuracy_m;
    float pdop;
    uint8_t fix_type;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        double timestamp_sec;
        double latitude_deg;
        double longitude_deg;
        float altitude_m;
        int16_t vn_ms;
        int16_t ve_ms;
        int16_t vd_ms;
        double unixtime_sec;
        uint8_t satellites;
        uint16_t horiz_accuracy_m;
        uint16_t vert_accuracy_m;
        uint16_t pdop;
        uint8_t fix_type;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 26;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->latitude_deg = latitude_deg;
        _buf->longitude_deg = longitude_deg;
        _buf->altitude_m = altitude_m;
        _buf->vn_ms = intround(vn_ms * 100);
        _buf->ve_ms = intround(ve_ms * 100);
        _buf->vd_ms = intround(vd_ms * 100);
        _buf->unixtime_sec = unixtime_sec;
        _buf->satellites = satellites;
        _buf->horiz_accuracy_m = uintround(horiz_accuracy_m * 100);
        _buf->vert_accuracy_m = uintround(vert_accuracy_m * 100);
        _buf->pdop = uintround(pdop * 100);
        _buf->fix_type = fix_type;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        latitude_deg = _buf->latitude_deg;
        longitude_deg = _buf->longitude_deg;
        altitude_m = _buf->altitude_m;
        vn_ms = _buf->vn_ms / (float)100;
        ve_ms = _buf->ve_ms / (float)100;
        vd_ms = _buf->vd_ms / (float)100;
        unixtime_sec = _buf->unixtime_sec;
        satellites = _buf->satellites;
        horiz_accuracy_m = _buf->horiz_accuracy_m / (float)100;
        vert_accuracy_m = _buf->vert_accuracy_m / (float)100;
        pdop = _buf->pdop / (float)100;
        fix_type = _buf->fix_type;
        return true;
    }
};

// Message: gps_v4 (id: 34)
struct gps_v4_t {
    // public fields
    uint8_t index;
    float timestamp_sec;
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float vn_ms;
    float ve_ms;
    float vd_ms;
    double unixtime_sec;
    uint8_t satellites;
    float horiz_accuracy_m;
    float vert_accuracy_m;
    float pdop;
    uint8_t fix_type;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        double latitude_deg;
        double longitude_deg;
        float altitude_m;
        int16_t vn_ms;
        int16_t ve_ms;
        int16_t vd_ms;
        double unixtime_sec;
        uint8_t satellites;
        uint16_t horiz_accuracy_m;
        uint16_t vert_accuracy_m;
        uint16_t pdop;
        uint8_t fix_type;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 34;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->latitude_deg = latitude_deg;
        _buf->longitude_deg = longitude_deg;
        _buf->altitude_m = altitude_m;
        _buf->vn_ms = intround(vn_ms * 100);
        _buf->ve_ms = intround(ve_ms * 100);
        _buf->vd_ms = intround(vd_ms * 100);
        _buf->unixtime_sec = unixtime_sec;
        _buf->satellites = satellites;
        _buf->horiz_accuracy_m = uintround(horiz_accuracy_m * 100);
        _buf->vert_accuracy_m = uintround(vert_accuracy_m * 100);
        _buf->pdop = uintround(pdop * 100);
        _buf->fix_type = fix_type;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        latitude_deg = _buf->latitude_deg;
        longitude_deg = _buf->longitude_deg;
        altitude_m = _buf->altitude_m;
        vn_ms = _buf->vn_ms / (float)100;
        ve_ms = _buf->ve_ms / (float)100;
        vd_ms = _buf->vd_ms / (float)100;
        unixtime_sec = _buf->unixtime_sec;
        satellites = _buf->satellites;
        horiz_accuracy_m = _buf->horiz_accuracy_m / (float)100;
        vert_accuracy_m = _buf->vert_accuracy_m / (float)100;
        pdop = _buf->pdop / (float)100;
        fix_type = _buf->fix_type;
        return true;
    }
};

// Message: gps_raw_v1 (id: 48)
struct gps_raw_v1_t {
    // public fields
    uint8_t index;
    float timestamp_sec;
    double receiver_tow;
    uint8_t num_sats;
    uint8_t svid[max_raw_sats];
    double pseudorange[max_raw_sats];
    double doppler[max_raw_sats];

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        double receiver_tow;
        uint8_t num_sats;
        uint8_t svid[max_raw_sats];
        double pseudorange[max_raw_sats];
        double doppler[max_raw_sats];
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->receiver_tow = receiver_tow;
        _buf->num_sats = num_sats;
        for (int _i=0; _i<max_raw_sats; _i++) _buf->svid[_i] = svid[_i];
        for (int _i=0; _i<max_raw_sats; _i++) _buf->pseudorange[_i] = pseudorange[_i];
        for (int _i=0; _i<max_raw_sats; _i++) _buf->doppler[_i] = doppler[_i];
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        receiver_tow = _buf->receiver_tow;
        num_sats = _buf->num_sats;
        for (int _i=0; _i<max_raw_sats; _i++) svid[_i] = _buf->svid[_i];
        for (int _i=0; _i<max_raw_sats; _i++) pseudorange[_i] = _buf->pseudorange[_i];
        for (int _i=0; _i<max_raw_sats; _i++) doppler[_i] = _buf->doppler[_i];
        return true;
    }
};

// Message: imu_v3 (id: 17)
struct imu_v3_t {
    // public fields
    uint8_t index;
    double timestamp_sec;
    float p_rad_sec;
    float q_rad_sec;
    float r_rad_sec;
    float ax_mps_sec;
    float ay_mps_sec;
    float az_mps_sec;
    float hx;
    float hy;
    float hz;
    float temp_C;
    uint8_t status;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        double timestamp_sec;
        float p_rad_sec;
        float q_rad_sec;
        float r_rad_sec;
        float ax_mps_sec;
        float ay_mps_sec;
        float az_mps_sec;
        float hx;
        float hy;
        float hz;
        int16_t temp_C;
        uint8_t status;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 17;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->p_rad_sec = p_rad_sec;
        _buf->q_rad_sec = q_rad_sec;
        _buf->r_rad_sec = r_rad_sec;
        _buf->ax_mps_sec = ax_mps_sec;
        _buf->ay_mps_sec = ay_mps_sec;
        _buf->az_mps_sec = az_mps_sec;
        _buf->hx = hx;
        _buf->hy = hy;
        _buf->hz = hz;
        _buf->temp_C = intround(temp_C * 10);
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        p_rad_sec = _buf->p_rad_sec;
        q_rad_sec = _buf->q_rad_sec;
        r_rad_sec = _buf->r_rad_sec;
        ax_mps_sec = _buf->ax_mps_sec;
        ay_mps_sec = _buf->ay_mps_sec;
        az_mps_sec = _buf->az_mps_sec;
        hx = _buf->hx;
        hy = _buf->hy;
        hz = _buf->hz;
        temp_C = _buf->temp_C / (float)10;
        status = _buf->status;
        return true;
    }
};

// Message: imu_v4 (id: 35)
struct imu_v4_t {
    // public fields
    uint8_t index;
    float timestamp_sec;
    float p_rad_sec;
    float q_rad_sec;
    float r_rad_sec;
    float ax_mps_sec;
    float ay_mps_sec;
    float az_mps_sec;
    float hx;
    float hy;
    float hz;
    float temp_C;
    uint8_t status;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        float p_rad_sec;
        float q_rad_sec;
        float r_rad_sec;
        float ax_mps_sec;
        float ay_mps_sec;
        float az_mps_sec;
        float hx;
        float hy;
        float hz;
        int16_t temp_C;
        uint8_t status;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 35;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->p_rad_sec = p_rad_sec;
        _buf->q_rad_sec = q_rad_sec;
        _buf->r_rad_sec = r_rad_sec;
        _buf->ax_mps_sec = ax_mps_sec;
        _buf->ay_mps_sec = ay_mps_sec;
        _buf->az_mps_sec = az_mps_sec;
        _buf->hx = hx;
        _buf->hy = hy;
        _buf->hz = hz;
        _buf->temp_C = intround(temp_C * 10);
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        p_rad_sec = _buf->p_rad_sec;
        q_rad_sec = _buf->q_rad_sec;
        r_rad_sec = _buf->r_rad_sec;
        ax_mps_sec = _buf->ax_mps_sec;
        ay_mps_sec = _buf->ay_mps_sec;
        az_mps_sec = _buf->az_mps_sec;
        hx = _buf->hx;
        hy = _buf->hy;
        hz = _buf->hz;
        temp_C = _buf->temp_C / (float)10;
        status = _buf->status;
        return true;
    }
};

// Message: imu_v5 (id: 45)
struct imu_v5_t {
    // public fields
    uint8_t index;
    float timestamp_sec;
    float p_rad_sec;
    float q_rad_sec;
    float r_rad_sec;
    float ax_mps_sec;
    float ay_mps_sec;
    float az_mps_sec;
    float hx;
    float hy;
    float hz;
    float ax_raw;
    float ay_raw;
    float az_raw;
    float hx_raw;
    float hy_raw;
    float hz_raw;
    float temp_C;
    uint8_t status;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        float p_rad_sec;
        float q_rad_sec;
        float r_rad_sec;
        float ax_mps_sec;
        float ay_mps_sec;
        float az_mps_sec;
        float hx;
        float hy;
        float hz;
        float ax_raw;
        float ay_raw;
        float az_raw;
        float hx_raw;
        float hy_raw;
        float hz_raw;
        int16_t temp_C;
        uint8_t status;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->p_rad_sec = p_rad_sec;
        _buf->q_rad_sec = q_rad_sec;
        _buf->r_rad_sec = r_rad_sec;
        _buf->ax_mps_sec = ax_mps_sec;
        _buf->ay_mps_sec = ay_mps_sec;
        _buf->az_mps_sec = az_mps_sec;
        _buf->hx = hx;
        _buf->hy = hy;
        _buf->hz = hz;
        _buf->ax_raw = ax_raw;
        _buf->ay_raw = ay_raw;
        _buf->az_raw = az_raw;
        _buf->hx_raw = hx_raw;
        _buf->hy_raw = hy_raw;
        _buf->hz_raw = hz_raw;
        _buf->temp_C = intround(temp_C * 10);
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        p_rad_sec = _buf->p_rad_sec;
        q_rad_sec = _buf->q_rad_sec;
        r_rad_sec = _buf->r_rad_sec;
        ax_mps_sec = _buf->ax_mps_sec;
        ay_mps_sec = _buf->ay_mps_sec;
        az_mps_sec = _buf->az_mps_sec;
        hx = _buf->hx;
        hy = _buf->hy;
        hz = _buf->hz;
        ax_raw = _buf->ax_raw;
        ay_raw = _buf->ay_raw;
        az_raw = _buf->az_raw;
        hx_raw = _buf->hx_raw;
        hy_raw = _buf->hy_raw;
        hz_raw = _buf->hz_raw;
        temp_C = _buf->temp_C / (float)10;
        status = _buf->status;
        return true;
    }
};

// Message: airdata_v5 (id: 18)
struct airdata_v5_t {
    // public fields
    uint8_t index;
    double timestamp_sec;
    float pressure_mbar;
    float temp_C;
    float airspeed_smoothed_kt;
    float altitude_smoothed_m;
    float altitude_true_m;
    float pressure_vertical_speed_fps;
    float wind_dir_deg;
    float wind_speed_kt;
    float pitot_scale_factor;
    uint8_t status;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        double timestamp_sec;
        uint16_t pressure_mbar;
        int16_t temp_C;
        int16_t airspeed_smoothed_kt;
        float altitude_smoothed_m;
        float altitude_true_m;
        int16_t pressure_vertical_speed_fps;
        uint16_t wind_dir_deg;
        uint8_t wind_speed_kt;
        uint8_t pitot_scale_factor;
        uint8_t status;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 18;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->pressure_mbar = uintround(pressure_mbar * 10);
        _buf->temp_C = intround(temp_C * 100);
        _buf->airspeed_smoothed_kt = intround(airspeed_smoothed_kt * 100);
        _buf->altitude_smoothed_m = altitude_smoothed_m;
        _buf->altitude_true_m = altitude_true_m;
        _buf->pressure_vertical_speed_fps = intround(pressure_vertical_speed_fps * 600);
        _buf->wind_dir_deg = uintround(wind_dir_deg * 100);
        _buf->wind_speed_kt = uintround(wind_speed_kt * 4);
        _buf->pitot_scale_factor = uintround(pitot_scale_factor * 100);
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        pressure_mbar = _buf->pressure_mbar / (float)10;
        temp_C = _buf->temp_C / (float)100;
        airspeed_smoothed_kt = _buf->airspeed_smoothed_kt / (float)100;
        altitude_smoothed_m = _buf->altitude_smoothed_m;
        altitude_true_m = _buf->altitude_true_m;
        pressure_vertical_speed_fps = _buf->pressure_vertical_speed_fps / (float)600;
        wind_dir_deg = _buf->wind_dir_deg / (float)100;
        wind_speed_kt = _buf->wind_speed_kt / (float)4;
        pitot_scale_factor = _buf->pitot_scale_factor / (float)100;
        status = _buf->status;
        return true;
    }
};

// Message: airdata_v6 (id: 40)
struct airdata_v6_t {
    // public fields
    uint8_t index;
    float timestamp_sec;
    float pressure_mbar;
    float temp_C;
    float airspeed_smoothed_kt;
    float altitude_smoothed_m;
    float altitude_true_m;
    float pressure_vertical_speed_fps;
    float wind_dir_deg;
    float wind_speed_kt;
    float pitot_scale_factor;
    uint8_t status;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        uint16_t pressure_mbar;
        int16_t temp_C;
        int16_t airspeed_smoothed_kt;
        float altitude_smoothed_m;
        float altitude_true_m;
        int16_t pressure_vertical_speed_fps;
        uint16_t wind_dir_deg;
        uint8_t wind_speed_kt;
        uint8_t pitot_scale_factor;
        uint8_t status;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->pressure_mbar = uintround(pressure_mbar * 10);
        _buf->temp_C = intround(temp_C * 100);
        _buf->airspeed_smoothed_kt = intround(airspeed_smoothed_kt * 100);
        _buf->altitude_smoothed_m = altitude_smoothed_m;
        _buf->altitude_true_m = altitude_true_m;
        _buf->pressure_vertical_speed_fps = intround(pressure_vertical_speed_fps * 600);
        _buf->wind_dir_deg = uintround(wind_dir_deg * 100);
        _buf->wind_speed_kt = uintround(wind_speed_kt * 4);
        _buf->pitot_scale_factor = uintround(pitot_scale_factor * 100);
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        pressure_mbar = _buf->pressure_mbar / (float)10;
        temp_C = _buf->temp_C / (float)100;
        airspeed_smoothed_kt = _buf->airspeed_smoothed_kt / (float)100;
        altitude_smoothed_m = _buf->altitude_smoothed_m;
        altitude_true_m = _buf->altitude_true_m;
        pressure_vertical_speed_fps = _buf->pressure_vertical_speed_fps / (float)600;
        wind_dir_deg = _buf->wind_dir_deg / (float)100;
        wind_speed_kt = _buf->wind_speed_kt / (float)4;
        pitot_scale_factor = _buf->pitot_scale_factor / (float)100;
        status = _buf->status;
        return true;
    }
};

// Message: airdata_v7 (id: 43)
struct airdata_v7_t {
    // public fields
    uint8_t index;
    float timestamp_sec;
    float pressure_mbar;
    float temp_C;
    float airspeed_smoothed_kt;
    float altitude_smoothed_m;
    float altitude_true_m;
    float pressure_vertical_speed_fps;
    float wind_dir_deg;
    float wind_speed_kt;
    float pitot_scale_factor;
    uint16_t error_count;
    uint8_t status;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        uint16_t pressure_mbar;
        int16_t temp_C;
        int16_t airspeed_smoothed_kt;
        float altitude_smoothed_m;
        float altitude_true_m;
        int16_t pressure_vertical_speed_fps;
        uint16_t wind_dir_deg;
        uint8_t wind_speed_kt;
        uint8_t pitot_scale_factor;
        uint16_t error_count;
        uint8_t status;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->pressure_mbar = uintround(pressure_mbar * 10);
        _buf->temp_C = intround(temp_C * 100);
        _buf->airspeed_smoothed_kt = intround(airspeed_smoothed_kt * 100);
        _buf->altitude_smoothed_m = altitude_smoothed_m;
        _buf->altitude_true_m = altitude_true_m;
        _buf->pressure_vertical_speed_fps = intround(pressure_vertical_speed_fps * 600);
        _buf->wind_dir_deg = uintround(wind_dir_deg * 100);
        _buf->wind_speed_kt = uintround(wind_speed_kt * 4);
        _buf->pitot_scale_factor = uintround(pitot_scale_factor * 100);
        _buf->error_count = error_count;
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        pressure_mbar = _buf->pressure_mbar / (float)10;
        temp_C = _buf->temp_C / (float)100;
        airspeed_smoothed_kt = _buf->airspeed_smoothed_kt / (float)100;
        altitude_smoothed_m = _buf->altitude_smoothed_m;
        altitude_true_m = _buf->altitude_true_m;
        pressure_vertical_speed_fps = _buf->pressure_vertical_speed_fps / (float)600;
        wind_dir_deg = _buf->wind_dir_deg / (float)100;
        wind_speed_kt = _buf->wind_speed_kt / (float)4;
        pitot_scale_factor = _buf->pitot_scale_factor / (float)100;
        error_count = _buf->error_count;
        status = _buf->status;
        return true;
    }
};

// Message: filter_v3 (id: 31)
struct filter_v3_t {
    // public fields
    uint8_t index;
    double timestamp_sec;
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float vn_ms;
    float ve_ms;
    float vd_ms;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float p_bias;
    float q_bias;
    float r_bias;
    float ax_bias;
    float ay_bias;
    float az_bias;
    uint8_t sequence_num;
    uint8_t status;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        double timestamp_sec;
        double latitude_deg;
        double longitude_deg;
        float altitude_m;
        int16_t vn_ms;
        int16_t ve_ms;
        int16_t vd_ms;
        int16_t roll_deg;
        int16_t pitch_deg;
        int16_t yaw_deg;
        int16_t p_bias;
        int16_t q_bias;
        int16_t r_bias;
        int16_t ax_bias;
        int16_t ay_bias;
        int16_t az_bias;
        uint8_t sequence_num;
        uint8_t status;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 31;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->latitude_deg = latitude_deg;
        _buf->longitude_deg = longitude_deg;
        _buf->altitude_m = altitude_m;
        _buf->vn_ms = intround(vn_ms * 100);
        _buf->ve_ms = intround(ve_ms * 100);
        _buf->vd_ms = intround(vd_ms * 100);
        _buf->roll_deg = intround(roll_deg * 10);
        _buf->pitch_deg = intround(pitch_deg * 10);
        _buf->yaw_deg = intround(yaw_deg * 10);
        _buf->p_bias = intround(p_bias * 10000);
        _buf->q_bias = intround(q_bias * 10000);
        _buf->r_bias = intround(r_bias * 10000);
        _buf->ax_bias = intround(ax_bias * 1000);
        _buf->ay_bias = intround(ay_bias * 1000);
        _buf->az_bias = intround(az_bias * 1000);
        _buf->sequence_num = sequence_num;
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        latitude_deg = _buf->latitude_deg;
        longitude_deg = _buf->longitude_deg;
        altitude_m = _buf->altitude_m;
        vn_ms = _buf->vn_ms / (float)100;
        ve_ms = _buf->ve_ms / (float)100;
        vd_ms = _buf->vd_ms / (float)100;
        roll_deg = _buf->roll_deg / (float)10;
        pitch_deg = _buf->pitch_deg / (float)10;
        yaw_deg = _buf->yaw_deg / (float)10;
        p_bias = _buf->p_bias / (float)10000;
        q_bias = _buf->q_bias / (float)10000;
        r_bias = _buf->r_bias / (float)10000;
        ax_bias = _buf->ax_bias / (float)1000;
        ay_bias = _buf->ay_bias / (float)1000;
        az_bias = _buf->az_bias / (float)1000;
        sequence_num = _buf->sequence_num;
        status = _buf->status;
        return true;
    }
};

// Message: filter_v4 (id: 36)
struct filter_v4_t {
    // public fields
    uint8_t index;
    float timestamp_sec;
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float vn_ms;
    float ve_ms;
    float vd_ms;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float p_bias;
    float q_bias;
    float r_bias;
    float ax_bias;
    float ay_bias;
    float az_bias;
    uint8_t sequence_num;
    uint8_t status;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        double latitude_deg;
        double longitude_deg;
        float altitude_m;
        int16_t vn_ms;
        int16_t ve_ms;
        int16_t vd_ms;
        int16_t roll_deg;
        int16_t pitch_deg;
        int16_t yaw_deg;
        int16_t p_bias;
        int16_t q_bias;
        int16_t r_bias;
        int16_t ax_bias;
        int16_t ay_bias;
        int16_t az_bias;
        uint8_t sequence_num;
        uint8_t status;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 36;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->latitude_deg = latitude_deg;
        _buf->longitude_deg = longitude_deg;
        _buf->altitude_m = altitude_m;
        _buf->vn_ms = intround(vn_ms * 100);
        _buf->ve_ms = intround(ve_ms * 100);
        _buf->vd_ms = intround(vd_ms * 100);
        _buf->roll_deg = intround(roll_deg * 10);
        _buf->pitch_deg = intround(pitch_deg * 10);
        _buf->yaw_deg = intround(yaw_deg * 10);
        _buf->p_bias = intround(p_bias * 10000);
        _buf->q_bias = intround(q_bias * 10000);
        _buf->r_bias = intround(r_bias * 10000);
        _buf->ax_bias = intround(ax_bias * 1000);
        _buf->ay_bias = intround(ay_bias * 1000);
        _buf->az_bias = intround(az_bias * 1000);
        _buf->sequence_num = sequence_num;
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        latitude_deg = _buf->latitude_deg;
        longitude_deg = _buf->longitude_deg;
        altitude_m = _buf->altitude_m;
        vn_ms = _buf->vn_ms / (float)100;
        ve_ms = _buf->ve_ms / (float)100;
        vd_ms = _buf->vd_ms / (float)100;
        roll_deg = _buf->roll_deg / (float)10;
        pitch_deg = _buf->pitch_deg / (float)10;
        yaw_deg = _buf->yaw_deg / (float)10;
        p_bias = _buf->p_bias / (float)10000;
        q_bias = _buf->q_bias / (float)10000;
        r_bias = _buf->r_bias / (float)10000;
        ax_bias = _buf->ax_bias / (float)1000;
        ay_bias = _buf->ay_bias / (float)1000;
        az_bias = _buf->az_bias / (float)1000;
        sequence_num = _buf->sequence_num;
        status = _buf->status;
        return true;
    }
};

// Message: filter_v5 (id: 47)
struct filter_v5_t {
    // public fields
    uint8_t index;
    float timestamp_sec;
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float vn_ms;
    float ve_ms;
    float vd_ms;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float p_bias;
    float q_bias;
    float r_bias;
    float ax_bias;
    float ay_bias;
    float az_bias;
    float max_pos_cov;
    float max_vel_cov;
    float max_att_cov;
    uint8_t sequence_num;
    uint8_t status;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        double latitude_deg;
        double longitude_deg;
        float altitude_m;
        int16_t vn_ms;
        int16_t ve_ms;
        int16_t vd_ms;
        int16_t roll_deg;
        int16_t pitch_deg;
        int16_t yaw_deg;
        int16_t p_bias;
        int16_t q_bias;
        int16_t r_bias;
        int16_t ax_bias;
        int16_t ay_bias;
        int16_t az_bias;
        uint16_t max_pos_cov;
        uint16_t max_vel_cov;
        uint16_t max_att_cov;
        uint8_t sequence_num;
        uint8_t status;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->latitude_deg = latitude_deg;
        _buf->longitude_deg = longitude_deg;
        _buf->altitude_m = altitude_m;
        _buf->vn_ms = intround(vn_ms * 100);
        _buf->ve_ms = intround(ve_ms * 100);
        _buf->vd_ms = intround(vd_ms * 100);
        _buf->roll_deg = intround(roll_deg * 10);
        _buf->pitch_deg = intround(pitch_deg * 10);
        _buf->yaw_deg = intround(yaw_deg * 10);
        _buf->p_bias = intround(p_bias * 10000);
        _buf->q_bias = intround(q_bias * 10000);
        _buf->r_bias = intround(r_bias * 10000);
        _buf->ax_bias = intround(ax_bias * 1000);
        _buf->ay_bias = intround(ay_bias * 1000);
        _buf->az_bias = intround(az_bias * 1000);
        _buf->max_pos_cov = uintround(max_pos_cov * 100);
        _buf->max_vel_cov = uintround(max_vel_cov * 1000);
        _buf->max_att_cov = uintround(max_att_cov * 10000);
        _buf->sequence_num = sequence_num;
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        latitude_deg = _buf->latitude_deg;
        longitude_deg = _buf->longitude_deg;
        altitude_m = _buf->altitude_m;
        vn_ms = _buf->vn_ms / (float)100;
        ve_ms = _buf->ve_ms / (float)100;
        vd_ms = _buf->vd_ms / (float)100;
        roll_deg = _buf->roll_deg / (float)10;
        pitch_deg = _buf->pitch_deg / (float)10;
        yaw_deg = _buf->yaw_deg / (float)10;
        p_bias = _buf->p_bias / (float)10000;
        q_bias = _buf->q_bias / (float)10000;
        r_bias = _buf->r_bias / (float)10000;
        ax_bias = _buf->ax_bias / (float)1000;
        ay_bias = _buf->ay_bias / (float)1000;
        az_bias = _buf->az_bias / (float)1000;
        max_pos_cov = _buf->max_pos_cov / (float)100;
        max_vel_cov = _buf->max_vel_cov / (float)1000;
        max_att_cov = _buf->max_att_cov / (float)10000;
        sequence_num = _buf->sequence_num;
        status = _buf->status;
        return true;
    }
};

// Message: actuator_v2 (id: 21)
struct actuator_v2_t {
    // public fields
    uint8_t index;
    double timestamp_sec;
    float aileron;
    float elevator;
    float throttle;
    float rudder;
    float channel5;
    float flaps;
    float channel7;
    float channel8;
    uint8_t status;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        double timestamp_sec;
        int16_t aileron;
        int16_t elevator;
        uint16_t throttle;
        int16_t rudder;
        int16_t channel5;
        int16_t flaps;
        int16_t channel7;
        int16_t channel8;
        uint8_t status;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 21;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->aileron = intround(aileron * 20000);
        _buf->elevator = intround(elevator * 20000);
        _buf->throttle = uintround(throttle * 60000);
        _buf->rudder = intround(rudder * 20000);
        _buf->channel5 = intround(channel5 * 20000);
        _buf->flaps = intround(flaps * 20000);
        _buf->channel7 = intround(channel7 * 20000);
        _buf->channel8 = intround(channel8 * 20000);
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        aileron = _buf->aileron / (float)20000;
        elevator = _buf->elevator / (float)20000;
        throttle = _buf->throttle / (float)60000;
        rudder = _buf->rudder / (float)20000;
        channel5 = _buf->channel5 / (float)20000;
        flaps = _buf->flaps / (float)20000;
        channel7 = _buf->channel7 / (float)20000;
        channel8 = _buf->channel8 / (float)20000;
        status = _buf->status;
        return true;
    }
};

// Message: actuator_v3 (id: 37)
struct actuator_v3_t {
    // public fields
    uint8_t index;
    float timestamp_sec;
    float aileron;
    float elevator;
    float throttle;
    float rudder;
    float channel5;
    float flaps;
    float channel7;
    float channel8;
    uint8_t status;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        int16_t aileron;
        int16_t elevator;
        uint16_t throttle;
        int16_t rudder;
        int16_t channel5;
        int16_t flaps;
        int16_t channel7;
        int16_t channel8;
        uint8_t status;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 37;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->aileron = intround(aileron * 20000);
        _buf->elevator = intround(elevator * 20000);
        _buf->throttle = uintround(throttle * 60000);
        _buf->rudder = intround(rudder * 20000);
        _buf->channel5 = intround(channel5 * 20000);
        _buf->flaps = intround(flaps * 20000);
        _buf->channel7 = intround(channel7 * 20000);
        _buf->channel8 = intround(channel8 * 20000);
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        aileron = _buf->aileron / (float)20000;
        elevator = _buf->elevator / (float)20000;
        throttle = _buf->throttle / (float)60000;
        rudder = _buf->rudder / (float)20000;
        channel5 = _buf->channel5 / (float)20000;
        flaps = _buf->flaps / (float)20000;
        channel7 = _buf->channel7 / (float)20000;
        channel8 = _buf->channel8 / (float)20000;
        status = _buf->status;
        return true;
    }
};

// Message: pilot_v2 (id: 20)
struct pilot_v2_t {
    // public fields
    uint8_t index;
    double timestamp_sec;
    float channel[8];
    uint8_t status;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        double timestamp_sec;
        int16_t channel[8];
        uint8_t status;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        for (int _i=0; _i<8; _i++) _buf->channel[_i] = intround(channel[_i] * 20000);
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        for (int _i=0; _i<8; _i++) channel[_i] = _buf->channel[_i] / (float)20000;
        status = _buf->status;
        return true;
    }
};

// Message: pilot_v3 (id: 38)
struct pilot_v3_t {
    // public fields
    uint8_t index;
    float timestamp_sec;
    float channel[8];
    uint8_t status;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        int16_t channel[8];
        uint8_t status;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 38;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        for (int _i=0; _i<8; _i++) _buf->channel[_i] = intround(channel[_i] * 20000);
        _buf->status = status;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        for (int _i=0; _i<8; _i++) channel[_i] = _buf->channel[_i] / (float)20000;
        status = _buf->status;
        return true;
    }
};

// Message: ap_status_v4 (id: 30)
struct ap_status_v4_t {
    // public fields
    uint8_t index;
    double timestamp_sec;
    float groundtrack_deg;
    float roll_deg;
    uint16_t altitude_msl_ft;
    uint16_t altitude_ground_m;
    float pitch_deg;
    float airspeed_kt;
    uint16_t flight_timer;
    uint16_t target_waypoint_idx;
    double wp_longitude_deg;
    double wp_latitude_deg;
    uint16_t wp_index;
    uint16_t route_size;
    uint8_t sequence_num;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        double timestamp_sec;
        int16_t groundtrack_deg;
        int16_t roll_deg;
        uint16_t altitude_msl_ft;
        uint16_t altitude_ground_m;
        int16_t pitch_deg;
        int16_t airspeed_kt;
        uint16_t flight_timer;
        uint16_t target_waypoint_idx;
        double wp_longitude_deg;
        double wp_latitude_deg;
        uint16_t wp_index;
        uint16_t route_size;
        uint8_t sequence_num;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 30;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->groundtrack_deg = intround(groundtrack_deg * 10);
        _buf->roll_deg = intround(roll_deg * 10);
        _buf->altitude_msl_ft = altitude_msl_ft;
        _buf->altitude_ground_m = altitude_ground_m;
        _buf->pitch_deg = intround(pitch_deg * 10);
        _buf->airspeed_kt = intround(airspeed_kt * 10);
        _buf->flight_timer = flight_timer;
        _buf->target_waypoint_idx = target_waypoint_idx;
        _buf->wp_longitude_deg = wp_longitude_deg;
        _buf->wp_latitude_deg = wp_latitude_deg;
        _buf->wp_index = wp_index;
        _buf->route_size = route_size;
        _buf->sequence_num = sequence_num;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        groundtrack_deg = _buf->groundtrack_deg / (float)10;
        roll_deg = _buf->roll_deg / (float)10;
        altitude_msl_ft = _buf->altitude_msl_ft;
        altitude_ground_m = _buf->altitude_ground_m;
        pitch_deg = _buf->pitch_deg / (float)10;
        airspeed_kt = _buf->airspeed_kt / (float)10;
        flight_timer = _buf->flight_timer;
        target_waypoint_idx = _buf->target_waypoint_idx;
        wp_longitude_deg = _buf->wp_longitude_deg;
        wp_latitude_deg = _buf->wp_latitude_deg;
        wp_index = _buf->wp_index;
        route_size = _buf->route_size;
        sequence_num = _buf->sequence_num;
        return true;
    }
};

// Message: ap_status_v5 (id: 32)
struct ap_status_v5_t {
    // public fields
    uint8_t index;
    double timestamp_sec;
    uint8_t flags;
    float groundtrack_deg;
    float roll_deg;
    uint16_t altitude_msl_ft;
    uint16_t altitude_ground_m;
    float pitch_deg;
    float airspeed_kt;
    uint16_t flight_timer;
    uint16_t target_waypoint_idx;
    double wp_longitude_deg;
    double wp_latitude_deg;
    uint16_t wp_index;
    uint16_t route_size;
    uint8_t sequence_num;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        double timestamp_sec;
        uint8_t flags;
        int16_t groundtrack_deg;
        int16_t roll_deg;
        uint16_t altitude_msl_ft;
        uint16_t altitude_ground_m;
        int16_t pitch_deg;
        int16_t airspeed_kt;
        uint16_t flight_timer;
        uint16_t target_waypoint_idx;
        double wp_longitude_deg;
        double wp_latitude_deg;
        uint16_t wp_index;
        uint16_t route_size;
        uint8_t sequence_num;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 32;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->flags = flags;
        _buf->groundtrack_deg = intround(groundtrack_deg * 10);
        _buf->roll_deg = intround(roll_deg * 10);
        _buf->altitude_msl_ft = altitude_msl_ft;
        _buf->altitude_ground_m = altitude_ground_m;
        _buf->pitch_deg = intround(pitch_deg * 10);
        _buf->airspeed_kt = intround(airspeed_kt * 10);
        _buf->flight_timer = flight_timer;
        _buf->target_waypoint_idx = target_waypoint_idx;
        _buf->wp_longitude_deg = wp_longitude_deg;
        _buf->wp_latitude_deg = wp_latitude_deg;
        _buf->wp_index = wp_index;
        _buf->route_size = route_size;
        _buf->sequence_num = sequence_num;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        flags = _buf->flags;
        groundtrack_deg = _buf->groundtrack_deg / (float)10;
        roll_deg = _buf->roll_deg / (float)10;
        altitude_msl_ft = _buf->altitude_msl_ft;
        altitude_ground_m = _buf->altitude_ground_m;
        pitch_deg = _buf->pitch_deg / (float)10;
        airspeed_kt = _buf->airspeed_kt / (float)10;
        flight_timer = _buf->flight_timer;
        target_waypoint_idx = _buf->target_waypoint_idx;
        wp_longitude_deg = _buf->wp_longitude_deg;
        wp_latitude_deg = _buf->wp_latitude_deg;
        wp_index = _buf->wp_index;
        route_size = _buf->route_size;
        sequence_num = _buf->sequence_num;
        return true;
    }
};

// Message: ap_status_v6 (id: 33)
struct ap_status_v6_t {
    // public fields
    uint8_t index;
    double timestamp_sec;
    uint8_t flags;
    float groundtrack_deg;
    float roll_deg;
    uint16_t altitude_msl_ft;
    uint16_t altitude_ground_m;
    float pitch_deg;
    float airspeed_kt;
    uint16_t flight_timer;
    uint16_t target_waypoint_idx;
    double wp_longitude_deg;
    double wp_latitude_deg;
    uint16_t wp_index;
    uint16_t route_size;
    uint8_t task_id;
    uint16_t task_attribute;
    uint8_t sequence_num;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        double timestamp_sec;
        uint8_t flags;
        int16_t groundtrack_deg;
        int16_t roll_deg;
        uint16_t altitude_msl_ft;
        uint16_t altitude_ground_m;
        int16_t pitch_deg;
        int16_t airspeed_kt;
        uint16_t flight_timer;
        uint16_t target_waypoint_idx;
        double wp_longitude_deg;
        double wp_latitude_deg;
        uint16_t wp_index;
        uint16_t route_size;
        uint8_t task_id;
        uint16_t task_attribute;
        uint8_t sequence_num;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 33;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->flags = flags;
        _buf->groundtrack_deg = intround(groundtrack_deg * 10);
        _buf->roll_deg = intround(roll_deg * 10);
        _buf->altitude_msl_ft = altitude_msl_ft;
        _buf->altitude_ground_m = altitude_ground_m;
        _buf->pitch_deg = intround(pitch_deg * 10);
        _buf->airspeed_kt = intround(airspeed_kt * 10);
        _buf->flight_timer = flight_timer;
        _buf->target_waypoint_idx = target_waypoint_idx;
        _buf->wp_longitude_deg = wp_longitude_deg;
        _buf->wp_latitude_deg = wp_latitude_deg;
        _buf->wp_index = wp_index;
        _buf->route_size = route_size;
        _buf->task_id = task_id;
        _buf->task_attribute = task_attribute;
        _buf->sequence_num = sequence_num;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        flags = _buf->flags;
        groundtrack_deg = _buf->groundtrack_deg / (float)10;
        roll_deg = _buf->roll_deg / (float)10;
        altitude_msl_ft = _buf->altitude_msl_ft;
        altitude_ground_m = _buf->altitude_ground_m;
        pitch_deg = _buf->pitch_deg / (float)10;
        airspeed_kt = _buf->airspeed_kt / (float)10;
        flight_timer = _buf->flight_timer;
        target_waypoint_idx = _buf->target_waypoint_idx;
        wp_longitude_deg = _buf->wp_longitude_deg;
        wp_latitude_deg = _buf->wp_latitude_deg;
        wp_index = _buf->wp_index;
        route_size = _buf->route_size;
        task_id = _buf->task_id;
        task_attribute = _buf->task_attribute;
        sequence_num = _buf->sequence_num;
        return true;
    }
};

// Message: ap_status_v7 (id: 39)
struct ap_status_v7_t {
    // public fields
    uint8_t index;
    float timestamp_sec;
    uint8_t flags;
    float groundtrack_deg;
    float roll_deg;
    float altitude_msl_ft;
    float altitude_ground_m;
    float pitch_deg;
    float airspeed_kt;
    float flight_timer;
    uint16_t target_waypoint_idx;
    double wp_longitude_deg;
    double wp_latitude_deg;
    uint16_t wp_index;
    uint16_t route_size;
    uint8_t task_id;
    uint16_t task_attribute;
    uint8_t sequence_num;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        uint8_t flags;
        int16_t groundtrack_deg;
        int16_t roll_deg;
        uint16_t altitude_msl_ft;
        uint16_t altitude_ground_m;
        int16_t pitch_deg;
        int16_t airspeed_kt;
        uint16_t flight_timer;
        uint16_t target_waypoint_idx;
        double wp_longitude_deg;
        double wp_latitude_deg;
        uint16_t wp_index;
        uint16_t route_size;
        uint8_t task_id;
        uint16_t task_attribute;
        uint8_t sequence_num;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 39;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->flags = flags;
        _buf->groundtrack_deg = intround(groundtrack_deg * 10);
        _buf->roll_deg = intround(roll_deg * 10);
        _buf->altitude_msl_ft = uintround(altitude_msl_ft * 1);
        _buf->altitude_ground_m = uintround(altitude_ground_m * 1);
        _buf->pitch_deg = intround(pitch_deg * 10);
        _buf->airspeed_kt = intround(airspeed_kt * 10);
        _buf->flight_timer = uintround(flight_timer * 1);
        _buf->target_waypoint_idx = target_waypoint_idx;
        _buf->wp_longitude_deg = wp_longitude_deg;
        _buf->wp_latitude_deg = wp_latitude_deg;
        _buf->wp_index = wp_index;
        _buf->route_size = route_size;
        _buf->task_id = task_id;
        _buf->task_attribute = task_attribute;
        _buf->sequence_num = sequence_num;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        flags = _buf->flags;
        groundtrack_deg = _buf->groundtrack_deg / (float)10;
        roll_deg = _buf->roll_deg / (float)10;
        altitude_msl_ft = _buf->altitude_msl_ft / (float)1;
        altitude_ground_m = _buf->altitude_ground_m / (float)1;
        pitch_deg = _buf->pitch_deg / (float)10;
        airspeed_kt = _buf->airspeed_kt / (float)10;
        flight_timer = _buf->flight_timer / (float)1;
        target_waypoint_idx = _buf->target_waypoint_idx;
        wp_longitude_deg = _buf->wp_longitude_deg;
        wp_latitude_deg = _buf->wp_latitude_deg;
        wp_index = _buf->wp_index;
        route_size = _buf->route_size;
        task_id = _buf->task_id;
        task_attribute = _buf->task_attribute;
        sequence_num = _buf->sequence_num;
        return true;
    }
};

// Message: system_health_v4 (id: 19)
struct system_health_v4_t {
    // public fields
    uint8_t index;
    double timestamp_sec;
    float system_load_avg;
    float avionics_vcc;
    float main_vcc;
    float cell_vcc;
    float main_amps;
    float total_mah;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        double timestamp_sec;
        uint16_t system_load_avg;
        uint16_t avionics_vcc;
        uint16_t main_vcc;
        uint16_t cell_vcc;
        uint16_t main_amps;
        uint16_t total_mah;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 19;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->system_load_avg = uintround(system_load_avg * 100);
        _buf->avionics_vcc = uintround(avionics_vcc * 1000);
        _buf->main_vcc = uintround(main_vcc * 1000);
        _buf->cell_vcc = uintround(cell_vcc * 1000);
        _buf->main_amps = uintround(main_amps * 1000);
        _buf->total_mah = uintround(total_mah * 10);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        system_load_avg = _buf->system_load_avg / (float)100;
        avionics_vcc = _buf->avionics_vcc / (float)1000;
        main_vcc = _buf->main_vcc / (float)1000;
        cell_vcc = _buf->cell_vcc / (float)1000;
        main_amps = _buf->main_amps / (float)1000;
        total_mah = _buf->total_mah / (float)10;
        return true;
    }
};

// Message: system_health_v5 (id: 41)
struct system_health_v5_t {
    // public fields
    uint8_t index;
    float timestamp_sec;
    float system_load_avg;
    float avionics_vcc;
    float main_vcc;
    float cell_vcc;
    float main_amps;
    float total_mah;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        uint16_t system_load_avg;
        uint16_t avionics_vcc;
        uint16_t main_vcc;
        uint16_t cell_vcc;
        uint16_t main_amps;
        uint16_t total_mah;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->system_load_avg = uintround(system_load_avg * 100);
        _buf->avionics_vcc = uintround(avionics_vcc * 1000);
        _buf->main_vcc = uintround(main_vcc * 1000);
        _buf->cell_vcc = uintround(cell_vcc * 1000);
        _buf->main_amps = uintround(main_amps * 1000);
        _buf->total_mah = uintround(total_mah * 0.1);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        system_load_avg = _buf->system_load_avg / (float)100;
        avionics_vcc = _buf->avionics_vcc / (float)1000;
        main_vcc = _buf->main_vcc / (float)1000;
        cell_vcc = _buf->cell_vcc / (float)1000;
        main_amps = _buf->main_amps / (float)1000;
        total_mah = _buf->total_mah / (float)0.1;
        return true;
    }
};

// Message: system_health_v6 (id: 46)
struct system_health_v6_t {
    // public fields
    uint8_t index;
    float timestamp_sec;
    float system_load_avg;
    uint16_t fmu_timer_misses;
    float avionics_vcc;
    float main_vcc;
    float cell_vcc;
    float main_amps;
    float total_mah;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        uint16_t system_load_avg;
        uint16_t fmu_timer_misses;
        uint16_t avionics_vcc;
        uint16_t main_vcc;
        uint16_t cell_vcc;
        uint16_t main_amps;
        uint16_t total_mah;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->system_load_avg = uintround(system_load_avg * 100);
        _buf->fmu_timer_misses = fmu_timer_misses;
        _buf->avionics_vcc = uintround(avionics_vcc * 1000);
        _buf->main_vcc = uintround(main_vcc * 1000);
        _buf->cell_vcc = uintround(cell_vcc * 1000);
        _buf->main_amps = uintround(main_amps * 1000);
        _buf->total_mah = uintround(total_mah * 0.1);
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        system_load_avg = _buf->system_load_avg / (float)100;
        fmu_timer_misses = _buf->fmu_timer_misses;
        avionics_vcc = _buf->avionics_vcc / (float)1000;
        main_vcc = _buf->main_vcc / (float)1000;
        cell_vcc = _buf->cell_vcc / (float)1000;
        main_amps = _buf->main_amps / (float)1000;
        total_mah = _buf->total_mah / (float)0.1;
        return true;
    }
};

// Message: payload_v2 (id: 23)
struct payload_v2_t {
    // public fields
    uint8_t index;
    double timestamp_sec;
    uint16_t trigger_num;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        double timestamp_sec;
        uint16_t trigger_num;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 23;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->trigger_num = trigger_num;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        trigger_num = _buf->trigger_num;
        return true;
    }
};

// Message: payload_v3 (id: 42)
struct payload_v3_t {
    // public fields
    uint8_t index;
    float timestamp_sec;
    uint16_t trigger_num;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        float timestamp_sec;
        uint16_t trigger_num;
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
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->trigger_num = trigger_num;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        trigger_num = _buf->trigger_num;
        return true;
    }
};

// Message: event_v1 (id: 27)
struct event_v1_t {
    // public fields
    uint8_t index;
    double timestamp_sec;
    string message;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t index;
        double timestamp_sec;
        uint8_t message_len;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 27;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        size += message.length();
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->index = index;
        _buf->timestamp_sec = timestamp_sec;
        _buf->message_len = message.length();
        memcpy(&(payload[len]), message.c_str(), message.length());
        len += message.length();
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        index = _buf->index;
        timestamp_sec = _buf->timestamp_sec;
        message = string((char *)&(payload[len]), _buf->message_len);
        len += _buf->message_len;
        return true;
    }
};

// Message: event_v2 (id: 44)
struct event_v2_t {
    // public fields
    float timestamp_sec;
    uint8_t sequence_num;
    string message;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        float timestamp_sec;
        uint8_t sequence_num;
        uint8_t message_len;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 44;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        size += message.length();
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->timestamp_sec = timestamp_sec;
        _buf->sequence_num = sequence_num;
        _buf->message_len = message.length();
        memcpy(&(payload[len]), message.c_str(), message.length());
        len += message.length();
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        timestamp_sec = _buf->timestamp_sec;
        sequence_num = _buf->sequence_num;
        message = string((char *)&(payload[len]), _buf->message_len);
        len += _buf->message_len;
        return true;
    }
};

// Message: command_v1 (id: 28)
struct command_v1_t {
    // public fields
    uint8_t sequence_num;
    string message;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        uint8_t sequence_num;
        uint8_t message_len;
    };
    #pragma pack(pop)

    // public info fields
    static const uint8_t id = 28;
    int len = 0;

    bool pack() {
        len = sizeof(_compact_t);
        // size sanity check
        int size = len;
        size += message.length();
        if ( size > message_max_len ) {
            return false;
        }
        // copy values
        _compact_t *_buf = (_compact_t *)payload;
        _buf->sequence_num = sequence_num;
        _buf->message_len = message.length();
        memcpy(&(payload[len]), message.c_str(), message.length());
        len += message.length();
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        sequence_num = _buf->sequence_num;
        message = string((char *)&(payload[len]), _buf->message_len);
        len += _buf->message_len;
        return true;
    }
};

// Message: stream_v1 (id: 49)
struct stream_v1_t {
    // public fields
    float sigma1;
    float sigma2;
    float sigma3;

    // internal structure for packing
    uint8_t payload[message_max_len];
    #pragma pack(push, 1)
    struct _compact_t {
        float sigma1;
        float sigma2;
        float sigma3;
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
        _buf->sigma1 = sigma1;
        _buf->sigma2 = sigma2;
        _buf->sigma3 = sigma3;
        return true;
    }

    bool unpack(uint8_t *external_message, int message_size) {
        if ( message_size > message_max_len ) {
            return false;
        }
        memcpy(payload, external_message, message_size);
        _compact_t *_buf = (_compact_t *)payload;
        len = sizeof(_compact_t);
        sigma1 = _buf->sigma1;
        sigma2 = _buf->sigma2;
        sigma3 = _buf->sigma3;
        return true;
    }
};

} // namespace message
