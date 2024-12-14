#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/OwnPtr.h>

class AP_OpticalFlow_Pixart
{
public:
    /// constructor
    AP_OpticalFlow_Pixart(const char *devname);

    // init - initialise the sensor
    void init();

    // update - read latest values from sensor and fill in x,y and totals.
    void update(void);

    // detect if the sensor is available
    static AP_OpticalFlow_Pixart *detect(const char *devname);

    struct OpticalFlow_state {
        uint8_t  surface_quality;   // image quality (below TBD you can't trust the dx,dy values returned)
        Vector2f flowRate;          // optical flow angular rate in rad/sec measured about the X and Y body axis. A RH rotation about a sensor axis produces a positive rate.
    } state;

private:
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;
    
    struct RegData {
        uint8_t reg;
        uint8_t value;
    };

    struct PACKED MotionBurst {
        uint8_t motion;
        uint8_t observation;
        uint8_t delta_x_l;
        uint8_t delta_x_h;
        uint8_t delta_y_l;
        uint8_t delta_y_h;
        uint8_t reserved;
        uint8_t squal;
        uint8_t rawdata_sum;
        uint8_t max_raw;
        uint8_t min_raw;
        uint8_t shutter_upper;
        uint8_t shutter_middle;
        uint8_t shutter_lower;
    } burst;

    struct {
        Vector2l sum;
        // uint32_t last_frame_us;
        uint32_t sum_us;
        uint32_t qual_sum;
        uint32_t count;
    } integral;

    enum Mode : uint8_t {
        Bright        = 0,
        LowLight      = 1,
        SuperLowLight = 2,
        unknown       = 3,
    } mode;
    
    const uint8_t res = 0xFF;
    const float flow_pixel_scaling = 39.3701f * 11.914f * ((float)(res + 1) / 43.0f);

    bool reset(void);
    // setup sensor
    bool setup_sensor(void);
    
    void reg_write(uint8_t reg, uint8_t value);
    uint8_t reg_read(uint8_t reg);
    int16_t reg_read16s(uint8_t reg);
    uint16_t reg_read16u(uint8_t reg);

    void ConfigureStandardDetectionSetting();
    void ConfigureEnhancedDetectionMode();
    void ConfigureAutomaticModeSwitching();
    void EnableLed();
    void Configure();

    void timer_int(uint8_t, bool, uint32_t);
    void timer(void);
    void timer_thread(void);
    bool motion_burst(uint32_t *dt);
    void send_zeros(void);
    uint32_t dt_us();

    uint32_t last_burst_us;
    uint32_t last_update_ms;

    uint32_t last_spi_op_us;
    enum {
        rd = 0,
        wr = 1,
    } last_spi_op;

    bool challenging;

    HAL_Semaphore _sem;
};

