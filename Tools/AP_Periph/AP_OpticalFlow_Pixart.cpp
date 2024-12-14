/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  NOTE: This sensor does not use traditional SPI register access. The
  timing for register reads and writes is critical
 */



#include "AP_OpticalFlow_Pixart.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Math/crc.h>
#include <utility>
#include <stdio.h>
#include "AP_Periph.h"

#define debug(fmt, args ...)  //can_printf(fmt, ## args)

extern const AP_HAL::HAL& hal;

#define PIXART_REG_PRODUCT_ID  0x00
#define PIXART_REG_REVISION_ID 0x01
#define PIXART_REG_MOTION      0x02
#define PIXART_REG_DELTA_X_L   0x03
#define PIXART_REG_DELTA_X_H   0x04
#define PIXART_REG_DELTA_Y_L   0x05
#define PIXART_REG_DELTA_Y_H   0x06
#define PIXART_REG_SQUAL       0x07
#define PIXART_REG_RAWDATA_SUM 0x08
#define PIXART_REG_RAWDATA_MAX 0x09
#define PIXART_REG_RAWDATA_MIN 0x0A
#define PIXART_REG_SHUTTER_LOW 0x0B
#define PIXART_REG_SHUTTER_HI  0x0C

#define PIXART_REG_POWER_RST   0x3A
#define PIXART_REG_SHUTDOWN    0x3B
#define PIXART_REG_INV_PROD_ID 0x5F
#define PIXART_REG_MOT_BURST   0x16


// writing to registers needs this flag
#define PIXART_WRITE_FLAG      0x80
#define PIXART_READ_FLAG       0x7F

// timings in microseconds
#define PIXART_Tsww             11
#define PIXART_Tswr             6
#define PIXART_Tsrw             2
#define PIXART_Tsrr             2
#define PIXART_Tsrad            2
#define PIXART_Tbexit           1

static constexpr uint16_t SAMPLE_INTERVAL_MODE_0{1000000 / 126}; // 126 fps
static constexpr uint16_t SAMPLE_INTERVAL_MODE_1{1000000 / 126}; // 126 fps
static constexpr uint16_t SAMPLE_INTERVAL_MODE_2{1000000 / 50};  // 50 fps

// constructor
AP_OpticalFlow_Pixart::AP_OpticalFlow_Pixart(const char *devname)
{
    _dev = std::move(hal.spi->get_device(devname));
}

// detect the device
AP_OpticalFlow_Pixart *AP_OpticalFlow_Pixart::detect(const char *devname)
{
    AP_OpticalFlow_Pixart *sensor = new AP_OpticalFlow_Pixart(devname);
    if (!sensor) {
        return nullptr;
    }
    if (!sensor->setup_sensor()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

void AP_OpticalFlow_Pixart::ConfigureStandardDetectionSetting()
{
	// Standard Detection Setting is recommended for general tracking operations. In this mode, the chip can detect
	// when it is operating over striped, checkerboard, and glossy tile surfaces where tracking performance is
	// compromised.

	reg_write(0x7F, 0x00);
	reg_write(0x51, 0xFF);
	reg_write(0x4E, 0x2A);
	reg_write(0x66, 0x3E);
	reg_write(0x7F, 0x14);
	reg_write(0x7E, 0x71);
	reg_write(0x55, 0x00);
	reg_write(0x59, 0x00);
	reg_write(0x6F, 0x2C);
	reg_write(0x7F, 0x05);
	reg_write(0x4D, 0xAC);
	reg_write(0x4E, 0x32);
	reg_write(0x7F, 0x09);
	reg_write(0x5C, 0xAF);
	reg_write(0x5F, 0xAF);
	reg_write(0x70, 0x08);
	reg_write(0x71, 0x04);
	reg_write(0x72, 0x06);
	reg_write(0x74, 0x3C);
	reg_write(0x75, 0x28);
	reg_write(0x76, 0x20);
	reg_write(0x4E, 0xBF);
	reg_write(0x7F, 0x03);
	reg_write(0x64, 0x14);
	reg_write(0x65, 0x0A);
	reg_write(0x66, 0x10);
	reg_write(0x55, 0x3C);
	reg_write(0x56, 0x28);
	reg_write(0x57, 0x20);
	reg_write(0x4A, 0x2D);

	reg_write(0x4B, 0x2D);
	reg_write(0x4E, 0x4B);
	reg_write(0x69, 0xFA);
	reg_write(0x7F, 0x05);
	reg_write(0x69, 0x1F);
	reg_write(0x47, 0x1F);
	reg_write(0x48, 0x0C);
	reg_write(0x5A, 0x20);
	reg_write(0x75, 0x0F);
	reg_write(0x4A, 0x0F);
	reg_write(0x42, 0x02);
	reg_write(0x45, 0x03);
	reg_write(0x65, 0x00);
	reg_write(0x67, 0x76);
	reg_write(0x68, 0x76);
	reg_write(0x6A, 0xC5);
	reg_write(0x43, 0x00);
	reg_write(0x7F, 0x06);
	reg_write(0x4A, 0x18);
	reg_write(0x4B, 0x0C);
	reg_write(0x4C, 0x0C);
	reg_write(0x4D, 0x0C);
	reg_write(0x46, 0x0A);
	reg_write(0x59, 0xCD);
	reg_write(0x7F, 0x0A);
	reg_write(0x4A, 0x2A);
	reg_write(0x48, 0x96);
	reg_write(0x52, 0xB4);
	reg_write(0x7F, 0x00);
	reg_write(0x5B, 0xA0);
}

void AP_OpticalFlow_Pixart::ConfigureEnhancedDetectionMode()
{
	// Enhance Detection Setting relatively has better detection sensitivity, it is recommended where yaw motion
	// detection is required, and also where more sensitive challenging surface detection is required. The recommended
	// operating height must be greater than 15 cm to avoid false detection on challenging surfaces due to increasing of
	// sensitivity.

	reg_write(0x7F, 0x00);
	reg_write(0x51, 0xFF);
	reg_write(0x4E, 0x2A);
	reg_write(0x66, 0x26);
	reg_write(0x7F, 0x14);
	reg_write(0x7E, 0x71);
	reg_write(0x55, 0x00);
	reg_write(0x59, 0x00);
	reg_write(0x6F, 0x2C);
	reg_write(0x7F, 0x05);
	reg_write(0x4D, 0xAC);
	reg_write(0x4E, 0x65);
	reg_write(0x7F, 0x09);
	reg_write(0x5C, 0xAF);
	reg_write(0x5F, 0xAF);
	reg_write(0x70, 0x00);
	reg_write(0x71, 0x00);
	reg_write(0x72, 0x00);
	reg_write(0x74, 0x14);
	reg_write(0x75, 0x14);
	reg_write(0x76, 0x06);
	reg_write(0x4E, 0x8F);
	reg_write(0x7F, 0x03);
	reg_write(0x64, 0x00);
	reg_write(0x65, 0x00);
	reg_write(0x66, 0x00);
	reg_write(0x55, 0x14);
	reg_write(0x56, 0x14);
	reg_write(0x57, 0x06);
	reg_write(0x4A, 0x20);

	reg_write(0x4B, 0x20);
	reg_write(0x4E, 0x32);
	reg_write(0x69, 0xFE);
	reg_write(0x7F, 0x05);
	reg_write(0x69, 0x14);
	reg_write(0x47, 0x14);
	reg_write(0x48, 0x1C);
	reg_write(0x5A, 0x20);
	reg_write(0x75, 0xE5);
	reg_write(0x4A, 0x05);
	reg_write(0x42, 0x04);
	reg_write(0x45, 0x03);
	reg_write(0x65, 0x00);
	reg_write(0x67, 0x50);
	reg_write(0x68, 0x50);
	reg_write(0x6A, 0xC5);
	reg_write(0x43, 0x00);
	reg_write(0x7F, 0x06);
	reg_write(0x4A, 0x1E);
	reg_write(0x4B, 0x1E);
	reg_write(0x4C, 0x34);
	reg_write(0x4D, 0x34);
	reg_write(0x46, 0x32);
	reg_write(0x59, 0x0D);
	reg_write(0x7F, 0x0A);
	reg_write(0x4A, 0x2A);
	reg_write(0x48, 0x96);
	reg_write(0x52, 0xB4);
	reg_write(0x7F, 0x00);
	reg_write(0x5B, 0xA0);
}

void AP_OpticalFlow_Pixart::ConfigureAutomaticModeSwitching()
{
	// Automatic switching between Mode 0, 1 and 2:
	reg_write(0x7F, 0x08);
	reg_write(0x68, 0x02);
	reg_write(0x7F, 0x00);

	// Automatic switching between Mode 0 and 1 only:
	// reg_write(0x7F, 0x08);
	// reg_write(0x68, 0x01); // different than mode 0,1,2
	// reg_write(0x7F, 0x00);
}

void AP_OpticalFlow_Pixart::EnableLed()
{
	// Enable LED_N controls
	reg_write(0x7F, 0x14);
	reg_write(0x6F, 0x0C);
	reg_write(0x7F, 0x00);
}

void AP_OpticalFlow_Pixart::Configure()
{
	ConfigureStandardDetectionSetting();
	// ConfigureEnhancedDetectionMode();
    // last_burst_us = AP_HAL::micros();

	ConfigureAutomaticModeSwitching();

	// EnableLed();
}

bool AP_OpticalFlow_Pixart::reset(void)
{
    if (!_dev) {
        debug("SPI bus not found!\n");
        return false;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_chip_select(true);
    hal.scheduler->delay(1);
    _dev->set_chip_select(false);

    // check product ID
    uint8_t id1 = reg_read(PIXART_REG_PRODUCT_ID);
    uint8_t id2 = reg_read(PIXART_REG_INV_PROD_ID);

    debug("id1=0x%02x id2=0x%02x ~id1=0x%02x\n", id1, id2, uint8_t(~id1));
    if (id1 == 0xA2 && id2 == uint8_t(~id1)) {
        debug("Detected PAA3905\n");
    } else {
        debug("Not a recognised device\n");
        return false;
    }

    // power-up sequence
    reg_write(PIXART_REG_POWER_RST, 0x5A);
    hal.scheduler->delay(1);

    reg_read(0x02);
    reg_read(0x03);
    reg_read(0x04);
    reg_read(0x05);
    reg_read(0x06);

    Configure();

    reg_write(0x4E, res);

    debug("Pixart %s ready\n", "3905");
    // integral.last_frame_us = AP_HAL::micros();
    // last_burst_us = AP_HAL::micros();
    mode = unknown;
    // last_update_ms = AP_HAL::millis();

    return true;
}

// setup the device
bool AP_OpticalFlow_Pixart::setup_sensor(void)
{
    if (!_dev) {
        debug("SPI bus not found!\n");
        return false;
    }

    auto *uart = hal.serial(HAL_PERIPH_FLOW_SERIAL_OUT);
    if (uart == nullptr) {
        debug("UART not found!\n");
        return false;
    }

    uart->begin(19200);


    if (reset()) {
        _dev->register_periodic_callback(40000, FUNCTOR_BIND_MEMBER(&AP_OpticalFlow_Pixart::update, void));
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_OpticalFlow_Pixart::timer_thread, void),
                                          "flow",
                                          1024, AP_HAL::Scheduler::PRIORITY_BOOST, 0)) {
            debug("Failed to create flow thread\n");
        }
        // if (hal.gpio->attach_interrupt(FLOW_DRDY_PIN, FUNCTOR_BIND_MEMBER(&AP_OpticalFlow_Pixart::timer_int, void, uint8_t, bool, uint32_t), AP_HAL::GPIO::INTERRUPT_FALLING) != true) {
        //     debug("Could not attach interrupt!\n");
        //     return false;
        // }
        // debug("Attached interrupt!\n");
        return true;
    }
    return false;
}

// write an 8 bit register
void AP_OpticalFlow_Pixart::reg_write(uint8_t reg, uint8_t value)
{
    uint32_t elapsed_us = AP_HAL::micros() - last_spi_op_us;
    if (last_spi_op == rd && (elapsed_us < PIXART_Tsrw)) {
        hal.scheduler->delay_microseconds(PIXART_Tsrw - elapsed_us);
    }
    if (last_spi_op == wr && (elapsed_us < PIXART_Tsww)) {
        hal.scheduler->delay_microseconds(PIXART_Tsww - elapsed_us);
    }
    _dev->set_chip_select(true);
    reg |= PIXART_WRITE_FLAG;
    _dev->transfer(&reg, 1, nullptr, 0);
    _dev->transfer(&value, 1, nullptr, 0);
    _dev->set_chip_select(false);
    last_spi_op = wr;
    last_spi_op_us = AP_HAL::micros();
}

// read from an 8 bit register
uint8_t AP_OpticalFlow_Pixart::reg_read(uint8_t reg)
{
    uint32_t elapsed_us = AP_HAL::micros() - last_spi_op_us;
    if (last_spi_op == rd && (elapsed_us < PIXART_Tsrr)) {
        hal.scheduler->delay_microseconds(PIXART_Tsrr - elapsed_us);
    }
    if (last_spi_op == wr && (elapsed_us < PIXART_Tswr)) {
        hal.scheduler->delay_microseconds(PIXART_Tswr - elapsed_us);
    }
    uint8_t v = 0;
    _dev->set_chip_select(true);
    _dev->transfer(&reg, 1, nullptr, 0);
    hal.scheduler->delay_microseconds(PIXART_Tsrad);
    _dev->transfer(nullptr, 0, &v, 1);
    _dev->set_chip_select(false);
    last_spi_op = rd;
    last_spi_op_us = AP_HAL::micros();
    return v;
}

// read from a 16 bit unsigned register
uint16_t AP_OpticalFlow_Pixart::reg_read16u(uint8_t reg)
{
    uint16_t low = reg_read(reg);
    uint16_t high = reg_read(reg+1);
    return low | (high<<8);
}

// read from a 16 bit signed register
int16_t AP_OpticalFlow_Pixart::reg_read16s(uint8_t reg)
{
    return (int16_t)reg_read16u(reg);
}

bool AP_OpticalFlow_Pixart::motion_burst(uint32_t *dt)
{
    uint8_t *b = (uint8_t *)&burst;
    // bool ret = true;

    _dev->set_chip_select(true);
    uint8_t reg = PIXART_REG_MOT_BURST;

    _dev->transfer(&reg, 1, nullptr, 0);
    hal.scheduler->delay_microseconds(PIXART_Tsrad);
    // uint32_t now = AP_HAL::micros();
    // *dt = now - last_burst_us;
    // last_burst_us = now;
    _dev->transfer(nullptr, 0, b, 14);
    // for (uint8_t i=0; i<sizeof(burst); i++) {
    //     _dev->transfer(nullptr, 0, &b[i], 1);
    //     if (i == 0 && (burst.motion & 0x80) == 0) {
    //         ret = false;
    //     //     // no motion, save some bus bandwidth
    //     //     _dev->set_chip_select(false);
    //     //     // hal.scheduler->delay_microseconds(PIXART_Tbexit);
    //     //     return false;
    //     }
    // }
    _dev->set_chip_select(false);
    // hal.scheduler->delay_microseconds(PIXART_Tbexit);
    return (burst.motion & 0x80) ? true : false;
}

uint32_t AP_OpticalFlow_Pixart::dt_us()
{
    switch (mode) {
        case 0:
            return SAMPLE_INTERVAL_MODE_0;
        case 1:
            return SAMPLE_INTERVAL_MODE_1;
        case 2:
            return SAMPLE_INTERVAL_MODE_2;
        default:
            // debug("Invalid mode\n!");
            return UINT32_MAX;
    }
}

void AP_OpticalFlow_Pixart::timer_int(uint8_t pin, bool pin_state, uint32_t ts)
{
    // if (AP_HAL::micros() - last_burst_us < 500) {
    //     return;
    // }
    // debug("Received interrupt!\n");
    // uint32_t now = AP_HAL::micros();
    // debug("loop %ld\n", now - last_burst_us);
    // last_burst_us = now;
    WITH_SEMAPHORE(_dev->get_semaphore());
    timer();
}

void AP_OpticalFlow_Pixart::send_zeros(void)
{
    // uint32_t now = AP_HAL::millis();
    // uint32_t dt = now - last_update_ms;
    // if (dt > 250) {
    //     uint8_t buf[14] = {0};
    //     buf[0] = 0xFE;
    //     buf[1] = 0x0A;
    //     buf[2] = 0;
    //     buf[3] = 0;
    //     buf[4] = 0;
    //     buf[5] = 0;
    //     buf[6] = (dt * 1000) & 0xFF;
    //     buf[7] = ((dt * 1000) >> 8) & 0xFF;
    //     buf[10] = 0;
    //     buf[12] = buf[2] ^ buf[3] ^ buf[4] ^ buf [5] ^ buf[6] ^ buf[7] ^ buf[8] ^ buf[9] ^ buf[10] ^ buf[11];
    //     buf[13] = 0x55;

    //     auto *uart = hal.serial(HAL_PERIPH_FLOW_SERIAL_OUT);
    //     if (uart == nullptr) {
    //         debug("UART not found!\n");
    //         return;
    //     }
    //     uart->write(buf, 14);
    //     debug("Send zeros after %ldms!\n", dt);
    //     last_update_ms = now;
    // }
}

void AP_OpticalFlow_Pixart::timer_thread(void)
{
    while(true) {
        // uint32_t dt = dt_us();
        if (hal.gpio->wait_pin(FLOW_DRDY_PIN, AP_HAL::GPIO::INTERRUPT_FALLING, /*dt*/ /*20000*/ UINT32_MAX) != true) {
            // send_zeros();
            // if (AP_HAL::millis() - last_update_ms < ((dt * 20) / 1000)) {
                continue;
            // }
        }
        // hal.gpio->wait_pin(FLOW_DRDY_PIN, AP_HAL::GPIO::INTERRUPT_FALLING, 50000);
        // uint32_t now = AP_HAL::micros();
        // debug("loop %ld\n", now - last_burst_us);
        // last_burst_us = now;
        WITH_SEMAPHORE(_dev->get_semaphore());
        timer();
        // hal.scheduler->delay(1);
    }
}

void AP_OpticalFlow_Pixart::timer(void)
{
    // if (AP_HAL::micros() - last_burst_us < 500) {
    //     return;
    // }
    // if (hal.gpio->read(FLOW_DRDY_PIN) != 0) {
    //     return;
    // }

    uint32_t dt = 0;
    if (motion_burst(&dt) != true) {
        // debug("No motion update after %ldms!\n", AP_HAL::millis() - last_update_ms);
        
        return;
    }
    // debug("dt %ld!\n", dt);

    // uint32_t now = AP_HAL::micros();
    // debug("loop %ld\n", now - last_burst_us);
    // last_burst_us = now;

    if (burst.rawdata_sum > 0x98) {
        debug("Invalid raw data sum!\n");
        return;
    }

    if ((burst.observation & 0x3F) != 0x3F) {
		// Other value: recommend to issue a software reset
		debug("Observation not equal to 0x3F, resetting");
		reset();
		return;
	}

    bool new_challenging = burst.motion & 0x1;
    if (new_challenging && !challenging) {
		debug("challenging surface detected");
	} else if (challenging && !new_challenging) {
        debug("challenging surface cleared");
    }
    challenging = new_challenging;

    uint8_t new_mode = (burst.observation >> 6u) & 0x3;
    if (mode != new_mode) {
        debug("Mode changed to %d\n", new_mode);
    }
    mode = static_cast<Mode>(new_mode);

    // if (dt > 0xFFFF) {
    //     debug("dt truncated!!\n");
    //     dt = 0xFFFF;
    // }

    // uint16_t dt = 0;
    uint8_t shutter_lower = burst.shutter_lower;
	uint8_t shutter_middle = burst.shutter_middle;
	uint8_t shutter_upper = burst.shutter_upper & 0x7F;
    uint32_t shutter = (shutter_upper << 16u) | (shutter_middle << 8u) | shutter_lower;

    switch (new_mode) {
        case 0:
            if (burst.squal < 0x19 && shutter >= 0x00FF80) {
                debug("Mode 0 Invalid data\n!");
                // send_zeros();
                // burst.squal = 0;
                return;
            }
            dt = SAMPLE_INTERVAL_MODE_0;
            break;
        case 1:
            if (burst.squal < 0x46 && shutter >= 0x00FF80) {
                debug("Mode 1 Invalid data\n!");
                // send_zeros();
                // burst.squal = 0;
                return;
            }
            dt = SAMPLE_INTERVAL_MODE_1;
            break;
        case 2:
            if (burst.squal < 0x55 && shutter >= 0x025998) {
                debug("Mode 2 Invalid data\n!");
                // send_zeros();
                // burst.squal = 0;
                return;
            }
            dt = SAMPLE_INTERVAL_MODE_2;
            break;
        default:
            debug("Invalid mode\n!");
            return;
    }

    int16_t mot_x = (burst.delta_x_h << 8u) | burst.delta_x_l;
    int16_t mot_y = (burst.delta_y_h << 8u) | burst.delta_y_l;

    // debug("before x_h %u x_l %u y_h %u y_l %u\n!", burst.delta_x_h, burst.delta_x_l, burst.delta_y_h, burst.delta_y_l);

    // mot_x = (int16_t)(((float)mot_x * (10000.f)) / flow_pixel_scaling);
    // mot_y = (int16_t)(((float)mot_y * (10000.f)) / flow_pixel_scaling);

    // // mot_x = (int16_t)(((float)mot_x / flow_pixel_scaling) * 10000);
    // // mot_y = (int16_t)(((float)mot_y / flow_pixel_scaling) * 10000);

    // // debug("after x %d y %d\n!", mot_x, mot_y);

    // uint8_t buf[14] = {0};

    // buf[0] = 0xFE;
    // buf[1] = 0x0A;
    // buf[2] = mot_x & 0xFF;
    // buf[3] = (mot_x >> 8u) & 0xFF;
    // buf[4] = mot_y & 0xFF;
    // buf[5] = (mot_y >> 8u) & 0xFF;
    // buf[6] = dt & 0xFF;
    // buf[7] = (dt >> 8u) & 0xFF;
    // buf[10] = burst.squal;
    // buf[12] = buf[2] ^ buf[3] ^ buf[4] ^ buf [5] ^ buf[6] ^ buf[7] ^ buf[8] ^ buf[9] ^ buf[10] ^ buf[11];
    // buf[13] = 0x55;

    // auto *uart = hal.serial(HAL_PERIPH_FLOW_SERIAL_OUT);
    // if (uart == nullptr) {
    //     debug("UART not found!\n");
    //     return;
    // }
    // uart->write(buf, 14);
    // last_update_ms = AP_HAL::millis();
    // debug("res 0x%X\n", reg_read(0x4E));

    // last_burst_us = AP_HAL::micros();

    // uint32_t dt_us = last_burst_us - integral.last_frame_us;
    // // float dt = dt_us * 1.0e-6;
    // // const Vector3f &gyro = AP::ahrs().get_gyro();

    {
        WITH_SEMAPHORE(_sem);

        integral.sum.x += mot_x;
        integral.sum.y += mot_y;
        integral.sum_us += dt;
        integral.qual_sum += burst.squal;
        integral.count += 1;
        // integral.last_frame_us = last_burst_us;
        // integral.gyro += Vector2f(gyro.x, gyro.y) * dt;
    }

#if 0
    static uint32_t last_print_ms;
    static int fd = -1;
    if (fd == -1) {
        fd = open("/dev/ttyACM0", O_WRONLY);
    }
    // used for debugging
    static int32_t sum_x;
    static int32_t sum_y;
    sum_x += burst.delta_x;
    sum_y += burst.delta_y;

    uint32_t now = AP_HAL::millis();
    if (now - last_print_ms >= 100 && (sum_x != 0 || sum_y != 0)) {
        last_print_ms = now;
        dprintf(fd, "Motion: %d %d obs:0x%02x squal:%u rds:%u maxr:%u minr:%u sup:%u slow:%u\n",
               (int)sum_x, (int)sum_y, (unsigned)burst.squal, (unsigned)burst.rawdata_sum, (unsigned)burst.max_raw,
               (unsigned)burst.max_raw, (unsigned)burst.min_raw, (unsigned)burst.shutter_upper, (unsigned)burst.shutter_lower);
        sum_x = sum_y = 0;
    }
#endif
}

// update - read latest values from sensor and fill in x,y and totals.
void AP_OpticalFlow_Pixart::update(void)
{
    // uint32_t now = AP_HAL::millis();
    // if (now - last_update_ms < 100) {
    //     return;
    // }
    uint8_t buf[14] = {0};

    {
        WITH_SEMAPHORE(_sem);
        if (!integral.sum_us) {
            return;
        }
        int16_t mot_x = (int16_t)(((double)integral.sum.x * 10000) / flow_pixel_scaling);
        int16_t mot_y = (int16_t)(((double)integral.sum.y * 10000) / flow_pixel_scaling);

        // mot_x = (int16_t)(((float)mot_x / flow_pixel_scaling) * 10000);
        // mot_y = (int16_t)(((float)mot_y / flow_pixel_scaling) * 10000);

        // debug("after x %d y %d\n!", mot_x, mot_y);

        buf[0] = 0xFE;
        buf[1] = 0x0A;
        buf[2] = mot_x & 0xFF;
        buf[3] = (mot_x >> 8u) & 0xFF;
        buf[4] = mot_y & 0xFF;
        buf[5] = (mot_y >> 8u) & 0xFF;
        buf[6] = integral.sum_us & 0xFF;
        buf[7] = (integral.sum_us >> 8u) & 0xFF;
        buf[10] = integral.count ? (integral.qual_sum / integral.count) : 0;
        buf[12] = buf[2] ^ buf[3] ^ buf[4] ^ buf [5] ^ buf[6] ^ buf[7] ^ buf[8] ^ buf[9] ^ buf[10] ^ buf[11];
        buf[13] = 0x55;

        integral.sum.zero();
        integral.sum_us = 0;
        integral.qual_sum = 0;
        integral.count = 0;

    }
    auto *uart = hal.serial(HAL_PERIPH_FLOW_SERIAL_OUT);
    if (uart == nullptr) {
        debug("UART not found!\n");
        return;
    }
    uart->write(buf, 14);

    // last_update_ms = now;

    // state.surface_quality = burst.squal;

    // if (integral.sum_us > 0) {
    //     WITH_SEMAPHORE(_sem);

    //     // const Vector2f flowScaler = _flowScaler();
    //     // float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
    //     // float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;
    //     float dt = integral.sum_us * 1.0e-6;

    //     state.flowRate = Vector2f(integral.sum.x, integral.sum.y);
    //     state.flowRate *= flow_pixel_scaling / dt;

    //     // we only apply yaw to flowRate as body rate comes from AHRS
    //     // _applyYaw(state.flowRate);

    //     // state.bodyRate = integral.gyro / dt;

    //     integral.sum.zero();
    //     integral.sum_us = 0;
    //     // integral.gyro.zero();
    // } else {
    //     state.flowRate.zero();
    //     // state.bodyRate.zero();
    // }

    // copy results to front end
    // _update_frontend(state);
}

