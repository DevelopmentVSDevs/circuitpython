/*
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 Stephen Oliver
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "supervisor/board.h"

#include "mpconfigboard.h"
#include "shared-bindings/board/__init__.h"
#include "shared-bindings/busio/SPI.h"
#include "shared-bindings/busio/I2C.h"
#include "shared-bindings/displayio/FourWire.h"
#include "shared-bindings/microcontroller/Pin.h"
#include "shared-module/displayio/__init__.h"
#include "shared-module/displayio/mipi_constants.h"
#include "supervisor/shared/board.h"

#include "axp192.h"

#define DELAY 0x80


uint8_t display_init_sequence[] = {
    0x01, DELAY, 0x80,  // Software reset then delay 0x80 (128ms)
    0xEF, 0x03, 0x03, 0x80, 0x02,
    0xCF, 0x03, 0x00, 0xC1, 0x30,
    0xED, 0x04, 0x64, 0x03, 0x12, 0x81,
    0xE8, 0x03, 0x85, 0x00, 0x78,
    0xCB, 0x05, 0x39, 0x2C, 0x00, 0x34, 0x02,
    0xF7, 0x01, 0x20,
    0xEA, 0x02, 0x00, 0x00,
    0xc0, 0x01, 0x23,  // Power control VRH[5:0]
    0xc1, 0x01, 0x10,  // Power control SAP[2:0];BT[3:0]
    0xc5, 0x02, 0x3e, 0x28,  // VCM control
    0xc7, 0x01, 0x86,  // VCM control2
    0x36, 0x01, 0x48,  // Memory Access Control
    0x37, 0x01, 0x00,  // Vertical scroll zero
    0x3a, 0x01, 0x55,  // COLMOD: Pixel Format Set
    0xb1, 0x02, 0x00, 0x18,  // Frame Rate Control (In Normal Mode/Full Colors
    0xb6, 0x03, 0x08, 0x82, 0x27,  // Display Function Control
    0xF2, 0x01, 0x00,  // 3Gamma Function Disable
    0x26, 0x01, 0x01,  // Gamma curve selected
    0xe0, 0x0f, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,  // Set Gamma
    0xe1, 0x0f, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,  // Set Gamma
    
    0x36, 0x01, 0x08, //MADCTL
    0x21, 0x00, //DISPLAY_INVERSION_ON
    
    0x11, DELAY, 0x78,  // Exit Sleep then delay 0x78 (120ms)
    0x29, DELAY, 0x78   // Display on then delay 0x78 (120ms)
};


static bool pmic_init(void) {
    int rc;
    uint8_t read_buf[1];
    uint8_t write_buf[2];

    busio_i2c_obj_t *internal_i2c = common_hal_board_create_i2c(0);

    // Set LDO2 and LDO3 to 3.0V
    write_buf[0] = AXP192_LDO23_OUT_VOLTAGE;
    write_buf[1] = AXP192_LDO23_OUT_VOLTAGE_LDO2_3_0V |
                   AXP192_LDO23_OUT_VOLTAGE_LDO3_3_0V;
    rc = common_hal_busio_i2c_write(internal_i2c, AXP192_I2C_ADDRESS, write_buf, sizeof(write_buf));
    if (rc != 0) {
        return false;
    }

    // Enable EXTEN, Disable DCDC2
    write_buf[0] = AXP192_EXTEN_DCDC2_CTRL;
    write_buf[1] = AXP192_EXTEN_DCDC2_CTRL_EXTEN;
    rc = common_hal_busio_i2c_write(internal_i2c, AXP192_I2C_ADDRESS, write_buf, sizeof(write_buf));
    if (rc != 0) {
        return false;
    }

    // Enable LDO2, LDO3, DCDC1, DCDC3 by modifying existing register value
    write_buf[0] = AXP192_DCDC13_LDO23_CTRL;
    rc = common_hal_busio_i2c_write_read(internal_i2c, AXP192_I2C_ADDRESS, write_buf, 1, read_buf, sizeof(read_buf));
    if (rc != 0) {
        return false;
    }

    write_buf[0] = AXP192_DCDC13_LDO23_CTRL;
    write_buf[1] = read_buf[0] | 
                   AXP192_DCDC13_LDO23_CTRL_LDO2 |
                   AXP192_DCDC13_LDO23_CTRL_DCDC1 |
                   AXP192_DCDC13_LDO23_CTRL_DCDC3;
    rc = common_hal_busio_i2c_write(internal_i2c, AXP192_I2C_ADDRESS, write_buf, sizeof(write_buf));
    if (rc != 0) {
        return false;
    }

    // set DCDC3 output voltage by modifying existing register value
    write_buf[0] = AXP192_DCDC3_OUT_VOLTAGE;
    rc = common_hal_busio_i2c_write_read(internal_i2c, AXP192_I2C_ADDRESS, write_buf, 1, read_buf, sizeof(read_buf));
    if (rc != 0) {
        return false;
    }

    write_buf[0] = AXP192_DCDC3_OUT_VOLTAGE;
    write_buf[1] = read_buf[0] | 0x7f;
    rc = common_hal_busio_i2c_write(internal_i2c, AXP192_I2C_ADDRESS, write_buf, sizeof(write_buf));
    if (rc != 0) {
        return false;
    }

    // set DCDC1 output voltage
    write_buf[0] = AXP192_DCDC1_OUT_VOLTAGE;
    write_buf[1] = 0x6A;
    rc = common_hal_busio_i2c_write(internal_i2c, AXP192_I2C_ADDRESS, write_buf, sizeof(write_buf));
    if (rc != 0) {
        return false;
    }

    // ADC Sample Rate 200Hz, TS Pin 80uA, Temp Mon, Energy Saving
    write_buf[0] = AXP192_ADC_TS;
    write_buf[1] = AXP192_ADC_TS_SAMPLE_200HZ |
                   AXP192_ADC_TS_OUT_CUR_80uA |
                   AXP192_ADC_TS_PIN_TEMP_MON |
                   AXP192_ADC_TS_PIN_OUT_SAVE_ENG;
    rc = common_hal_busio_i2c_write(internal_i2c, AXP192_I2C_ADDRESS, write_buf, sizeof(write_buf));
    if (rc != 0) {
        return false;
    }

    // ADC Enable Battery, VBus, ACIn, APS, TS
    write_buf[0] = AXP192_ADC_ENABLE_1;
    write_buf[1] = AXP192_ADC_ENABLE_1_BATT_VOL |
                     AXP192_ADC_ENABLE_1_BATT_CUR |
                     AXP192_ADC_ENABLE_1_ACIN_VOL |
                     AXP192_ADC_ENABLE_1_ACIN_CUR |
                     AXP192_ADC_ENABLE_1_VBUS_VOL |
                     AXP192_ADC_ENABLE_1_VBUS_CUR |
                     AXP192_ADC_ENABLE_1_APS_VOL |
                     AXP192_ADC_ENABLE_1_TS_PIN;
    rc = common_hal_busio_i2c_write(internal_i2c, AXP192_I2C_ADDRESS, write_buf, sizeof(write_buf));
    if (rc != 0) {
        return false;
    }

    // VBus limit 500mA, Vbus hold at 4.4V
    write_buf[0] = AXP192_VBUS_IPSOUT;
    write_buf[1] = AXP192_VBUS_IPSOUT_VHOLD_LIMIT |
                   AXP192_VBUS_IPSOUT_VHOLD_VOLTAGE_4_4V |
                   AXP192_VBUS_IPSOUT_VBUS_LIMIT_CURRENT |
                   AXP192_VBUS_IPSOUT_VBUS_LIMIT_CURRENT_500mA;
    rc = common_hal_busio_i2c_write(internal_i2c, AXP192_I2C_ADDRESS, write_buf, sizeof(write_buf));
    if (rc != 0) {
        return false;
    }

    // Automatically power off at 3.0V
    write_buf[0] = AXP192_POWER_OFF_VOLTAGE;
    write_buf[1] = AXP192_POWER_OFF_VOLTAGE_3_0V;
    rc = common_hal_busio_i2c_write(internal_i2c, AXP192_I2C_ADDRESS, write_buf, sizeof(write_buf));
    if (rc != 0) {
        return false;
    }

    // Battery charging voltage 4.2V, current 360mA
    write_buf[0] = AXP192_CHARGING_CTRL1;
    write_buf[1] = AXP192_CHARGING_CTRL1_ENABLE |
                   AXP192_CHARGING_CTRL1_VOLTAGE_4_20V |
                   AXP192_CHARGING_CTRL1_CHARGING_THRESH_10PERC |
                   AXP192_CHARGING_CTRL1_CURRENT_360mA;
    rc = common_hal_busio_i2c_write(internal_i2c, AXP192_I2C_ADDRESS, write_buf, sizeof(write_buf));
    if (rc != 0) {
        return false;
    }

    // PEK Short Press 128ms, Long Press 1.5s, Power Off 4s
    write_buf[0] = AXP192_PEK;
    write_buf[1] = AXP192_PEK_SHORT_PRESS_128mS |
                   AXP192_PEK_LONG_PRESS_1_5S |
                   AXP192_PEK_LONG_PRESS_POWER_OFF |
                   AXP192_PEK_PWROK_DELAY_64mS |
                   AXP192_PEK_POWER_OFF_TIME_4S;
    rc = common_hal_busio_i2c_write(internal_i2c, AXP192_I2C_ADDRESS, write_buf, sizeof(write_buf));
    if (rc != 0) {
        return false;
    }

    // Ensure high temp threshold default value
    write_buf[0] = AXP192_BATT_TEMP_HIGH_THRESH;
    write_buf[1] = AXP192_BATT_TEMP_HIGH_THRESH_DEFAULT;
    rc = common_hal_busio_i2c_write(internal_i2c, AXP192_I2C_ADDRESS, write_buf, sizeof(write_buf));
    if (rc != 0) {
        return false;
    }

    // RTC Backup Battery Enable at 3.0V, charging with 200uA
    write_buf[0] = AXP192_BACKUP_BATT;
    write_buf[1] = AXP192_BACKUP_BATT_CHARGING_ENABLE |
                   AXP192_BACKUP_BATT_CHARGING_VOLTAGE_3_0V |
                   AXP192_BACKUP_BATT_CHARGING_CURRENT_200uA;
    rc = common_hal_busio_i2c_write(internal_i2c, AXP192_I2C_ADDRESS, write_buf, sizeof(write_buf));
    if (rc != 0) {
        return false;
    }

    // Set GPIO0 as LDOIO0
    write_buf[0] = AXP192_GPIO0_FUNCTION;
    write_buf[1] = AXP192_GPIO0_FUNCTION_LDO_OUTPUT;
    rc = common_hal_busio_i2c_write(internal_i2c, AXP192_I2C_ADDRESS, write_buf, sizeof(write_buf));
    if (rc != 0) {
        return false;
    }

    // Set LDOIO0 (now routed to GPIO0) to 3.3V
    write_buf[0] = AXP192_GPIO0_LDO_VOLTAGE;
    write_buf[1] = AXP192_GPIO0_LDO_VOLTAGE_3_3V;
    rc = common_hal_busio_i2c_write(internal_i2c, AXP192_I2C_ADDRESS, write_buf, sizeof(write_buf));
    if (rc != 0) {
        return false;
    }

    // Set GPIO1 to PWM1 for green LED
    write_buf[0] = AXP192_GPIO1_FUNCTION;
    write_buf[1] = AXP192_GPIO1_FUNCTION_PWM1_OUTPUT;
    rc = common_hal_busio_i2c_write(internal_i2c, AXP192_I2C_ADDRESS, write_buf, sizeof(write_buf));
    if (rc != 0) {
        return false;
    }
    
    // Set PWM1 to duty ratio 255 to turn off green LED by default
    write_buf[0] = AXP192_PWM1_DUTY_RATIO;
    write_buf[1] = 255;
    rc = common_hal_busio_i2c_write(internal_i2c, AXP192_I2C_ADDRESS, write_buf, sizeof(write_buf));
    if (rc != 0) {
        return false;
    }

    // set GPIO2 low to enable speaker
    write_buf[0] = AXP192_GPIO2_FUNCTION;
    write_buf[1] = 0x06;
    rc = common_hal_busio_i2c_write(internal_i2c, AXP192_I2C_ADDRESS, write_buf, sizeof(write_buf));
    if (rc != 0) {
        return false;
    }

    return true;
}


static bool display_init(void) {
    busio_spi_obj_t *spi = &displays[0].fourwire_bus.inline_bus;
    common_hal_busio_spi_construct(spi, &pin_GPIO18, &pin_GPIO23, &pin_GPIO38, false);
    common_hal_busio_spi_never_reset(spi);

    displayio_fourwire_obj_t *bus = &displays[0].fourwire_bus;
    bus->base.type = &displayio_fourwire_type;
    common_hal_displayio_fourwire_construct(bus,
        spi,
        &pin_GPIO15, // DC
        &pin_GPIO5, // CS 
        NULL, // RST, but it's connected to PMIC, not ESP32
        32000000, // Baudrate
        0, // Polarity
        0); // Phase

    displayio_display_obj_t *display = &displays[0].display;
    display->base.type = &displayio_display_type;
    common_hal_displayio_display_construct(
        display,
        bus,
        320,            // width (after rotation
        240,            // height (after rotation
        0,             // column start
        0,             // row start
        0,             // rotation
        16,             // color depth
        false,          // grayscale
        false,          // pixels in a byte share a row. Only valid for depths < 8
        1,              // bytes per cell. Only valid for depths < 8
        false,          // reverse_pixels_in_byte. Only valid for depths < 8
        true,           // reverse_pixels_in_word
        MIPI_COMMAND_SET_COLUMN_ADDRESS, // set column command
        MIPI_COMMAND_SET_PAGE_ADDRESS,   // set row command
        MIPI_COMMAND_WRITE_MEMORY_START, // write memory command
        display_init_sequence,
        sizeof(display_init_sequence),
        NULL,    // backlight pin, but it's connected to PMIC, not ESP32
        NO_BRIGHTNESS_COMMAND,
        1.0f,           // brightness (ignored)
        false,          // auto_brightness
        false,          // single_byte_bounds
        false,          // data_as_commands
        true,           // auto_refresh
        90,             // native_frames_per_second
        true,           // backlight_on_high
        false,          // SH1107_addressing
        50000           // backlight pwm frequency
        );

    return true;
}

void board_init(void) {
    if (!pmic_init()) {
        mp_printf(&mp_plat_print, "could not initialize axp192 pmic\n");
        return;
    }

    if (!display_init()) {
        mp_printf(&mp_plat_print, "could not initialize ili9342c LCD");
        return;
    }
}

bool board_requests_safe_mode(void) {
    return false;
}

void reset_board(void) {

}

void board_deinit(void) {
}
