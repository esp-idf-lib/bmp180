/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Frank Bargstedt
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file bmp180.c
 *
 * ESP-IDF driver for BMP180 digital pressure sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2015 Frank Bargstedt\n
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "bmp180.h"
#include <inttypes.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_idf_lib_helpers.h>

#define I2C_FREQ_HZ 1000000 // The datasheet allows up to 3.4 MHz

static const char *TAG = "bmp180";

#define BMP180_VERSION_REG        0xD0
#define BMP180_CONTROL_REG        0xF4
#define BMP180_RESET_REG          0xE0
#define BMP180_OUT_MSB_REG        0xF6

#define BMP180_CALIBRATION_REG    0xAA
#define BMP180_CALIBRATION_SIZE   22

// Values for BMP180_CONTROL_REG
#define BMP180_MEASURE_TEMP       0x2E
#define BMP180_MEASURE_PRESS      0x34

// Sco bit of BMP180_CONTROL_REG, stays 1 while the conversion is in progress
#define BMP180_CONTROL_SCO        BIT(5)

// CHIP ID stored in BMP180_VERSION_REG
#define BMP180_CHIP_ID            0x55

// Reset value for BMP180_RESET_REG
#define BMP180_RESET_VALUE        0xB6

// The reset runs the power-on sequence; the datasheet does not specify the
// time, Zephyr waits 2 ms
#define BMP180_RESET_DELAY_MS     2

// Conversion polling
#define BMP180_POLL_DELAY_US      500
#define BMP180_POLL_MARGIN_US     10000

// Max conversion time, datasheet table 3
#define BMP180_TEMP_CONVERSION_US  4500
static const uint32_t bmp180_press_conversion_us[] = { 4500, 7500, 13500, 25500 };

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static int16_t bmp180_be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static esp_err_t bmp180_read_reg_16(i2c_dev_t *dev, uint8_t reg, int16_t *r)
{
    uint8_t d[] = { 0, 0 };

    CHECK(i2c_dev_read_reg(dev, reg, d, sizeof(d)));
    *r = bmp180_be16(d);

    return ESP_OK;
}

static esp_err_t bmp180_wait_conversion(i2c_dev_t *dev, uint32_t conversion_us)
{
    uint32_t timeout_us = conversion_us + BMP180_POLL_MARGIN_US;
    uint32_t elapsed_us = 0;

    while (true)
    {
        uint8_t ctrl;
        CHECK(i2c_dev_read_reg(dev, BMP180_CONTROL_REG, &ctrl, 1));
        if (!(ctrl & BMP180_CONTROL_SCO))
            return ESP_OK;

        if (elapsed_us >= timeout_us)
        {
            ESP_LOGE(TAG, "The conversion did not finish in %" PRIu32 " us", timeout_us);
            return ESP_ERR_TIMEOUT;
        }

        esp_rom_delay_us(BMP180_POLL_DELAY_US);
        elapsed_us += BMP180_POLL_DELAY_US;
    }
}

static esp_err_t bmp180_start_measurement(i2c_dev_t *dev, uint8_t cmd, uint32_t conversion_us)
{
    CHECK(i2c_dev_write_reg(dev, BMP180_CONTROL_REG, &cmd, 1));

    return bmp180_wait_conversion(dev, conversion_us);
}

static esp_err_t bmp180_get_uncompensated_temperature(i2c_dev_t *dev, int32_t *ut)
{
    CHECK(bmp180_start_measurement(dev, BMP180_MEASURE_TEMP, BMP180_TEMP_CONVERSION_US));

    int16_t v;
    CHECK(bmp180_read_reg_16(dev, BMP180_OUT_MSB_REG, &v));
    *ut = v;

    return ESP_OK;
}

static esp_err_t bmp180_get_uncompensated_pressure(i2c_dev_t *dev, bmp180_mode_t oss, uint32_t *up)
{
    CHECK(bmp180_start_measurement(dev, BMP180_MEASURE_PRESS | (oss << 6), bmp180_press_conversion_us[oss]));

    uint8_t d[] = { 0, 0, 0 };
    CHECK(i2c_dev_read_reg(dev, BMP180_OUT_MSB_REG, d, sizeof(d)));

    uint32_t r = ((uint32_t)d[0] << 16) | ((uint32_t)d[1] << 8) | d[2];
    *up = r >> (8 - oss);

    return ESP_OK;
}

esp_err_t bmp180_init_desc(bmp180_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = BMP180_DEVICE_ADDRESS;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t bmp180_free_desc(bmp180_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t bmp180_reset(bmp180_dev_t *dev)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    uint8_t value = BMP180_RESET_VALUE;
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, BMP180_RESET_REG, &value, 1));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    TickType_t delay = pdMS_TO_TICKS(BMP180_RESET_DELAY_MS);
    vTaskDelay(delay ? delay : 1);

    return ESP_OK;
}

esp_err_t bmp180_init(bmp180_dev_t *dev)
{
    CHECK_ARG(dev);

    CHECK(bmp180_reset(dev));

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    uint8_t id;
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, BMP180_VERSION_REG, &id, 1));
    if (id != BMP180_CHIP_ID)
    {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        ESP_LOGE(TAG, "Invalid device ID: 0x%02x", id);
        return ESP_ERR_NOT_FOUND;
    }

    // The coefficients are read at once so that a failed transaction cannot
    // mix the values of two reads
    uint8_t cal[BMP180_CALIBRATION_SIZE];
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, BMP180_CALIBRATION_REG, cal, sizeof(cal)));

    dev->AC1 = bmp180_be16(&cal[0]);
    dev->AC2 = bmp180_be16(&cal[2]);
    dev->AC3 = bmp180_be16(&cal[4]);
    dev->AC4 = (uint16_t)bmp180_be16(&cal[6]);
    dev->AC5 = (uint16_t)bmp180_be16(&cal[8]);
    dev->AC6 = (uint16_t)bmp180_be16(&cal[10]);
    dev->B1 = bmp180_be16(&cal[12]);
    dev->B2 = bmp180_be16(&cal[14]);
    dev->MB = bmp180_be16(&cal[16]);
    dev->MC = bmp180_be16(&cal[18]);
    dev->MD = bmp180_be16(&cal[20]);

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    ESP_LOGD(TAG, "AC1:=%d AC2:=%d AC3:=%d AC4:=%u AC5:=%u AC6:=%u", dev->AC1, dev->AC2, dev->AC3, dev->AC4, dev->AC5, dev->AC6);
    ESP_LOGD(TAG, "B1:=%d B2:=%d", dev->B1, dev->B2);
    ESP_LOGD(TAG, "MB:=%d MC:=%d MD:=%d", dev->MB, dev->MC, dev->MD);

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t bmp180_measure(bmp180_dev_t *dev, float *temperature, uint32_t *pressure, bmp180_mode_t oss)
{
    CHECK_ARG(dev && temperature && pressure);
    CHECK_ARG(oss >= BMP180_MODE_ULTRA_LOW_POWER && oss <= BMP180_MODE_ULTRA_HIGH_RESOLUTION);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    // Temperature is always needed, also required for pressure only.
    //
    // Calculation taken from BMP180 Datasheet
    int32_t T, P;
    int32_t UT, X1, X2, B5;
    UT = 0;
    I2C_DEV_CHECK(&dev->i2c_dev, bmp180_get_uncompensated_temperature(&dev->i2c_dev, &UT));

    X1 = ((UT - (int32_t)dev->AC6) * (int32_t)dev->AC5) >> 15;

    int32_t divisor = X1 + (int32_t)dev->MD;
    if (!divisor)
    {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        ESP_LOGE(TAG, "Invalid calibration data: division by zero");
        return ESP_ERR_INVALID_RESPONSE;
    }

    X2 = ((int32_t)dev->MC << 11) / divisor;
    B5 = X1 + X2;
    T = (B5 + 8) >> 4;

    if (temperature)
        *temperature = T / 10.0;

    ESP_LOGD(TAG, "T:= %" PRIi32 ".%d", T / 10, abs(T % 10));

    if (pressure)
    {
        int32_t X3, B3, B6;
        uint32_t B4, B7, UP = 0;

        I2C_DEV_CHECK(&dev->i2c_dev, bmp180_get_uncompensated_pressure(&dev->i2c_dev, oss, &UP));

        // Calculation taken from BMP180 Datasheet
        B6 = B5 - 4000;
        X1 = ((int32_t)dev->B2 * ((B6 * B6) >> 12)) >> 11;
        X2 = ((int32_t)dev->AC2 * B6) >> 11;
        X3 = X1 + X2;

        B3 = ((((int32_t)dev->AC1 * 4 + X3) << oss) + 2) >> 2;
        X1 = ((int32_t)dev->AC3 * B6) >> 13;
        X2 = ((int32_t)dev->B1 * ((B6 * B6) >> 12)) >> 16;
        X3 = ((X1 + X2) + 2) >> 2;
        B4 = ((uint32_t)dev->AC4 * (uint32_t)(X3 + 32768)) >> 15;
        if (!B4)
        {
            I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
            ESP_LOGE(TAG, "Invalid calibration data: division by zero");
            return ESP_ERR_INVALID_RESPONSE;
        }

        B7 = ((uint32_t)UP - (uint32_t)B3) * (uint32_t)(50000UL >> oss);

        if (B7 < 0x80000000UL)
            P = (B7 * 2) / B4;
        else
            P = (B7 / B4) * 2;

        X1 = (P >> 8) * (P >> 8);
        X1 = (X1 * 3038) >> 16;
        X2 = (-7357 * P) >> 16;
        P = P + ((X1 + X2 + (int32_t)3791) >> 4);

        if (P <= 0)
        {
            I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
            ESP_LOGE(TAG, "Invalid pressure: %" PRIi32 " Pa", P);
            return ESP_ERR_INVALID_RESPONSE;
        }

        *pressure = P;

        ESP_LOGD(TAG, "P:= %" PRIi32, P);
    }

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}
