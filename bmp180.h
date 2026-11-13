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
 * @file bmp180.h
 * @defgroup bmp180 bmp180
 * @{
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
#ifndef __BMP180_H__
#define __BMP180_H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#define BMP180_DEVICE_ADDRESS 0x77 //!< I2C address

#ifdef __cplusplus
extern "C" {
#endif

/**
 * BMP180 device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;

    int16_t  AC1;
    int16_t  AC2;
    int16_t  AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;

    int16_t  B1;
    int16_t  B2;

    int16_t  MB;
    int16_t  MC;
    int16_t  MD;
} bmp180_dev_t;

/**
 * Hardware accuracy mode.
 * See Table 3 of the datasheet
 */
typedef enum
{
    BMP180_MODE_ULTRA_LOW_POWER = 0,  //!< 1 sample, 4.5 ms
    BMP180_MODE_STANDARD,             //!< 2 samples, 7.5 ms
    BMP180_MODE_HIGH_RESOLUTION,      //!< 4 samples, 13.5 ms
    BMP180_MODE_ULTRA_HIGH_RESOLUTION //!< 8 samples, 25.5 ms
} bmp180_mode_t;


/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param port I2C port number
 * @param sda_gpio GPIO pin number for SDA
 * @param scl_gpio GPIO pin number for SCL
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p dev is NULL
 *      - `ESP_ERR_NO_MEM` if the descriptor mutex cannot be created
 */
esp_err_t bmp180_init_desc(bmp180_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Pointer to BMP180 device descriptor
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p dev is NULL
 */
esp_err_t bmp180_free_desc(bmp180_dev_t *dev);

/**
 * @brief Reset the sensor
 *
 * Runs the power-on sequence: the control and the data registers return to
 * their default values and an ongoing conversion is aborted. The calibration
 * coefficients are stored in the E2PROM and survive the reset.
 *
 * @param dev Pointer to BMP180 device descriptor
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p dev is NULL
 *      - another error code from the I2C layer
 */
esp_err_t bmp180_reset(bmp180_dev_t *dev);

/**
 * @brief Initialize device
 *
 * Calls @ref bmp180_reset, verifies the chip ID and reads the calibration
 * coefficients. The reset puts the sensor into the known state after an MCU
 * restart while the sensor stays powered.
 *
 * @param dev Pointer to BMP180 device descriptor
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p dev is NULL
 *      - `ESP_ERR_NOT_FOUND` if the chip ID is not the BMP180 one
 *      - another error code from the I2C layer
 */
esp_err_t bmp180_init(bmp180_dev_t *dev);

/**
 * @brief Measure temperature and pressure
 *
 * The temperature is measured before the pressure, the pressure measurement
 * needs the temperature one. At least one of the output parameters has to be
 * given; when the pressure is not requested it is not measured, which saves
 * the conversion time of its mode.
 *
 * @param dev Pointer to BMP180 device descriptor
 * @param[out] temperature Temperature in degrees Celsius, may be NULL
 * @param[out] pressure Pressure in Pa, may be NULL
 * @param oss Measurement mode, one of @ref bmp180_mode_t
 * @return
 *      - `ESP_OK` on success
 *      - `ESP_ERR_INVALID_ARG` if @p dev is NULL, both outputs are NULL or
 *        @p oss is out of range
 *      - `ESP_ERR_TIMEOUT` if a conversion does not finish in time
 *      - `ESP_ERR_INVALID_RESPONSE` if the measured value is not physical,
 *        which means that the calibration data is corrupted
 *      - another error code from the I2C layer
 */
esp_err_t bmp180_measure(bmp180_dev_t *dev, float *temperature, uint32_t *pressure, bmp180_mode_t oss);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __BMP180_H__ */
