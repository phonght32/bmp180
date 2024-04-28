// MIT License

// Copyright (c) 2024 phonght32

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef __BMP180_H__
#define __BMP180_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "err_code.h"

#define BMP180_I2C_ADDR		(0x77)

typedef err_code_t (*bmp180_func_i2c_send)(uint8_t reg_addr, uint8_t *buf_send, uint16_t len);
typedef err_code_t (*bmp180_func_i2c_recv)(uint8_t reg_addr, uint8_t *buf_recv, uint16_t len);
typedef void (*bmp180_func_delay)(uint32_t ms);

/**
 * @brief   Handle structure.
 */
typedef struct bmp180 *bmp180_handle_t;

/**
 * @brief   Over sampling.
 */
typedef enum {
	BMP180_OVER_SAMPLING_LOW = 0,
	BMP180_OVER_SAMPLING_STANDARD,
	BMP180_OVER_SAMPLING_HIGH,
	BMP180_OVER_SAMPLING_ULTRA
} bmp180_over_sampling_t;

/**
 * @brief   Configuration structure.
 */
typedef struct {
	bmp180_over_sampling_t 		over_sampling;				/*!< Over sampling */
	bmp180_func_i2c_send       	i2c_send;        			/*!< Function send bytes */
	bmp180_func_i2c_recv       	i2c_recv;         			/*!< Function receive bytes */
	bmp180_func_delay          	delay;                 		/*!< Function delay function */
} bmp180_cfg_t;

/*
 * @brief   Initialize BMP180 with default parameters.
 *
 * @note    This function must be called first.
 *
 * @param   None.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
bmp180_handle_t bmp180_init(void);

/*
 * @brief   Set configuration parameters.
 *
 * @param 	handle Handle structure.
 * @param   config Configuration structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t bmp180_set_config(bmp180_handle_t handle, bmp180_cfg_t config);

/*
 * @brief   Configure BMP180 to run.
 *
 * @param 	handle Handle structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t bmp180_config(bmp180_handle_t handle);

/*
 * @brief   Get temperature.
 *
 * @param 	handle Handle structure.
 * @param 	temperature Temperature.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t bmp180_get_temperature(bmp180_handle_t handle, float *temperature);

/*
 * @brief   Get pressure.
 *
 * @param 	handle Handle structure.
 * @param 	pressure Pressure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t bmp180_get_pressure(bmp180_handle_t handle, int32_t *pressure);


#ifdef __cplusplus
}
#endif

#endif /* __BMP180_H__ */