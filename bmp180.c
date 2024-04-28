#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "bmp180.h"

#define BMP180_CALIB_AC1_MSB  		0xAA
#define BMP180_CALIB_AC1_LSB  		0xAB
#define BMP180_CALIB_AC2_MSB  		0xAC
#define BMP180_CALIB_AC2_LSB  		0xAD
#define BMP180_CALIB_AC3_MSB  		0xAE
#define BMP180_CALIB_AC3_LSB  		0xAF
#define BMP180_CALIB_AC4_MSB  		0xB0
#define BMP180_CALIB_AC4_LSB  		0xB1
#define BMP180_CALIB_AC5_MSB  		0xB2
#define BMP180_CALIB_AC5_LSB  		0xB3
#define BMP180_CALIB_AC6_MSB  		0xB4
#define BMP180_CALIB_AC6_LSB  		0xB5
#define BMP180_CALIB_B1_MSB  		0xB6
#define BMP180_CALIB_B1_LSB  		0xB7
#define BMP180_CALIB_B2_MSB  		0xB8
#define BMP180_CALIB_B2_LSB  		0xB9
#define BMP180_CALIB_MB_MSB  		0xBA
#define BMP180_CALIB_MB_LSB  		0xBB
#define BMP180_CALIB_MC_MSB  		0xBC
#define BMP180_CALIB_MC_LSB  		0xBD
#define BMP180_CALIB_MD_MSB  		0xBE
#define BMP180_CALIB_MD_LSB  		0xBF

#define BMP180_CHIP_ID 				0xD0
#define BMP180_SOFT_RESET 			0xE0
#define BMP180_CONTROL_MEAS 		0xF4
#define BMP180_OUT_MSB 				0xF6
#define BMP180_OUT_LSB   			0xF7
#define BMP180_OUT_XLSB   			0xF8

#define NUM_CALIB_REG  				22

#define BMP180_READ_TEMP_CMD		0x2E

#define BMP180_READ_TEMP_DELAY		5


typedef struct bmp180 {
	bmp180_over_sampling_t 		over_sampling;				/*!< Over sampling */
	bmp180_func_i2c_send       	i2c_send;        			/*!< Function send bytes */
	bmp180_func_i2c_recv       	i2c_recv;         			/*!< Function receive bytes */
	bmp180_func_delay          	delay;                 		/*!< Function delay function */
	int16_t 					ac1;						/*!< Calibration coefficient AC1 */
	int16_t 					ac2;						/*!< Calibration coefficient AC2 */
	int16_t 					ac3;						/*!< Calibration coefficient AC3 */
	uint16_t 					ac4;						/*!< Calibration coefficient AC4 */
	uint16_t 					ac5;						/*!< Calibration coefficient AC5 */
	uint16_t 					ac6;						/*!< Calibration coefficient AC6 */
	int16_t 					b1;							/*!< Calibration coefficient B1 */
	int16_t 					b2;							/*!< Calibration coefficient B2 */
	int16_t 					mb;							/*!< Calibration coefficient MB */
	int16_t 					mc;							/*!< Calibration coefficient MC */
	int16_t 					md;							/*!< Calibration coefficient MD */
} bmp180_t;

bmp180_handle_t bmp180_init(void)
{
	bmp180_handle_t handle = calloc(1, sizeof(bmp180_t));
	if (handle == NULL)
	{
		return NULL;
	}

	return handle;
}

err_code_t bmp180_set_config(bmp180_handle_t handle, bmp180_cfg_t config)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	handle->over_sampling = config.over_sampling;
	handle->i2c_send = config.i2c_send;
	handle->i2c_recv = config.i2c_recv;
	handle->delay = config.delay;

	return ERR_CODE_SUCCESS;
}

err_code_t bmp180_config(bmp180_handle_t handle)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	uint8_t calib_coef[NUM_CALIB_REG] = {0};
	handle->i2c_recv(BMP180_CALIB_AC1_MSB, calib_coef, NUM_CALIB_REG);

	handle->ac1 =  (int16_t)((calib_coef[0]  << 8) + calib_coef[1]);
	handle->ac2 =  (int16_t)((calib_coef[2]  << 8) + calib_coef[3]);
	handle->ac3 =  (int16_t)((calib_coef[4]  << 8) + calib_coef[5]);
	handle->ac4 = (uint16_t)((calib_coef[6]  << 8) + calib_coef[7]);
	handle->ac5 = (uint16_t)((calib_coef[8]  << 8) + calib_coef[9]);
	handle->ac6 = (uint16_t)((calib_coef[10] << 8) + calib_coef[11]);
	handle->b1  =  (int16_t)((calib_coef[12] << 8) + calib_coef[13]);
	handle->b2  =  (int16_t)((calib_coef[14] << 8) + calib_coef[15]);
	handle->mb  =  (int16_t)((calib_coef[16] << 8) + calib_coef[17]);
	handle->mc  =  (int16_t)((calib_coef[18] << 8) + calib_coef[19]);
	handle->md  =  (int16_t)((calib_coef[20] << 8) + calib_coef[21]);

	return ERR_CODE_SUCCESS;
}

err_code_t bmp180_get_temperature(bmp180_handle_t handle, float *temperature)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	uint8_t cmd_data = BMP180_READ_TEMP_CMD;
	uint8_t out_reg[2];
	int32_t ut, x1, x2, b5;
	float result;

	handle->i2c_send(BMP180_CONTROL_MEAS, &cmd_data, 1);
	handle->delay(BMP180_READ_TEMP_DELAY);
	handle->i2c_recv(BMP180_OUT_MSB, out_reg, 2);

	ut = (out_reg[0] << 8) | out_reg[1];

	x1 = (ut - handle->ac6) * handle->ac5 / (1 << 15);
	x2 = (handle->mc * (1 << 11)) / (x1 + handle->md);
	b5 = x1 + x2;
	result = ((b5 + 8) / (1 << 4)) / 10;

	*temperature = result;

	return ERR_CODE_SUCCESS;
}

err_code_t bmp180_get_pressure(bmp180_handle_t handle, int32_t *pressure)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	uint8_t cmd_data = 0;
	uint8_t out_reg[3];
	int32_t ut, up, x1, x2, x3, b3, b4, b5, b6, b7;
	uint32_t read_pressure_delay = 0;
	int32_t p;

	handle->i2c_send(BMP180_CONTROL_MEAS, &cmd_data, 1);
	handle->delay(BMP180_READ_TEMP_DELAY);
	handle->i2c_recv(BMP180_OUT_MSB, out_reg, 2);

	ut = (out_reg[0] << 8) | out_reg[1];

	switch (handle->over_sampling)
	{
	case BMP180_OVER_SAMPLING_LOW:
		read_pressure_delay = 5;
		cmd_data = 0x34;
		break;
	case BMP180_OVER_SAMPLING_STANDARD:
		read_pressure_delay = 8;
		cmd_data = 0x74;
		break;
	case BMP180_OVER_SAMPLING_HIGH:
		read_pressure_delay = 14;
		cmd_data = 0xb4;
		break;
	case BMP180_OVER_SAMPLING_ULTRA:
		read_pressure_delay = 26;
		cmd_data = 0xf4;
		break;
	default:
		read_pressure_delay = 5;
		cmd_data = 0x34;
		break;
	}

	handle->i2c_send(BMP180_CONTROL_MEAS, &cmd_data, 1);
	handle->delay(read_pressure_delay);
	handle->i2c_recv(BMP180_OUT_MSB, out_reg, 3);

	up = ((out_reg[0] << 16) | (out_reg[1] << 8) | out_reg[2]) >> (8 - handle->over_sampling);

	x1 = (ut - handle->ac6) * handle->ac5 / (1 << 15);
	x2 = (handle->mc * (1 << 11)) / (x1 + handle->md);
	b5 = x1 + x2;
	b6 = b5 - 4000;
	x1 = (handle->b2 * (b6 * b6 / (1 << 12))) / (1 << 11);
	x2 = handle->ac2 * b6 / (1 << 11);
	x3 = x1 + x2;
	b3 = (((handle->ac1 * 4 + x3) << handle->over_sampling) + 2) / 4;
	x1 = handle->ac3 * b6 / (1 << 13);
	x2 = (handle->b1 * (b6 * b6 / (1 << 12))) / (1 << 16);
	x3 = ((x1 + x2) + 2) / 4;
	b4 = handle->ac4 * (uint32_t) (x3 + 32768) / (1 << 15);
	b7 = ((uint32_t) up - b3) * (50000 >> handle->over_sampling);
	if (b7 < 0x80000000)
		p = (b7 * 2) / b4;
	else
		p = (b7 / b4) * 2;
	x1 = (p / (1 << 8)) * (p / (1 << 8));
	x1 = (x1 * 3038) / (1 << 16);
	x2 = (-7357 * p) / (1 << 16);
	p = p + (x1 + x2 + 3791) / (1 << 4);

	*pressure = p;

	return ERR_CODE_SUCCESS;
}