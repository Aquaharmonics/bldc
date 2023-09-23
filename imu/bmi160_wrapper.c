/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "bmi160_wrapper.h"
#include "utils_math.h"

#include <stdio.h>
#include <string.h>

// Threads
static THD_FUNCTION(bmi_thread, arg);

// Private functions
static bool reset_init_bmi(BMI_STATE *s);
void user_delay_ms(uint32_t ms);

void bmi160_wrapper_init(BMI_STATE *s, stkalign_t *work_area, size_t work_area_size) {
	s->read_callback = 0;

	if (s->sensor.interface == BMI160_SPI_INTF) {
		s->rate_hz = MIN(s->rate_hz, 5000);
	} else {
		s->rate_hz = MIN(s->rate_hz, 1000);
	}

	if (reset_init_bmi(s)) {
		s->should_stop = false;
		chThdCreateStatic(work_area, work_area_size, NORMALPRIO, bmi_thread, s);
	}
}

void bmi160_wrapper_set_read_callback(BMI_STATE *s, void(*func)(float *accel, float *gyro, float *mag)) {
	s->read_callback = func;
}

void bmi160_wrapper_stop(BMI_STATE *s) {
	s->should_stop = true;
	while(s->is_running) {
		chThdSleep(1);
	}
}

static bool reset_init_bmi(BMI_STATE *s) {
	s->sensor.delay_ms = user_delay_ms;

	bmi160_init(&(s->sensor));

	//Alex changes. Lowered the ranges to 2G and 125DPS. This should give better resolution
	//for the measuerment ranges we care about for balance boards.
	//In testing the IMU on the board, I found it hard to get the board to saturate the 2G range
	// for the accelerometer, and hard to saturate the 125DPS range for the gyro.


	s->sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
	s->sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

	s->sensor.gyro_cfg.range = BMI160_GYRO_RANGE_125_DPS;
	s->sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

	// Note for other future IMU mods: maybe do any other initilization here????

	if(s->rate_hz <= 25){
		s->sensor.accel_cfg.odr = BMI160_ACCEL_ODR_25HZ;
		s->sensor.gyro_cfg.odr = BMI160_GYRO_ODR_50HZ;
	}else if(s->rate_hz <= 50){
		s->sensor.accel_cfg.odr = BMI160_ACCEL_ODR_50HZ;
		s->sensor.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
	}else if(s->rate_hz <= 100){
		s->sensor.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
		s->sensor.gyro_cfg.odr = BMI160_GYRO_ODR_200HZ;
	}else if(s->rate_hz <= 200){
		s->sensor.accel_cfg.odr = BMI160_ACCEL_ODR_200HZ;
		s->sensor.gyro_cfg.odr = BMI160_GYRO_ODR_400HZ;
	}else if(s->rate_hz <= 400){
		s->sensor.accel_cfg.odr = BMI160_ACCEL_ODR_400HZ;
		s->sensor.gyro_cfg.odr = BMI160_GYRO_ODR_800HZ;
	}else if(s->rate_hz <= 800){
		s->sensor.accel_cfg.odr = BMI160_ACCEL_ODR_800HZ;
		s->sensor.gyro_cfg.odr = BMI160_GYRO_ODR_1600HZ;
	}else{
		s->sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
		s->sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
	}

	if(s->filter == IMU_FILTER_LOW){
		s->sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
		s->sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
	}else if(s->filter == IMU_FILTER_MEDIUM){
		s->sensor.accel_cfg.bw = BMI160_ACCEL_BW_OSR2_AVG2;
		s->sensor.gyro_cfg.bw = BMI160_GYRO_BW_OSR2_MODE;
		s->sensor.accel_cfg.odr = fmin(s->sensor.accel_cfg.odr + 1, BMI160_ACCEL_ODR_800HZ);
		s->sensor.gyro_cfg.odr = fmin(s->sensor.gyro_cfg.odr + 1, BMI160_GYRO_ODR_1600HZ);
	}else if(s->filter == IMU_FILTER_HIGH){
		s->sensor.accel_cfg.bw = BMI160_ACCEL_BW_OSR4_AVG1;
		s->sensor.gyro_cfg.bw = BMI160_GYRO_BW_OSR4_MODE;
		s->sensor.accel_cfg.odr = fmin(s->sensor.accel_cfg.odr + 2, BMI160_ACCEL_ODR_1600HZ);
		s->sensor.gyro_cfg.odr = fmin(s->sensor.gyro_cfg.odr + 2, BMI160_GYRO_ODR_3200HZ);
	}

	chThdSleepMilliseconds(50);
	int8_t res = bmi160_set_sens_conf(&(s->sensor));
	chThdSleepMilliseconds(50);

	return res == BMI160_OK;
}

void user_delay_ms(uint32_t ms) {
	chThdSleepMilliseconds(ms);
}

static THD_FUNCTION(bmi_thread, arg) {
	BMI_STATE *s = (BMI_STATE*)arg;

	chRegSetThreadName("BMI Sampling");

	s->is_running = true;

	systime_t iteration_timer = chVTGetSystemTimeX();
	const systime_t desired_interval = US2ST(1000000 / s->rate_hz);

	for(;;) {
		struct bmi160_sensor_data accel;
		struct bmi160_sensor_data gyro;
		// To read both Accel and Gyro data along with time based on the BMI160 datasheet and the bmi160_defs/h 
		// int8_t res = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL), 
		// 		&accel, &gyro, &(s->sensor));

			// original code, not sure that this is correct to get time stamps out of accelerometer based on the BMI160 datasheet and the bmi160_defs/h 
		int8_t res = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL),
				&accel, &gyro, &(s->sensor));

		if (res != BMI160_OK) {
			chThdSleepMilliseconds(5);
			continue;
		}
		
		//Note: this appears to be the  first manipulation of the IMU data, as in, the data is still in the IMU's native units. 

		//Note: this appears to be the  first manipulation of the IMU data

		// Alex changes. Adjusted full scale range values here as well.
		
		float tmp_accel[3], tmp_gyro[3], tmp_mag[3];
		//uint32_t accel_sensor_time=0; 
		//uint32_t DT=0;
		//uint32_t last_accel_sensor_time=0;
		
		
		tmp_accel[0] = (float)accel.x * 2.0 / 32768.0;
		tmp_accel[1] = (float)accel.y * 2.0 / 32768.0;
		tmp_accel[2] = (float)accel.z * 2.0 / 32768.0;

		tmp_gyro[0] = (float)gyro.x * 125.0 / 32768.0;
		tmp_gyro[1] = (float)gyro.y * 125.0 / 32768.0;
		tmp_gyro[2] = (float)gyro.z * 125.0 / 32768.0;
		
		memset(tmp_mag, 0, sizeof(tmp_mag));
		// uint32_t accel_sensor_time = accel.sensortime;
		// //uint32_t DT  = (uint32_t)accel.sensortime;
		// //u_int32_t DT= (accel_sensor_time -accel_sensor_time);
		// //last_accel_sensor_time = accel.sensortime;

		if (s->read_callback) {
			//s->read_callback(tmp_accel, tmp_gyro, tmp_mag, accel_sensor_time); 
			s->read_callback(tmp_accel, tmp_gyro, tmp_mag); 
		}

		if (s->should_stop) {
			s->is_running = false;
			return;
		}

		// Delay between loops
		iteration_timer += desired_interval;
		systime_t current_time = chVTGetSystemTimeX();
		systime_t remainin_sleep_time = iteration_timer - current_time;
		if (remainin_sleep_time > 0 && remainin_sleep_time < desired_interval) {
			// Sleep the remaining time.
			chThdSleep(remainin_sleep_time);
		}
		else {
			// Read was too slow or CPU was too buzy, reset the schedule.
			iteration_timer = current_time;
			chThdSleep(desired_interval);
		}
	}
}
// /*!
//  * @brief This API reads accel and gyro data along with sensor time
//  * if time is requested by user.
//  *  Kindly refer the user guide(README.md) for more info.
//  */
// static int8_t get_accel_gyro_data(uint8_t len,
//                                   struct bmi160_sensor_data *accel,
//                                   struct bmi160_sensor_data *gyro,
//                                   const struct bmi160_dev *dev)
// {
//     int8_t rslt;
//     uint8_t idx = 0;
//     uint8_t data_array[15] = { 0 };
//     uint8_t time_0 = 0;
//     uint16_t time_1 = 0;
//     uint32_t time_2 = 0;
//     uint8_t lsb;
//     uint8_t msb;
//     int16_t msblsb;

//     /* read both accel and gyro sensor data
//      * along with time if requested */
//     rslt = bmi160_get_regs(BMI160_GYRO_DATA_ADDR, data_array, 12 + len, dev);
//     if (rslt == BMI160_OK)
//     {
//         /* Gyro Data */
//         lsb = data_array[idx++];
//         msb = data_array[idx++];
//         msblsb = (int16_t)((msb << 8) | lsb);
//         gyro->x = msblsb; /* gyro X axis data */
//         lsb = data_array[idx++];
//         msb = data_array[idx++];
//         msblsb = (int16_t)((msb << 8) | lsb);
//         gyro->y = msblsb; /* gyro Y axis data */
//         lsb = data_array[idx++];
//         msb = data_array[idx++];
//         msblsb = (int16_t)((msb << 8) | lsb);
//         gyro->z = msblsb; /* gyro Z axis data */
//         /* Accel Data */
//         lsb = data_array[idx++];
//         msb = data_array[idx++];
//         msblsb = (int16_t)((msb << 8) | lsb);
//         accel->x = (int16_t)msblsb; /* accel X axis data */
//         lsb = data_array[idx++];
//         msb = data_array[idx++];
//         msblsb = (int16_t)((msb << 8) | lsb);
//         accel->y = (int16_t)msblsb; /* accel Y axis data */
//         lsb = data_array[idx++];
//         msb = data_array[idx++];
//         msblsb = (int16_t)((msb << 8) | lsb);
//         accel->z = (int16_t)msblsb; /* accel Z axis data */
//         if (len == 3)
//         {
//             time_0 = data_array[idx++];
//             time_1 = (uint16_t)(data_array[idx++] << 8);
//             time_2 = (uint32_t)(data_array[idx++] << 16);
//             accel->sensortime = (uint32_t)(time_2 | time_1 | time_0);
//             gyro->sensortime = (uint32_t)(time_2 | time_1 | time_0);
//         }
//         else
//         {
//             accel->sensortime = 0;
//             gyro->sensortime = 0;
//         }
//     }
//     else
//     {
//         rslt = BMI160_E_COM_FAIL;
//     }

//     return rslt;
// }


// struct bmi160_sensor_data accel, gyro;
// int8_t result = get_accel_gyro_data(len, &accel, &gyro, &dev);
// if (result == BMI160_OK) {
//     // Access accelerometer data and sensor time
//     int16_t accel_x = accel.x;
//     int16_t accel_y = accel.y;
//     int16_t accel_z = accel.z;
//     uint32_t accel_sensor_time = accel.sensortime;
//     // ... (process the data further)
// } else {
//     // Handle error
// }