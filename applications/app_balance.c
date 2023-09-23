/*
	Copyright 2019 Mitch Lustig

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

#include "conf_general.h"

#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout
#include "commands.h"
#include "imu/imu.h"
#include "imu/ahrs.h"
#include "utils_math.h"
#include "utils_sys.h"
#include "datatypes.h"
#include "comm_can.h"
#include "terminal.h"
#include "digital_filter.h"
// #include "utils.h"


// adding this include to get servo function for tidal compensation
#include "servo_simple.h"


#include <math.h>
#include <stdio.h>

// Can
#define MAX_CAN_AGE 0.1

// Data type (Value 5 was removed, and can be reused at a later date, but i wanted to preserve the current value's numbers for UIs)
typedef enum {
	STARTUP = 0,
	RUNNING = 1,
	RUNNING_TILTBACK_DUTY = 2,
	RUNNING_TILTBACK_HIGH_VOLTAGE = 3,
	RUNNING_TILTBACK_LOW_VOLTAGE = 4,
	FAULT_ANGLE_PITCH = 6,
	FAULT_ANGLE_ROLL = 7,
	FAULT_SWITCH_HALF = 8,
	FAULT_SWITCH_FULL = 9,
	FAULT_DUTY = 10,
	FAULT_STARTUP = 11
} BalanceState;

typedef enum {
	CENTERING = 0,
	TILTBACK_DUTY,
	TILTBACK_HV,
	TILTBACK_LV,
	TILTBACK_NONE
} SetpointAdjustmentType;

typedef enum {
	OFF = 0,
	HALF,
	ON
} SwitchState;

// Balance thread
static THD_FUNCTION(balance_thread, arg);
static THD_WORKING_AREA(balance_thread_wa, 1024); // 2kb stack for this thread

static thread_t *app_thread;

// Config values
static volatile balance_config balance_conf;
static volatile imu_config imu_conf;
static systime_t loop_time;
static float startup_step_size;
static float tiltback_duty_step_size, tiltback_hv_step_size, tiltback_lv_step_size, tiltback_return_step_size;
static float torquetilt_on_step_size, torquetilt_off_step_size, turntilt_step_size;
static float tiltback_variable, tiltback_variable_max_erpm, noseangling_step_size;

// Runtime values read from elsewhere
static float pitch_angle, last_pitch_angle, roll_angle, abs_roll_angle, abs_roll_angle_sin, last_gyro_y;
static float gyro[3];
static float duty_cycle, abs_duty_cycle;
static float erpm, abs_erpm, avg_erpm;
static float motor_current;
static float motor_position;
static float adc1, adc2;
static SwitchState switch_state;
static float app_nunchuk_get_decoded_y(void);
static float ReadSensorTimeStamp;//maybe this should be a uint32_t
static float TidalCompSetpoint;
static float TidalCompSetpointPulses;
static float TidalCompSetpointTime;
static float ServoOut;

// Rumtime state values
static BalanceState state;
static float proportional, integral, derivative, proportional2, integral2, derivative2;
static float last_proportional, abs_proportional;
static float pid_value;
static float setpoint, setpoint_target, setpoint_target_interpolated;
static float noseangling_interpolated;
static float torquetilt_filtered_current, torquetilt_target, torquetilt_interpolated;
static Biquad torquetilt_current_biquad;
static float turntilt_target, turntilt_interpolated;
static SetpointAdjustmentType setpointAdjustmentType;
static float yaw_proportional, yaw_integral, yaw_derivative, yaw_last_proportional, yaw_pid_value, yaw_setpoint;
static systime_t current_time, last_time, diff_time, loop_overshoot;
static float filtered_loop_overshoot, loop_overshoot_alpha, filtered_diff_time;
static systime_t fault_angle_pitch_timer, fault_angle_roll_timer, fault_switch_timer, fault_switch_half_timer, fault_duty_timer;
static float d_pt1_lowpass_state, d_pt1_lowpass_k, d_pt1_highpass_state, d_pt1_highpass_k;
static float motor_timeout;
static systime_t brake_timeout;

// Debug values
static int debug_render_1, debug_render_2;
static int debug_sample_field, debug_sample_count, debug_sample_index;
static int debug_experiment_1, debug_experiment_2, debug_experiment_3, debug_experiment_4, debug_experiment_5, debug_experiment_6;

// Function Prototypes
static void set_current(float current, float yaw_current);
static void terminal_render(int argc, const char **argv);
static void terminal_sample(int argc, const char **argv);
static void terminal_experiment(int argc, const char **argv);
static float app_balance_get_debug(int index);
static void app_balance_sample_debug(void);
static void app_balance_experiment(void);

// Exposed Functions
void app_balance_configure(balance_config *conf, imu_config *conf2) {
	balance_conf = *conf;
	imu_conf = *conf2;
	// Set calculated values from config
	loop_time = US2ST((int)((1000.0 / balance_conf.hertz) * 1000.0));

	motor_timeout = ((1000.0 / balance_conf.hertz)/1000.0) * 20; // Times 20 for a nice long grace period

	startup_step_size = balance_conf.startup_speed / balance_conf.hertz;
	tiltback_duty_step_size = balance_conf.tiltback_duty_speed / balance_conf.hertz;
	tiltback_hv_step_size = balance_conf.tiltback_hv_speed / balance_conf.hertz;
	tiltback_lv_step_size = balance_conf.tiltback_lv_speed / balance_conf.hertz;
	tiltback_return_step_size = balance_conf.tiltback_return_speed / balance_conf.hertz;
	torquetilt_on_step_size = balance_conf.torquetilt_on_speed / balance_conf.hertz;
	torquetilt_off_step_size = balance_conf.torquetilt_off_speed / balance_conf.hertz;
	turntilt_step_size = balance_conf.turntilt_speed / balance_conf.hertz;
	noseangling_step_size = balance_conf.noseangling_speed / balance_conf.hertz;

	// Init Filters
	if(balance_conf.loop_time_filter > 0){
		loop_overshoot_alpha = 2*M_PI*((float)1/balance_conf.hertz)*balance_conf.loop_time_filter/(2*M_PI*((float)1/balance_conf.hertz)*balance_conf.loop_time_filter+1);
	}
	if(balance_conf.kd_pt1_lowpass_frequency > 0){
		float dT = 1.0 / balance_conf.hertz;
		float RC = 1.0 / ( 2.0 * M_PI * balance_conf.kd_pt1_lowpass_frequency);
		d_pt1_lowpass_k =  dT / (RC + dT);
	}
	if(balance_conf.kd_pt1_highpass_frequency > 0){
		float dT = 1.0 / balance_conf.hertz;
		float RC = 1.0 / ( 2.0 * M_PI * balance_conf.kd_pt1_highpass_frequency);
		d_pt1_highpass_k =  dT / (RC + dT);
	}
	if(balance_conf.torquetilt_filter > 0){ // Torquetilt Current Biquad
		float Fc = balance_conf.torquetilt_filter / balance_conf.hertz;
		biquad_config(&torquetilt_current_biquad, BQ_LOWPASS, Fc);
	}

	// Variable nose angle adjustment / tiltback (setting is per 1000erpm, convert to per erpm)
	tiltback_variable = balance_conf.tiltback_variable / 1000;
	if (tiltback_variable > 0) {
		tiltback_variable_max_erpm = fabsf(balance_conf.tiltback_variable_max / tiltback_variable);
	} else {
		tiltback_variable_max_erpm = 100000;
	}

	// Reset loop time variables
	last_time = 0;
	filtered_loop_overshoot = 0;
	
}

void app_balance_start(void) {
	// First start only, override state to startup
	state = STARTUP;
	// Register terminal commands
	terminal_register_command_callback(
		"app_balance_render",
		"Render debug values on the balance real time data graph",
		"[Field Number] [Plot (Optional 1 or 2)]",
		terminal_render);
	terminal_register_command_callback(
		"app_balance_sample",
		"Output real time values to the terminal",
		"[Field Number] [Sample Count]",
		terminal_sample);
	terminal_register_command_callback(
		"app_balance_experiment",
		"Output real time values to the experiments graph",
		"[Field Number] [Plot 1-6]",
		terminal_experiment);
	// Start the balance thread
	app_thread = chThdCreateStatic(balance_thread_wa, sizeof(balance_thread_wa), NORMALPRIO, balance_thread, NULL);
}


void app_balance_stop(void) {
	if(app_thread != NULL){
		chThdTerminate(app_thread);
		chThdWait(app_thread);
	}
	set_current(0, 0);
	terminal_unregister_callback(terminal_render);
	terminal_unregister_callback(terminal_sample);
}

float app_balance_get_pid_output(void) {
	return pid_value;
}
float app_balance_get_pitch_angle(void) {
	return pitch_angle;

}
float app_balance_get_roll_angle(void) {
	return roll_angle;
}
uint32_t app_balance_get_diff_time(void) {
	return ST2US(diff_time);
}
float app_balance_get_motor_current(void) {
	return motor_current;
}
uint16_t app_balance_get_state(void) {
	return state;
}
uint16_t app_balance_get_switch_state(void) {
	return switch_state;
}
float app_balance_get_adc1(void) {
	return adc1;
}
float app_balance_get_adc2(void) {
	return adc2;
}
float app_balance_get_debug1(void) {
	return app_balance_get_debug(debug_render_1);
}
float app_balance_get_debug2(void) {
	return app_balance_get_debug(debug_render_2);
}

// adding PTO Position Buffer. This will be used to determine average PTO position and then use 
//that to determine the amount for the tidal compensation to move the PTO position to .
// Declare the array outside main
#define PTO_POS_BUFFER_1_SIZE 1000
#define PTO_POS_BUFFER_2_SIZE 10*60
#define PTO_POS_BUFFER_3_SIZE 1
float  PTOPos1Buffer[PTO_POS_BUFFER_1_SIZE];
float  PTOPos2Buffer[PTO_POS_BUFFER_2_SIZE];
float  PTOPos3Buffer[PTO_POS_BUFFER_3_SIZE];

static float PTOPos1Averaged =0;
static float PTOPos2Averaged =0;
static float PTOPos3Averaged =0;

int PTOPos1BufferIndex = 0;
int PTOPos2BufferIndex = 0;
int PTOPos3BufferIndex = 0;


// Function to apply a simple moving average filter

	static float simpleMovingAverage(float *buffer, int length) {
    float sum = 0;
    for (int i = 0; i < length; i++) {
        sum += buffer[i];
    }
    return sum / length;
}


#define WINDOW_SIZE 3 // Size of the filtering window (odd number)
#define POLY_DEGREE 2 // Degree of the fitting polynomial

// static void apply_kalman_kp(data *d){
// 	// Apply a broad Kalman filter to d->proportional
// 	float Q1 = .0042; //.005 //0.022;			//the noise in the system
// 	float R1 = .7; //0.617;					//the noise in the system
// 	float K1;
// 	float P1;
// 	float P_temp1;
// 	float x_temp_est1;
// 	float x_est1;
// 	float z_measured1; //the 'noisy' value we measured	
// 	x_temp_est1 = d->x_est_last1; 			//do a prediction
// 	P_temp1 = d->P_last1 + Q1;
// 	K1 = P_temp1 * (1.0/(P_temp1 + R1));		//calculate the Kalman gain
// 	z_measured1 = d->proportional;					//measure
// 	x_est1 = x_temp_est1 + K1 * (z_measured1 - x_temp_est1); 	//correct
// 	P1 = (1- K1) * P_temp1;
// 	d->P_last1 = P1;			//update our last's
// 	d->x_est_last1 = x_est1;		//update our last's
	
// 	//Apply a more precise filter for pitch velocity
// 	float Q2 = .08; //.08 //.1 //0.022;			//the noise in the system
// 	float R2 = 1; //0.617;	//the noise in the system
// 	float K2;
// 	float P2;
// 	float P_temp2;
// 	float x_temp_est2;
// 	float x_est2;
// 	float z_measured2; //the 'noisy' value we measured	
// 	x_temp_est2 = d->x_est_last2; 			//do a prediction
// 	P_temp2 = d->P_last2 + Q2;
// 	K2 = P_temp2 * (1.0/(P_temp2 + R2));		//calculate the Kalman gain
// 	z_measured2 = d->proportional - d->last_proportional;		//measure
// 	x_est2 = x_temp_est2 + K2 * (z_measured2 - x_temp_est2); 	//correct
// 	P2 = (1- K2) * P_temp2;
// 	d->P_last2 = P2;			//update our last's
// 	d->x_est_last2 = x_est2;		//update our last's			

// 	d->prop_smooth = x_est1 + x_est2 * 0.9;	//smoothed d->proportional, plus an estimate for the next step based on smoothed pitch speed	






// Internal Functions
static void reset_vars(void){
	// Clear accumulated values.
	integral = 0;
	last_proportional = 0;
	integral2 = 0;
	yaw_integral = 0;
	yaw_last_proportional = 0;
	d_pt1_lowpass_state = 0;
	d_pt1_highpass_state = 0;
	// Set values for startup
	setpoint = pitch_angle;
	setpoint_target_interpolated = pitch_angle;
	setpoint_target = 0;
	noseangling_interpolated = 0;
	torquetilt_target = 0;
	torquetilt_interpolated = 0;
	torquetilt_filtered_current = 0;
	biquad_reset(&torquetilt_current_biquad);
	turntilt_target = 0;
	turntilt_interpolated = 0;
	setpointAdjustmentType = CENTERING;
	yaw_setpoint = 0;
	state = RUNNING;
	current_time = 0;
	last_time = 0;
	diff_time = 0;
	brake_timeout = 0;
	//app_nunchuk_get_decoded_y()=0;
}

static float get_setpoint_adjustment_step_size(void){
	switch(setpointAdjustmentType){
		case (CENTERING):
			return startup_step_size;
		case (TILTBACK_DUTY):
			return tiltback_duty_step_size;
		case (TILTBACK_HV):
			return tiltback_hv_step_size;
		case (TILTBACK_LV):
			return tiltback_lv_step_size;
		case (TILTBACK_NONE):
			return tiltback_return_step_size;
		default:
			;
	}
	return 0;
}

// Fault checking order does not really matter. From a UX perspective, switch should be before angle.
static bool check_faults(bool ignoreTimers){
	// Check switch
	// Switch fully open
	if(switch_state == OFF){
		if(ST2MS(current_time - fault_switch_timer) > balance_conf.fault_delay_switch_full || ignoreTimers){
			state = FAULT_SWITCH_FULL;
			return true;
		}
	} else {
		fault_switch_timer = current_time;
	}

	// Switch partially open and stopped
	if(!balance_conf.fault_is_dual_switch) {
		if((switch_state == HALF || switch_state == OFF) && abs_erpm < balance_conf.fault_adc_half_erpm){
			if(ST2MS(current_time - fault_switch_half_timer) > balance_conf.fault_delay_switch_half || ignoreTimers){
				state = FAULT_SWITCH_HALF;
				return true;
			}
		} else {
			fault_switch_half_timer = current_time;
		}
	}

	// Check pitch angle
	if(fabsf(pitch_angle) > balance_conf.fault_pitch){
		if(ST2MS(current_time - fault_angle_pitch_timer) > balance_conf.fault_delay_pitch || ignoreTimers){
			state = FAULT_ANGLE_PITCH;
			return true;
		}
	}else{
		fault_angle_pitch_timer = current_time;
	}

	// Check roll angle
	if(fabsf(roll_angle) > balance_conf.fault_roll){
		if(ST2MS(current_time - fault_angle_roll_timer) > balance_conf.fault_delay_roll || ignoreTimers){
			state = FAULT_ANGLE_ROLL;
			return true;
		}
	}else{
		fault_angle_roll_timer = current_time;
	}

	// Check for duty
	if(abs_duty_cycle > balance_conf.fault_duty){
		if(ST2MS(current_time - fault_duty_timer) > balance_conf.fault_delay_duty || ignoreTimers){
			state = FAULT_DUTY;
			return true;
		}
	} else {
		fault_duty_timer = current_time;
	}

	return false;
}

static void calculate_setpoint_target(void){
	if(setpointAdjustmentType == CENTERING && setpoint_target_interpolated != setpoint_target){
		// Ignore tiltback during centering sequence
		state = RUNNING;
	}else if(abs_duty_cycle > balance_conf.tiltback_duty){
		if(erpm > 0){
			setpoint_target = balance_conf.tiltback_duty_angle;
		} else {
			setpoint_target = -balance_conf.tiltback_duty_angle;
		}
		setpointAdjustmentType = TILTBACK_DUTY;
		state = RUNNING_TILTBACK_DUTY;
	}else if(abs_duty_cycle > 0.05 && GET_INPUT_VOLTAGE() > balance_conf.tiltback_hv){
		if(erpm > 0){
			setpoint_target = balance_conf.tiltback_hv_angle;
		} else {
			setpoint_target = -balance_conf.tiltback_hv_angle;
		}
		setpointAdjustmentType = TILTBACK_HV;
		state = RUNNING_TILTBACK_HIGH_VOLTAGE;
	}else if(abs_duty_cycle > 0.05 && GET_INPUT_VOLTAGE() < balance_conf.tiltback_lv){
		if(erpm > 0){
			setpoint_target = balance_conf.tiltback_lv_angle;
		} else {
			setpoint_target = -balance_conf.tiltback_lv_angle;
		}
		setpointAdjustmentType = TILTBACK_LV;
		state = RUNNING_TILTBACK_LOW_VOLTAGE;
	}else{
		setpointAdjustmentType = TILTBACK_NONE;
		setpoint_target = 0;
		state = RUNNING;
	}
}

static void calculate_setpoint_interpolated(void){
	if(setpoint_target_interpolated != setpoint_target){
		// If we are less than one step size away, go all the way
		if(fabsf(setpoint_target - setpoint_target_interpolated) < get_setpoint_adjustment_step_size()){
			setpoint_target_interpolated = setpoint_target;
		}else if (setpoint_target - setpoint_target_interpolated > 0){
			setpoint_target_interpolated += get_setpoint_adjustment_step_size();
		}else{
			setpoint_target_interpolated -= get_setpoint_adjustment_step_size();
		}
	}
}

static void apply_noseangling(void){
	// Nose angle adjustment, add variable then constant tiltback
	float noseangling_target = 0;
	if (fabsf(erpm) > tiltback_variable_max_erpm) {
		noseangling_target = fabsf(balance_conf.tiltback_variable_max) * SIGN(erpm);
	} else {
		noseangling_target = tiltback_variable * erpm;
	}

	if(erpm > balance_conf.tiltback_constant_erpm){
		noseangling_target += balance_conf.tiltback_constant;
	} else if(erpm < -balance_conf.tiltback_constant_erpm){
		noseangling_target += -balance_conf.tiltback_constant;
	}

	if(fabsf(noseangling_target - noseangling_interpolated) < noseangling_step_size){
		noseangling_interpolated = noseangling_target;
	}else if (noseangling_target - noseangling_interpolated > 0){
		noseangling_interpolated += noseangling_step_size;
	}else{
		noseangling_interpolated -= noseangling_step_size;
	}
	setpoint += noseangling_interpolated;
}

static void apply_torquetilt(void){
	// Filter current (Biquad)
	if(balance_conf.torquetilt_filter > 0){
		torquetilt_filtered_current = biquad_process(&torquetilt_current_biquad, motor_current);
	}else{
		torquetilt_filtered_current  = motor_current;
	}


	// Wat is this line O_o
	// Take abs motor current, subtract start offset, and take the max of that with 0 to get the current above our start threshold (absolute).
	// Then multiply it by "power" to get our desired angle, and min with the limit to respect boundaries.
	// Finally multiply it by sign motor current to get directionality back
	torquetilt_target = fminf(fmaxf((fabsf(torquetilt_filtered_current) - balance_conf.torquetilt_start_current), 0) * balance_conf.torquetilt_strength, balance_conf.torquetilt_angle_limit) * SIGN(torquetilt_filtered_current);

	float step_size;
	if((torquetilt_interpolated - torquetilt_target > 0 && torquetilt_target > 0) || (torquetilt_interpolated - torquetilt_target < 0 && torquetilt_target < 0)){
		step_size = torquetilt_off_step_size;
	}else{
		step_size = torquetilt_on_step_size;
	}

	if(fabsf(torquetilt_target - torquetilt_interpolated) < step_size){
		torquetilt_interpolated = torquetilt_target;
	}else if (torquetilt_target - torquetilt_interpolated > 0){
		torquetilt_interpolated += step_size;
	}else{
		torquetilt_interpolated -= step_size;
	}
	setpoint += torquetilt_interpolated;
}

static void apply_turntilt(void){
	// Calculate desired angle
	turntilt_target = abs_roll_angle_sin * balance_conf.turntilt_strength;

	// Apply cutzone
	if(abs_roll_angle < balance_conf.turntilt_start_angle){
		turntilt_target = 0;
	}

	// Disable below erpm threshold otherwise add directionality
	if(abs_erpm < balance_conf.turntilt_start_erpm){
		turntilt_target = 0;
	}else {
		turntilt_target *= SIGN(erpm);
	}

	// Apply speed scaling
	if(abs_erpm < balance_conf.turntilt_erpm_boost_end){
		turntilt_target *= 1 + ((balance_conf.turntilt_erpm_boost/100.0f) * (abs_erpm / balance_conf.turntilt_erpm_boost_end));
	}else{
		turntilt_target *= 1 + (balance_conf.turntilt_erpm_boost/100.0f);
	}

	// Limit angle to max angle
	if(turntilt_target > 0){
		turntilt_target = fminf(turntilt_target, balance_conf.turntilt_angle_limit);
	}else{
		turntilt_target = fmaxf(turntilt_target, -balance_conf.turntilt_angle_limit);
	}

	// Move towards target limited by max speed
	if(fabsf(turntilt_target - turntilt_interpolated) < turntilt_step_size){
		turntilt_interpolated = turntilt_target;
	}else if (turntilt_target - turntilt_interpolated > 0){
		turntilt_interpolated += turntilt_step_size;
	}else{
		turntilt_interpolated -= turntilt_step_size;
	}
	setpoint += turntilt_interpolated;

}

static float apply_deadzone(float error){
	if(balance_conf.deadzone == 0){
		return error;
	}

	if(error < balance_conf.deadzone && error > -balance_conf.deadzone){
		return 0;
	} else if(error > balance_conf.deadzone){
		return error - balance_conf.deadzone;
	} else {
		return error + balance_conf.deadzone;
	}
}

static void brake(void){
	// Brake timeout logic
	if(balance_conf.brake_timeout > 0 && (abs_erpm > 1 || brake_timeout == 0)){
		brake_timeout = current_time + S2ST(balance_conf.brake_timeout);
	}
	if(brake_timeout != 0 && current_time > brake_timeout){
		return;
	}

	// Reset the timeout
	timeout_reset();
	// Set current
	mc_interface_set_brake_current(balance_conf.brake_current);
	if(balance_conf.multi_esc){
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg *msg = comm_can_get_status_msg_index(i);
			if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
				comm_can_set_current_brake(msg->id, balance_conf.brake_current);
			}
		}
	}
}

static void set_current(float current, float yaw_current){
	// Limit current output to configured max output (does not account for yaw_current)
	if(current > 0 && current > mc_interface_get_configuration()->l_current_max){
		current = mc_interface_get_configuration()->l_current_max;
	}else if(current < 0 && current < mc_interface_get_configuration()->l_current_min){
		current = mc_interface_get_configuration()->l_current_min;
	}
	i
	// Reset the timeout
	timeout_reset();
	// Set current
	if(balance_conf.multi_esc){
		// Set the current delay
		mc_interface_set_current_off_delay(motor_timeout);
		// Set Current
		mc_interface_set_current(current + yaw_current);
		// Can bus
		for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			can_status_msg *msg = comm_can_get_status_msg_index(i);
			

			if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
				comm_can_set_current_off_delay(msg->id, current - yaw_current, motor_timeout);// Assume 2 motors, i don't know how to steer 3 anyways
			}
		}
	} else {
		// Set the current delay
		mc_interface_set_current_off_delay(motor_timeout);
		// Set Current
		mc_interface_set_current(current);
	}
}

static THD_FUNCTION(balance_thread, arg) {
	(void)arg;
	chRegSetThreadName("APP_BALANCE");

	while (!chThdShouldTerminateX()) {
		// Update times
		current_time = chVTGetSystemTimeX();


		float dt = UTILS_AGE_S(ReadSensorTimeStamp); //time elapsed since last sensor read, in seconds.
		//dt_check=dt;
		ReadSensorTimeStamp = chVTGetSystemTimeX();

		if(last_time == 0){
		  last_time = current_time;
		}
		diff_time = current_time - last_time;
		filtered_diff_time = 0.03 * diff_time + 0.97 * filtered_diff_time; // Purely a metric
		last_time = current_time;
		if(balance_conf.loop_time_filter > 0){
			loop_overshoot = diff_time - (loop_time - roundf(filtered_loop_overshoot));
			filtered_loop_overshoot = loop_overshoot_alpha * loop_overshoot + (1-loop_overshoot_alpha)*filtered_loop_overshoot;
		}
		
		// Function to apply a simple moving average filter
		//Apply scaling to ADC1. For now 0.89V and 3.3V is the full scale range. Range is 2.41 volts.
		//This equates to 30 degrees. Therefore, 1V is 12.44 degrees. For the midrange to be zero degrees, we must offset the reading. 
		// Midrange is 2.41/2=1.205V+.89=2.095V, which is zero degrees. (2.095-adc1)*12.44=PTO angle in degrees.

		//PTOPos1Buffer[PTOPos1BufferIndex] =(2.095-adc1)*12.44;

		PTOPos1Averaged=(2.095-adc1)*12.44;
		PTOPos1BufferIndex = (PTOPos1BufferIndex + 1) % PTO_POS_BUFFER_1_SIZE;
		// PTOPos1Averaged=simpleMovingAverage( PTOPos1Buffer ,PTO_POS_BUFFER_1_SIZE);


		// PTOPos2Buffer[PTOPos2BufferIndex] =PTOPos1Averaged;
		// PTOPos2BufferIndex = (PTOPos2BufferIndex + 1) % PTO_POS_BUFFER_2_SIZE;
		// PTOPos2Averaged=simpleMovingAverage( PTOPos2Buffer ,PTO_POS_BUFFER_2_SIZE);
		

		// PTOPos3Buffer[PTOPos3BufferIndex] =PTOPos2Averaged;
		// PTOPos3BufferIndex = (PTOPos3BufferIndex + 1) % PTO_POS_BUFFER_3_SIZE;
		// PTOPos3Averaged=simpleMovingAverage( PTOPos3Buffer ,PTO_POS_BUFFER_3_SIZE);

		PTOPos2Buffer[PTOPos2BufferIndex]=UTILS_LP_MOVING_AVG_APPROX(PTOPos2Averaged, PTOPos1Averaged, PTO_POS_BUFFER_1_SIZE);


		//PTOPos2Buffer[PTOPos2BufferIndex] =PTOPos1Averaged;
		PTOPos2BufferIndex = (PTOPos2BufferIndex + 1) % PTO_POS_BUFFER_2_SIZE;
		PTOPos3Averaged=simpleMovingAverage( PTOPos2Buffer,PTO_POS_BUFFER_2_SIZE);

		// //Calculate the rolling average 
		// 		if (PTOPos2BufferIndex % 60 == 0) {
		// 			PTOPos2BufferIndex = PTOPos2BufferIndex / 60;
		// 			PTOPos3Buffer[PTOPos3BufferIndex] = simpleMovingAverage( PTOPos2Buffer ,PTO_POS_BUFFER_2_SIZE);
		// 		}

		
		

		



		
		
		
		//PTOPos2Averaged=simpleMovingAverage( avgDataBuffer ,PTO_POS_BUFFER_2_SIZE);

		TidalCompSetpoint=PTOPos3Averaged;//average angular position in degrees.







		//#define SAMPLE_RATE_HZ 1000
		// #define ROLLING_INTERVAL 1  // Rolling average interval in seconds

		// // Define buffer sizes based on your memory constraints
		// //#define RAW_DATA_BUFFER_SIZE (SAMPLE_RATE_HZ * ROLLING_INTERVAL)
		// //#define AVG_DATA_BUFFER_SIZE (20 * 60)  // 20 minutes at 1 sample/second

		// float rawDataBuffer[PTO_POS_BUFFER_1_SIZE];
		// float avgDataBuffer[PTO_POS_BUFFER_2_SIZE];
		// int rawDataIndex = 0;
		
		// // Function to calculate rolling average
		// float calculateRollingAverage(float* buffer, int bufferSize) {
		// 	float sum = 0;
		// 	for (int i = 0; i < bufferSize; i++) {
		// 		sum += buffer[i];
		// 	}
		// 	return sum / bufferSize;
		// }
		// 		float newData =(2.095-adc1)*12.44;

		// 		// Store the new data point in the raw data buffer
		// 		rawDataBuffer[rawDataIndex] = newData;
		// 		rawDataIndex = (rawDataIndex + 1) % PTO_POS_BUFFER_1_SIZE;

		// 		// Calculate the rolling average every second
		// 		//if (rawDataIndex % 1000 == 0) {
		// 			int avgDataIndex = rawDataIndex / 1000;
		// 			avgDataBuffer[avgDataIndex] = calculateRollingAverage(rawDataBuffer, PTO_POS_BUFFER_1_SIZE);
		// 		//}




				







		//servo set for 200 pulses per revolution. 
		//Setpoint drives frequency. 12 kHz would be 1200 rev/min or 20 rev/sec. 200 pulses/rev * 20 rev/sec = 4000 pulses/sec.

		//We want to move a set number of pulses. Lets say we do it slowly for now, so fix the frequency to something smallish.
		//Lets say move 100 rev/min. This would be 1.67 rev/sec. 200 pulses/rev * 1.67 rev/sec = 334 pulses/sec=334Hz.

		//Servo scaling is set from zero to one input, with 4000 pulses/sec being the max. So, 334/4000=0.0835.

		//One revolution = 360 degrees, so 1 degree = 1/360=0.0027778 rev. 0.0027778*200 pulses/rev=0.5556 pulses/degree.
		// The TidalCompSetpoint should be in degrees, so multiply by 0.5556 to get pulses. Then multiply by 0.0835 to get servo input.

		//There is a gear ratio involved with the worm drive. balance_conf.ki2 will be repuposed to be the gear ratio. 
		//This could be say, 100:1. For now we assume 1:1.

		// move_steppermotor( Direction, Speed, Time)
		// 11:41
		// or it could be (Direction, Speed, Distance)
		// and then a final layer,
		// take long running average linear position
		// on a very long interval, if average position is not 0, move stepper motor equal to (or opposite?) to the average

		TidalCompSetpointPulses=round(TidalCompSetpoint*0.5556*balance_conf.ki2);
		float TidalCompSetpointFrequency=334.0;//frequency in Hz to move the setpoint.
		TidalCompSetpointTime=TidalCompSetpointPulses/TidalCompSetpointFrequency;//time in seconds to move the setpoint.

		// if (app_nunchuk_get_decoded_y()>.05) {
		// 	servo_simple_set_output((app_nunchuk_get_decoded_y()*balance_conf.ki2))
		// 	}else if (app_nunchuk_get_decoded_y()<.05) {
		// 	if (TidalCompSetpointTime > 0) {
		// 	(ReadSensorTimeStamp+TidalCompSetpointTime<ReadSensorTimeStamp) 
		// 	servo_simple_set_output((TidalCompSetpointFrequency/4000)*balance_conf.ki2);
		// 	} else {
		// 	servo_simple_set_output(0);
		// 	}
		// }
		

			// if(fabsf(TidalCompSetpointPulses) < turntilt_step_size){
			// turntilt_interpolated = turntilt_target;
			// }else if (turntilt_target - turntilt_interpolated > 0){
			// 	turntilt_interpolated += turntilt_step_size;
			// }else{
			// 	turntilt_interpolated -= turntilt_step_size;
			// }
			// setpoint += turntilt_interpolated;


		// if (app_nunchuk_get_decoded_y() > 0.05) {
		// servo_simple_set_output(app_nunchuk_get_decoded_y() * balance_conf.ki2);
		// 	if (TidalCompSetpointTime > 0) {
		// 		if (ReadSensorTimeStamp + TidalCompSetpointTime < ReadSensorTimeStamp) {
		// 			servo_simple_set_output((TidalCompSetpointFrequency / 4000) * balance_conf.ki2);
		// 		} else {
		// 			servo_simple_set_output(0);
		// 		}
		// 	}
		// }



		// 	if (TidalCompSetpointTime > 0) {
		// 	void moveMotorForTime(uint32_t duration_s) {
		// 	uint32_t startTime = ReadSensorTimeStamp;
		// 	servo_simple_set_output((TidalCompSetpointFrequency / 4000) * balance_conf.ki2);
		// 	// Calculate the elapsed time
		// 	uint32_t currentTime = ReadSensorTimeStamp;
		// 	uint32_t elapsedTime = currentTime - startTime;
				
		// 		// Check if the desired duration has passed
		// 		if (elapsedTime >= duration_ms) {
		// 			// Stop the motor 
		// 			servo_simple_set_output(0);
					
		// 		}
			
		// }
			if (TidalCompSetpointTime > 0.1) {
			uint32_t startTime = ReadSensorTimeStamp;
			servo_simple_set_output((TidalCompSetpointFrequency / 4000) * balance_conf.ki2);
			// Calculate the elapsed time
			uint32_t currentTime = ReadSensorTimeStamp;
			uint32_t elapsedTime = currentTime - startTime;
				// Check if the desired duration has passed
				if (elapsedTime >= TidalCompSetpointTime) {
					// Stop the motor 
					servo_simple_set_output(0);
				}
				//ServoOut=servo_simple_set_output(void);
		}

		// Call the moveMotorForTime function with the desired duration
			//moveMotorForTime(TidalCompSetpointTime);
		
		
			// // Call the moveMotorForTime function with the desired duration
			// moveMotorForTime(TidalCompSetpointTime);
			
			

			
		//}

		// if(balance_conf.fault_adc1 == 0 && balance_conf.fault_adc2 == 0){ // No Switch
		// 	switch_state = ON;
		// }else if(balance_conf.fault_adc2 == 0){ // Single switch on ADC1
		// 	if(adc1 > balance_conf.fault_adc1){
		// 		switch_state = ON;
		// 	} else {
		// 		switch_state = OFF;
		// 	}

		//servo_simple_set_output((app_nunchuk_get_decoded_y()*balance_conf.ki2));


//     // Call the moveServo function to move the servo
//     moveServo(desiredAngle, speedPPM);

//     return 0;
// }	








		

		
		// Read values for GUI
		motor_current = mc_interface_get_tot_current_directional_filtered();
		motor_position = mc_interface_get_pid_pos_now();

		// Set "last" values to previous loops values
		last_pitch_angle = pitch_angle;
		last_gyro_y = gyro[1];
		// Get the values we want
		pitch_angle = RAD2DEG_f(imu_get_pitch());
		roll_angle = RAD2DEG_f(imu_get_roll());
		abs_roll_angle = fabsf(roll_angle);
		abs_roll_angle_sin = sinf(DEG2RAD_f(abs_roll_angle));
		imu_get_gyro(gyro);
		duty_cycle = mc_interface_get_duty_cycle_now();
		abs_duty_cycle = fabsf(duty_cycle);
		erpm = mc_interface_get_rpm();
		abs_erpm = fabsf(erpm);
		if(balance_conf.multi_esc){
			avg_erpm = erpm;
			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg *msg = comm_can_get_status_msg_index(i);
				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
					avg_erpm += msg->rpm;
				}
			}
			avg_erpm = avg_erpm/2;// Assume 2 motors, i don't know how to steer 3 anyways
		}
		adc1 = (((float)ADC_Value[ADC_IND_EXT])/4095) * V_REG;
#ifdef ADC_IND_EXT2
		adc2 = (((float)ADC_Value[ADC_IND_EXT2])/4095) * V_REG;
#else
		adc2 = 0.0;
#endif

		// Calculate switch state from ADC values
		if(balance_conf.fault_adc1 == 0 && balance_conf.fault_adc2 == 0){ // No Switch
			switch_state = ON;
		}else if(balance_conf.fault_adc2 == 0){ // Single switch on ADC1
			if(adc1 > balance_conf.fault_adc1){
				switch_state = ON;
			} else {
				switch_state = OFF;
			}
		}else if(balance_conf.fault_adc1 == 0){ // Single switch on ADC2
			if(adc2 > balance_conf.fault_adc2){
				switch_state = ON;
			} else {
				switch_state = OFF;
			}
		}else{ // Double switch
			if(adc1 > balance_conf.fault_adc1 && adc2 > balance_conf.fault_adc2){
				switch_state = ON;
			}else if(adc1 > balance_conf.fault_adc1 || adc2 > balance_conf.fault_adc2){
				if (balance_conf.fault_is_dual_switch)
					switch_state = ON;
				else
					switch_state = HALF;
			}else{
				switch_state = OFF;
			}
		}


		// Control Loop State Logic
		switch(state){
			case (STARTUP):
				// Disable output
				// brake();
				// if(imu_startup_done()){
				// 	reset_vars();
				// 	state = FAULT_STARTUP; // Trigger a fault so we need to meet start conditions to start
				// }
				break;
			case (RUNNING):
			// case (RUNNING_TILTBACK_DUTY):
			// case (RUNNING_TILTBACK_HIGH_VOLTAGE):
			// case (RUNNING_TILTBACK_LOW_VOLTAGE):

				// // Check for faults
				// if(check_faults(false)){
				// 	break;
				// }

				// Calculate setpoint and interpolation
				calculate_setpoint_target();
				calculate_setpoint_interpolated();
				setpoint = setpoint_target_interpolated;
				apply_noseangling();
				apply_torquetilt();
				apply_turntilt();

				// Do PID maths
				proportional = setpoint - pitch_angle;
				// Apply deadzone
				proportional = apply_deadzone(proportional);
				// Resume real PID maths
				integral = integral + proportional;
				derivative = last_pitch_angle - pitch_angle;

				// Apply I term Filter
				if(balance_conf.ki_limit > 0 && fabsf(integral * balance_conf.ki) > balance_conf.ki_limit){
					integral = balance_conf.ki_limit / balance_conf.ki * SIGN(integral);
				}

				// Apply D term filters
				if(balance_conf.kd_pt1_lowpass_frequency > 0){
					d_pt1_lowpass_state = d_pt1_lowpass_state + d_pt1_lowpass_k * (derivative - d_pt1_lowpass_state);
					derivative = d_pt1_lowpass_state;
				}
				if(balance_conf.kd_pt1_highpass_frequency > 0){
					d_pt1_highpass_state = d_pt1_highpass_state + d_pt1_highpass_k * (derivative - d_pt1_highpass_state);
					derivative = derivative - d_pt1_highpass_state;
				}

				pid_value = (balance_conf.kp * proportional) + (balance_conf.ki * integral) + (balance_conf.kd * derivative);

				if(balance_conf.pid_mode == BALANCE_PID_MODE_ANGLE_RATE_CASCADE){
					proportional2 = pid_value - gyro[1];
					integral2 = integral2 + proportional2;
					derivative2 = last_gyro_y - gyro[1];

					// Apply I term Filter
					if(balance_conf.ki_limit > 0 && fabsf(integral2 * balance_conf.ki2) > balance_conf.ki_limit){
						integral2 = balance_conf.ki_limit / balance_conf.ki2 * SIGN(integral2);
					}

					pid_value = (balance_conf.kp2 * proportional2) + (balance_conf.ki2 * integral2) + (balance_conf.kd2 * derivative2);
				}

				last_proportional = proportional;

				// Apply Booster
				abs_proportional = fabsf(proportional);
				if(abs_proportional > balance_conf.booster_angle){
					if(abs_proportional - balance_conf.booster_angle < balance_conf.booster_ramp){
						pid_value += (balance_conf.booster_current * SIGN(proportional)) * ((abs_proportional - balance_conf.booster_angle) / balance_conf.booster_ramp);
					}else{
						pid_value += balance_conf.booster_current * SIGN(proportional);
					}
				}

				if(balance_conf.multi_esc){
					// Calculate setpoint
					if(abs_duty_cycle < .02){
						yaw_setpoint = 0;
					} else if(avg_erpm < 0){
						yaw_setpoint = (-balance_conf.roll_steer_kp * roll_angle) + (balance_conf.roll_steer_erpm_kp * roll_angle * avg_erpm);
					} else{
						yaw_setpoint = (balance_conf.roll_steer_kp * roll_angle) + (balance_conf.roll_steer_erpm_kp * roll_angle * avg_erpm);
					}
					// Do PID maths
					yaw_proportional = yaw_setpoint - gyro[2];
					yaw_integral = yaw_integral + yaw_proportional;
					yaw_derivative = yaw_proportional - yaw_last_proportional;

					yaw_pid_value = (balance_conf.yaw_kp * yaw_proportional) + (balance_conf.yaw_ki * yaw_integral) + (balance_conf.yaw_kd * yaw_derivative);

					if(yaw_pid_value > balance_conf.yaw_current_clamp){
						yaw_pid_value = balance_conf.yaw_current_clamp;
					}else if(yaw_pid_value < -balance_conf.yaw_current_clamp){
						yaw_pid_value = -balance_conf.yaw_current_clamp;
					}

					yaw_last_proportional = yaw_proportional;
				}

				// Output to motor
				set_current(pid_value, yaw_pid_value);
				

				//servo_simple_init();
				//servo_simple_set_output(adc1);

				break;
			case (FAULT_ANGLE_PITCH):
			case (FAULT_ANGLE_ROLL):
			case (FAULT_SWITCH_HALF):
			case (FAULT_SWITCH_FULL):
			case (FAULT_STARTUP):
				// Check for valid startup position and switch state
				if(fabsf(pitch_angle) < balance_conf.startup_pitch_tolerance && fabsf(roll_angle) < balance_conf.startup_roll_tolerance && switch_state == ON){
					reset_vars();
					break;
				}
				// Disable output
				brake();
				break;
			case (FAULT_DUTY):
				// We need another fault to clear duty fault.
				// Otherwise duty fault will clear itself as soon as motor pauses, then motor will spool up again.
				// Rendering this fault useless.
				check_faults(true);
				// Disable output
				brake();
				break;
		}

		// Debug outputs
		app_balance_sample_debug();
		app_balance_experiment();

		// Delay between loops
		chThdSleep(loop_time - roundf(filtered_loop_overshoot));
	}

	// Disable output
	brake();
}

// Terminal commands
static void terminal_render(int argc, const char **argv) {
	if (argc == 2 || argc == 3) {
		int field = 0;
		int graph = 1;
		sscanf(argv[1], "%d", &field);
		if(argc == 3){
			sscanf(argv[2], "%d", &graph);
			if(graph < 1 || graph > 2){
				graph = 1;
			}
		}
		if(graph == 1){
			debug_render_1 = field;
		}else{
			debug_render_2 = field;
		}
	} else {
		commands_printf("This command requires one or two argument(s).\n");
	}
}

static void terminal_sample(int argc, const char **argv) {
	if (argc == 3) {
		debug_sample_field = 0;
		debug_sample_count = 0;
		sscanf(argv[1], "%d", &debug_sample_field);
		sscanf(argv[2], "%d", &debug_sample_count);
		debug_sample_index = 0;
	} else {
		commands_printf("This command requires two arguments.\n");
	}
}

static void terminal_experiment(int argc, const char **argv) {
	if (argc == 3) {
		int field = 0;
		int graph = 1;
		sscanf(argv[1], "%d", &field);
		sscanf(argv[2], "%d", &graph);
		switch(graph){
			case (1):
				debug_experiment_1 = field;
				break;
			case (2):
				debug_experiment_2 = field;
				break;
			case (3):
				debug_experiment_3 = field;
				break;
			case (4):
				debug_experiment_4 = field;
				break;
			case (5):
				debug_experiment_5 = field;
				break;
			case (6):
				debug_experiment_6 = field;
				break;
		}
		commands_init_plot("Microseconds", "Balance App Debug Data");
		commands_plot_add_graph("1");
		commands_plot_add_graph("2");
		commands_plot_add_graph("3");
		commands_plot_add_graph("4");
		commands_plot_add_graph("5");
		commands_plot_add_graph("6");
	} else {
		commands_printf("This command requires two arguments.\n");
	}
}

// Debug functions
static float app_balance_get_debug(int index){
	switch(index){
		case(1):
			return ServoOut;
		case(2):
			return setpoint;
		case(3):
			return (app_nunchuk_get_decoded_y()*balance_conf.ki2);
		case(4):
			return PTOPos1Averaged;
		case(5):
			return PTOPos2Averaged;
		case(6):
			return PTOPos3Averaged;
		case(7):
			return TidalCompSetpointPulses;
		case(8):
			return TidalCompSetpointTime;
		case(9):
			return loop_time;
		case(10):
			return diff_time;
		case(11):
			return loop_overshoot;
		case(12):
			return filtered_loop_overshoot;
		case(13):
			return filtered_diff_time;
		case(14):
			return integral;
		case(15):
			return integral * balance_conf.ki;
		case(16):
			return integral2;
		case(17):
			return integral2 * balance_conf.ki2;
		default:
			return 0;
	}
}
static void app_balance_sample_debug(){
	if(debug_sample_index < debug_sample_count){
		commands_printf("%f", (double)app_balance_get_debug(debug_sample_field));
		debug_sample_index += 1;
	}
}
static void app_balance_experiment(){
	if(debug_experiment_1 != 0){
		commands_plot_set_graph(0);
		commands_send_plot_points(ST2MS(current_time), app_balance_get_debug(debug_experiment_1));
	}
	if(debug_experiment_2 != 0){
		commands_plot_set_graph(1);
		commands_send_plot_points(ST2MS(current_time), app_balance_get_debug(debug_experiment_2));
	}
	if(debug_experiment_3 != 0){
		commands_plot_set_graph(2);
		commands_send_plot_points(ST2MS(current_time), app_balance_get_debug(debug_experiment_3));
	}
	if(debug_experiment_4 != 0){
		commands_plot_set_graph(3);
		commands_send_plot_points(ST2MS(current_time), app_balance_get_debug(debug_experiment_4));
	}
	if(debug_experiment_5 != 0){
		commands_plot_set_graph(4);
		commands_send_plot_points(ST2MS(current_time), app_balance_get_debug(debug_experiment_5));
	}
	if(debug_experiment_6 != 0){
		commands_plot_set_graph(5);
		commands_send_plot_points(ST2MS(current_time), app_balance_get_debug(debug_experiment_6));
	}
}
