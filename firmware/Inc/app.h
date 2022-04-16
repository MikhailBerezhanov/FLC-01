#ifndef _APP_H
#define _APP_H

#include <stdint.h>
#include "rtc.h"

#define APP_VERSION					"1.6"					// Firmware version

#define WORK_PERIODS_NUM		1							// Supported number of work periods to serve

// Smooth power-control settings
#define MIN_SMOOTH_VALUE		((uint16_t)1)	// Main coefficient of smooth control formula MAX value
#define MAX_SMOOTH_VALUE		UINT16_MAX		// Main coefficient of smooth control formula MIN value
#define PWM_DISCRETE				1							// Power step [0.1%]

// RTC poll settings
#define RTC_POLL_PERIOD			2							// Peroid of current time polling [sec] 

typedef enum{
	APP_OK = 0,							
	APP_INVALID_DTIME_FORMAT,			// "warning" : we can process further
	APP_INVALID_TIME_FORMAT,			// "warning" : we can process further
	APP_CRITICAL_ERROR,						// Higher -  "errors" : we can't process further
	APP_JSON_PARSE_ERR,
	APP_JSON_NO_OBJ,
	APP_INVALID_DATA,	
	APP_DATA_OVERFLOW,
}app_err_t;

// This data is stored in internal FLASH, so we use words (32bits) 
typedef __packed struct {
  int32_t start_hour;		//
	int32_t start_min;		//
	int32_t start_sec;		//
	int32_t stop_hour;		//
	int32_t stop_min;			//
	int32_t stop_sec;			//
	int32_t enabled;			// Flag of active work period
}work_period_t;

typedef __packed struct{
	int32_t ch1;    	// White leds PWM %	(OnBoard CH1 -- PB0)
	int32_t ch2;			// Red leds PWM %		(OnBoard CH2 -- PA6)
	int32_t ch3;    	// Blue leds PWM %	(OnBoard CH3 -- PA7)
}pwm_val_t;

// Request (New settings)
typedef struct{
	pwm_val_t pwm;
	uint32_t smooth_value;
	char smooth_control[8];
	char curr_dtime[32];
	char curr_time[16];
	char start_time[16];
	char stop_time[16];
}req_t;

typedef __packed struct{
	uint32_t smooth_control;
	uint32_t smooth_value;
	pwm_val_t pwm;
	work_period_t wp[WORK_PERIODS_NUM];
}app_data_t;


void app_init(void);
void serve_input(void);
void config_BT(void);
void check_work_periods(rt_t *rt);
void smooth_power_up_handler(int tim_num);
void adjust_led_power(rt_t *rt);

#endif
