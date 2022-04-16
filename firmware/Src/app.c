
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>

#include "jsmn.h"
#include "jWrite.h"
#include "uart.h"
#include "flash.h"
#include "timer.h"
#include "rtc.h"
#include "app.h"

#define DEBUG_MSG 0
#include "dbgmsg.h"

#define MAX_TOKEN_NUM			32
#define MAX_TOKEN_LENGTH	128

static req_t request;
static app_data_t app_data;

static int app_data_wsize = sizeof(app_data_t) / sizeof(uint32_t);	// in words (32bit)	

static char uart_buf_rx[UART_BUF_SIZE];
static char uart_buf_tx[1024];				//UART_BUF_SIZE
static char uart_num = 0;							// From what UART request comes (1 or 3)

static jsmn_parser jp;
static jsmntok_t jt[MAX_TOKEN_NUM]; 	// We expect no more tokens 
static char keyString[MAX_TOKEN_LENGTH];
static char Prev_keyString[MAX_TOKEN_LENGTH];

static bool powered_on = false;				// default state - power_off



void app_init(void)
{
	memset(uart_buf_rx, '\0', sizeof(uart_buf_rx));
	memset(uart_buf_tx, '\0', sizeof(uart_buf_tx));
	
	// Restore app_data from FLASH
	flash_read_page(ADDR_FLASH_PAGE_127, (uint32_t*)&app_data, app_data_wsize);

#if 0
	dbgmsg("ch1_pwm_val: %d\r\nch2_pwm_val: %u\r\nch3_pwm_val: %u\r\n\
start_hour: %d\r\nstart_min: %d\r\nstop_hour: %d\r\nstop_min: %d\r\nenabled: %d\r\n",  
		app_data.pwm.ch1, app_data.pwm.ch2, app_data.pwm.ch3, app_data.wp[0].start_hour, 
		app_data.wp[0].start_min, app_data.wp[0].stop_hour, app_data.wp[0].stop_min, 
		app_data.wp[0].enabled);
#endif	
	
	TIM2_Init(10);	// 1 ms resolution
}


#define CHECK_AND_SET(str, var, min, max) 	do{	\
																			long _tmp = strtol(str, NULL, 10);	\
																			if(_tmp > max) var = max;	\
																			else if(_tmp < min) var = min;	\
																			else var = _tmp;	\
																		}while(0);

#define CHECK_AND_COPY(src, dest)		do{ \
																			if (len < sizeof(dest)) \
																				strcpy(dest, src); \
																			else{ \
																				strncpy(dest, src, sizeof(dest)-1); \
																				dest[sizeof(dest)-1] = '\0'; \
																			} \
																		}while(0);

static app_err_t parse_json(char *str)
{
	jsmn_init(&jp);
	memset(&request, -1, sizeof(pwm_val_t));
	strcpy(request.curr_dtime, "");
	strcpy(request.curr_time, "");
	strcpy(request.start_time, "");
	strcpy(request.stop_time, ""); 
	strcpy(request.smooth_control, "");

	int cnt = jsmn_parse(&jp, str, strlen(str), jt, sizeof(jt) / sizeof(jt[0]));
	
	if (cnt < 0) {
    dbgmsg("Failed to parse JSON: %d\r\n", cnt);
    return APP_JSON_PARSE_ERR;
  }

  /* Assume the top-level element is an object */
  if (cnt < 1 || jt[0].type != JSMN_OBJECT) {
    dbgmsg("jsmn_parse() Object expected\r\n");
    return APP_JSON_NO_OBJ;
  }
	
	for (int i = 0; i < cnt; i++){
		jsmntok_t key = jt[i];
		uint16_t len = key.end - key.start;
 
		if (len < MAX_TOKEN_LENGTH){
			memcpy(keyString, &str[key.start], len);
			keyString[len] = '\0';
			
			if(!strcmp(Prev_keyString, "ch1_power")){
				CHECK_AND_SET(keyString, request.pwm.ch1, 0, 1000);
			}
			else if(!strcmp(Prev_keyString, "ch2_power")){
				CHECK_AND_SET(keyString, request.pwm.ch2, 0, 1000);
			}
			else if(!strcmp(Prev_keyString, "ch3_power")){
				CHECK_AND_SET(keyString, request.pwm.ch3, 0, 1000);
			}
			else if(!strcmp(Prev_keyString, "current_time")){
				CHECK_AND_COPY(keyString, request.curr_time);
			}
			else if(!strcmp(Prev_keyString, "current_dtime")){
				CHECK_AND_COPY(keyString, request.curr_dtime);
			}
			else if(!strcmp(Prev_keyString, "smooth_control")){
				CHECK_AND_COPY(keyString, request.smooth_control);
			}
			else if(!strcmp(Prev_keyString, "smooth_value")){
				CHECK_AND_SET(keyString, request.smooth_value, (long)MIN_SMOOTH_VALUE, (long)MAX_SMOOTH_VALUE);
			}
			
			// TODO: Array of different work periods
			else if(!strcmp(Prev_keyString, "start_time")){
				CHECK_AND_COPY(keyString, request.start_time);
			}
			else if(!strcmp(Prev_keyString, "stop_time")){
				CHECK_AND_COPY(keyString, request.stop_time);
			}
			
			strcpy(Prev_keyString, keyString);
		}
		else{
			dbgmsg("WARNING: current token len > MAX_TOKEN_LENGTH\r\n");
			return APP_DATA_OVERFLOW;
		}	
	}
	
	return APP_OK;
}


static char* err_to_text(app_err_t err)
{
	static char str[512];		// For JSON PARSE error text
	
	switch(err){
		case APP_OK: return("OK");
    case APP_INVALID_DTIME_FORMAT: return("Invalid date-time format. Try 'dd.mm.yyyy hh:mm:ss'");
		case APP_INVALID_TIME_FORMAT: return("Invalid time format. Try 'hh:mm:ss' or 'hh:mm'"); 
		
		case APP_JSON_PARSE_ERR: {
			memset(str, '\0', sizeof(str));
			snprintf(str, sizeof(str), "JSON parser error: ( %s )", uart_buf_rx);
			return(str);
		}
		case APP_JSON_NO_OBJ: return("No JSON object found");
		case APP_INVALID_DATA: return("Invalid data");
		case APP_DATA_OVERFLOW: return("Data overflow");
		default: return("Unknown error");
	}
}

static void create_response(app_err_t err)
{
	memset(uart_buf_tx, 0, sizeof(uart_buf_tx));
	int res = 0;
	UNUSED(res);
	
	jwOpen( uart_buf_tx, sizeof(uart_buf_tx), JW_OBJECT, JW_COMPACT/*JW_PRETTY*/ );
	if(err < APP_CRITICAL_ERROR){
		rt_t rt;
		char str[32];
		RTC_Get_DateTime(&rt);
		sprintf(str, "%02u.%02u.%04u %02u:%02u:%02u",
								rt.date, rt.month, rt.year, rt.hour, rt.min, rt.sec);
		jwObj_int( "result_code", err );
		jwObj_string( "result_text", err_to_text(err) );
		jwObj_string( "current_dtime", str );
		memset(str, 0, sizeof(str));
		powered_on ? strcpy(str, "On") : strcpy(str, "Off");
		jwObj_string( "power_state", str );
		jwObj_int( "ch1_power", app_data.pwm.ch1 );
		jwObj_int( "ch2_power", app_data.pwm.ch2 );
		jwObj_int( "ch3_power", app_data.pwm.ch3 );
		memset(str, 0, sizeof(str));
		sprintf(str, "%02u:%02u:%02u", app_data.wp[0].start_hour, 
													app_data.wp[0].start_min, app_data.wp[0].start_sec );
		jwObj_string( "start_time", str );
		memset(str, 0, sizeof(str));
		sprintf(str, "%02u:%02u:%02u", app_data.wp[0].stop_hour, 
													app_data.wp[0].stop_min, app_data.wp[0].stop_sec );
		jwObj_string( "stop_time", str );
		jwObj_bool( "smooth_control", app_data.smooth_control);
		jwObj_int( "smooth_value", app_data.smooth_value );
	}
	else{
		jwObj_int( "result_code", err );
		jwObj_string( "result_text", err_to_text(err) );
	}

	if ((res = jwClose()) != 0){
		dbgmsg("jwClose() failed (%d)\r\n", res);
		uart_num = 0;
		return;
	}
	// Add Terminating symbols
	uart_buf_tx[strlen(uart_buf_tx)] = '\r';
	uart_buf_tx[strlen(uart_buf_tx)] = '\n';
	
	if (uart_num == 1) UART1_Send((uint8_t*)uart_buf_tx, strlen(uart_buf_tx));
	else if (uart_num == 3) UART3_Send((uint8_t*)uart_buf_tx, strlen(uart_buf_tx));
	
	uart_num = 0;
}

static bool is_number(char *str, int len)
{
	char *ptr = str;
	
	for(int i = 0; i < len; i++){
		if (!isdigit(*ptr)){
			//dbgmsg("'%c' is not digital\r\n", *ptr);
			return false;
		}
		ptr++;
	}
	
	return true;
}

// Supported time format: 'dd.mm.yyyy hh:mm:ss'
static rt_t* parse_dtime_string(char *str)
{
	static rt_t rt;
	memset(&rt, 0, sizeof(rt));
	char *p_end = NULL;
	char *p_start = NULL;
	char tmp[5] = {0};
	
	// Searching for 'date' value
	if ((p_end = strchr(str, '.')) == NULL) return NULL;
  *p_end = '\0';	// for strlen() and strcpy
	if(strlen(str) > 2) return NULL;
	memset(tmp, 0, sizeof(tmp));
	strcpy(tmp, str);
	if(!is_number(tmp, strlen(tmp))) return NULL;
	CHECK_AND_SET(tmp, rt.date, 1, 31);
	p_end++;	// move to 'mm'
	p_start = p_end;
	
	// Searching for 'month' value
	if ((p_end = strchr(p_start, '.')) == NULL) return NULL;
  *p_end = '\0';	// for strlen() and strcpy
	if(strlen(p_start) > 2) return NULL;
	memset(tmp, 0, sizeof(tmp));
	strcpy(tmp, p_start);
	if(!is_number(tmp, strlen(tmp))) return NULL;
	CHECK_AND_SET(tmp, rt.month, 1, 12);
	p_end++;	// move to 'yyyy'
	p_start = p_end;
	
	// Searching for 'year' value
	if ((p_end = strchr(p_start, ' ')) == NULL) return NULL;
  *p_end = '\0';	// for strlen() and strcpy
	if(strlen(p_start) != 4) return NULL;
	memset(tmp, 0, sizeof(tmp));
	strcpy(tmp, p_start);
	if(!is_number(tmp, strlen(tmp))) return NULL;
	CHECK_AND_SET(tmp, rt.year, 2019, 9999); 
	p_end++;	// move to 'hh'
	p_start = p_end;
	
	// Searching for 'hour' value
	if ((p_end = strchr(p_start, ':')) == NULL) return NULL;
  *p_end = '\0';	// for strlen() and strcpy
	if(strlen(p_start) > 2) return NULL;
	memset(tmp, 0, sizeof(tmp));
	strcpy(tmp, p_start);
	if(!is_number(tmp, strlen(tmp))) return NULL;
	CHECK_AND_SET(tmp, rt.hour, 0, 23);
	p_end++;	// move to 'mm'
	p_start = p_end;
	
	// Searching for 'mimunte' value
	if ((p_end = strchr(p_start, ':')) == NULL) return NULL;
  *p_end = '\0';	// for strlen() and strcpy
	if(strlen(p_start) > 2) return NULL;
	memset(tmp, 0, sizeof(tmp));
	strcpy(tmp, p_start);
	if(!is_number(tmp, strlen(tmp))) return NULL;
	CHECK_AND_SET(tmp, rt.min, 0, 59); 
	p_end++;	// move to 'ss'
	p_start = p_end;
	
	// Searching for 'seconds' value
	if(strlen(p_start) > 2) return NULL;
	memset(tmp, 0, sizeof(tmp));
	strcpy(tmp, p_start);
	if(!is_number(tmp, strlen(tmp))) return NULL;
	CHECK_AND_SET(tmp, rt.sec, 0, 59);
	
	//dbgmsg("New time:\t%u.%u.%u %u:%u:%u\r\n", rt.date, rt.month, rt.year, rt.hour, rt.min, rt.sec);
	return &rt;
}

static rt_t* parse_time_string(char *str)
{
	static rt_t rt;
	memset(&rt, 0, sizeof(rt));
	char *p_end = NULL;
	char *p_start = NULL;
	char tmp[5] = {0};
	
	// Searching for 'hour' value
	if ((p_end = strchr(str, ':')) == NULL) return NULL;
  *p_end = '\0';	// for strlen() and strcpy
	if(strlen(str) > 2) return NULL;
	memset(tmp, 0, sizeof(tmp));
	strcpy(tmp, str);
	if(!is_number(tmp, strlen(tmp))) return NULL;
	CHECK_AND_SET(tmp, rt.hour, 0, 23);
	p_end++;	// move to 'mm'
	p_start = p_end;
	
	// Searching for 'min' value
	if ((p_end = strchr(p_start, ':')) == NULL)
	{
		if(strlen(p_start) > 2) return NULL;
		else{ 
			memset(tmp, 0, sizeof(tmp));
			strcpy(tmp, p_start);
			if(!is_number(tmp, strlen(tmp))) return NULL;
			CHECK_AND_SET(tmp, rt.min, 0, 59);
			rt.sec = 0;	// If no 'ss' field found, counting as zero 
		}
	}
	else
	{
		*p_end = '\0';	// for strlen() and strcpy
		if(strlen(p_start) > 2) return NULL;
		memset(tmp, 0, sizeof(tmp));
		strcpy(tmp, p_start);
		if(!is_number(tmp, strlen(tmp))) return NULL;
		CHECK_AND_SET(tmp, rt.min, 0, 59);
		p_end++;	// move to 'ss'
		p_start = p_end;
		
		// Searching for 'seconds' value
		if(strlen(p_start) > 2) return NULL;
		//dbgmsg("sec strlen: %d\r\n", strlen(p_start));
		memset(tmp, 0, sizeof(tmp));
		strcpy(tmp, p_start);
		if(!is_number(tmp, strlen(tmp))) return NULL;
		CHECK_AND_SET(tmp, rt.sec, 0, 59);
  }
	
	//dbgmsg("New time:\t%u.%u.%u %u:%u:%u\r\n", rt.date, rt.month, rt.year, rt.hour, rt.min, rt.sec);
	return &rt;
}

// Leaves only characters before 'open_c' char and after 'close_c' char in str
static char* str_cut(char* str, char open_c, char close_c)
{
    char* p, *t;
    while(*str && (*str != open_c))
        ++str;
        
    t = NULL;
    for(p = str; *p; ++p){
        if(*p == close_c)
            t = p;
    }
 
    if(t != NULL)
        *(t+1)= '\0';
    return str;
}

static void save_new_settings(void)
{
	flash_write_page(ADDR_FLASH_PAGE_127, (uint32_t*)&app_data, app_data_wsize);
}

static int capitalize(char *str)
{
    int k = strlen(str);
    for (int i = 0; i < k; i++)
    {
        str[i] = toupper(str[i]);
    }
    return k;
}

void config_BT(void)
{
	uint32_t len = 0;
	
	if (UART1_Get((uint8_t*)uart_buf_rx, &len)){
		uart_buf_rx[len]='\r';
		uart_buf_rx[len+1]='\n';
		
		dbgmsg("sending to BLE [%u bytes]: %s", strlen(uart_buf_rx), uart_buf_rx);
		
		UART3_Send((uint8_t*)uart_buf_rx, strlen(uart_buf_rx));
		
		memset(uart_buf_rx, 0, sizeof(uart_buf_rx));
	}
	
	if (UART3_Get((uint8_t*)uart_buf_rx, NULL)){
		dbgmsg("BLE response: %s\r\n", uart_buf_rx);
	}
	
	memset(uart_buf_rx, 0, sizeof(uart_buf_rx));
}



void serve_input(void)
{
	if (UART1_Get((uint8_t*)uart_buf_rx, NULL)) uart_num = 1;
	else if (UART3_Get((uint8_t*)uart_buf_rx, NULL)) uart_num = 3;
	
	char *str = NULL;
	if ( NULL != (str = str_cut(uart_buf_rx, '{', '}')) ) strcpy(uart_buf_rx, str);
	
	if(uart_num)
	{
		dbgmsg("[ APP ] Got string: '%s'\r\n", uart_buf_rx);
		
		bool save_needed = false;
		app_err_t res = parse_json(uart_buf_rx);
		rt_t *rt;
		
		if (res == APP_OK)
		{
			if(strcmp(request.curr_dtime, "")){
				if((rt = parse_dtime_string(request.curr_dtime)) != NULL){
					// Set new RTC date and time
					RTC_Set_DateTime(rt);
				}
				else res = APP_INVALID_DTIME_FORMAT;
			}
			
			if(strcmp(request.curr_time, "")){
				if((rt = parse_time_string(request.curr_time)) != NULL){
					// Set new RTC time
					RTC_Set_Time(rt->hour, rt->min, rt->sec);
				}
				else res = APP_INVALID_TIME_FORMAT;
			}
		
			if(strcmp(request.start_time, "")){
				if((rt = parse_time_string(request.start_time)) != NULL){
					// Set new Start time
					app_data.wp[0].start_hour = rt->hour;
					app_data.wp[0].start_min = rt->min;
					app_data.wp[0].start_sec = rt->sec;
					save_needed = true;
				}
				else res = APP_INVALID_TIME_FORMAT;
			}
			
			if(strcmp(request.stop_time, "")){
				if((rt = parse_time_string(request.stop_time)) != NULL){
					// Set new Stop time
					app_data.wp[0].stop_hour = rt->hour;
					app_data.wp[0].stop_min = rt->min;
					app_data.wp[0].stop_sec = rt->sec;
					save_needed = true;
				}
				else res = APP_INVALID_TIME_FORMAT;
			}
			
			if(strcmp(request.smooth_control, "")){
				capitalize(request.smooth_control);
				
				if(!strcmp(request.smooth_control, "TRUE")){
					app_data.smooth_control = 1;
					save_needed = true;
				}
				else if(!strcmp(request.smooth_control, "FALSE")){
					app_data.smooth_control = 0;
					save_needed = true;
				}
				else res = APP_INVALID_DATA;
			}
			
			if(request.smooth_value > 0){
				save_needed = true;
				app_data.smooth_value = request.smooth_value;
				// Sets up new coefficient for smooth control formula
			}
			
			if(request.pwm.ch2 >= 0){
				save_needed = true;
				app_data.pwm.ch2 = request.pwm.ch2;
				// Set new PWM1 (CH2 on PCB)
				if(powered_on) TIM3_PWM_ConfigAndStart(TIM_CHANNEL_1, app_data.pwm.ch2);
			}
			
			if(request.pwm.ch3 >= 0){
				save_needed = true;
				app_data.pwm.ch3 = request.pwm.ch3;
				// Set new PWM2 (CH3 on PCB)
				if(powered_on) TIM3_PWM_ConfigAndStart(TIM_CHANNEL_2, app_data.pwm.ch3);
			}
			
			if(request.pwm.ch1 >= 0){
				save_needed = true;
				app_data.pwm.ch1 = request.pwm.ch1;
				// Set new PWM3 (CH1 on PCB)
				if(powered_on) TIM3_PWM_ConfigAndStart(TIM_CHANNEL_3, app_data.pwm.ch1);
			}

			// Save new settings fo FLASH
			if(save_needed) save_new_settings();
		}
		
		create_response(res);	
	}	
	
	memset(uart_buf_rx, 0, sizeof(uart_buf_rx));
}	

// Smooth control section 

#define PWM_INC(x, max)	do{ \
	if(x < max) x = ((x + PWM_DISCRETE) > max) ? max : (x + PWM_DISCRETE); \
}while(0)

#define PWM_DEC(x)			do{ \
	if(x > 0) x = ((int)(x - PWM_DISCRETE) < 0) ? 0 : (x - PWM_DISCRETE); \
}while(0)


// Coefficients for smooth control formula
const uint32_t u = 430;
const uint32_t z = 10;

// Smooth control formula. Depending of current power sets up time period of power changing
// NOTE: For LEDs lower power should be changed faster than higher one for 'smooth' effect.
#define SMOOTH_PERIOD_ADJUSMENT(curr_power)		( ((curr_power + u) / app_data.smooth_value) + z )	

typedef struct
{
	uint32_t pwm_channel;		// Hardware timer PWM channel
	v_tim_name vtim;				// Virtual timer number
	uint32_t curr_power;		// Current channel power
	uint32_t dest_power;		// Needed to be set power
	uint32_t period_ms;			// Current vtim period
	_Bool powered_up;				// Flag that curr_power reaches dest_power
}smooth_ctrl;

// Smooth control data storage
#define CHANNELS_NUM			3
#define START_PERIOD_MS		9		

static smooth_ctrl sm_ctrl[CHANNELS_NUM] = {
	{TIM_CHANNEL_3, vtim1, 0, 0, START_PERIOD_MS, 0},	// CH1 smooth handler
	{TIM_CHANNEL_1, vtim2, 0, 0, START_PERIOD_MS, 0},	// CH2 smooth handler
	{TIM_CHANNEL_2, vtim3, 0, 0, START_PERIOD_MS, 0},	// CH3 smooth handler
};

// Prepare for power on routine
static void smooth_ctrl_init(void)
{
	for(int i = 0; i < CHANNELS_NUM; ++i){
		sm_ctrl[i].curr_power = 0;
		sm_ctrl[i].period_ms = START_PERIOD_MS;
		sm_ctrl[i].powered_up = 0;
		TIM2_set_virt_period(sm_ctrl[i].vtim, sm_ctrl[i].period_ms);
	}
	
	sm_ctrl[0].dest_power = app_data.pwm.ch1;
	sm_ctrl[1].dest_power = app_data.pwm.ch2;
	sm_ctrl[2].dest_power = app_data.pwm.ch3;
}

// Prepare for power off routine
static void smooth_ctrl_update(void)
{
	// Update cuur_power counter only if destination power was reached
	if(sm_ctrl[0].curr_power == sm_ctrl[0].dest_power){
		sm_ctrl[0].curr_power = app_data.pwm.ch1;
	}
	if(sm_ctrl[1].curr_power == sm_ctrl[1].dest_power){
		sm_ctrl[1].curr_power = app_data.pwm.ch2;
	}
	if(sm_ctrl[2].curr_power == sm_ctrl[2].dest_power){
		sm_ctrl[2].curr_power = app_data.pwm.ch3;
	}
	
	for(int i = 0; i < CHANNELS_NUM; ++i){
		sm_ctrl[i].dest_power = 0;
		//sm_ctrl[i].period_ms = START_PERIOD_MS;
		sm_ctrl[i].powered_up = 1;
		TIM2_set_virt_period(sm_ctrl[i].vtim, sm_ctrl[i].period_ms);
	}
}

// Called when any of virtual timers' period ellapsed  
void smooth_power_up_handler(int tim_num)
{
	if(powered_on)
	{
		// Routine for all of 3 pwm-channels (independent)
		for(int i = 0; i < CHANNELS_NUM; ++i){
			// Check number of virtual timer
			if(tim_num & sm_ctrl[i].vtim){
				// Set up PWM value
				PWM_INC(sm_ctrl[i].curr_power, sm_ctrl[i].dest_power);
				TIM3_PWM_ConfigAndStart(sm_ctrl[i].pwm_channel, sm_ctrl[i].curr_power);	
				// Update virtual timer if needed
				if(sm_ctrl[i].curr_power < sm_ctrl[i].dest_power){
					sm_ctrl[i].period_ms = SMOOTH_PERIOD_ADJUSMENT(sm_ctrl[i].curr_power);
					TIM2_set_virt_period(sm_ctrl[i].vtim, sm_ctrl[i].period_ms);
				}
				else sm_ctrl[i].powered_up = 1;
			}
		}
		
		// Switch timer off when every channel has power as demanded
		if(sm_ctrl[0].powered_up && sm_ctrl[1].powered_up && sm_ctrl[2].powered_up)
			TIM2_Stop();
	}
	else
	{
		dbgmsg("sm1: %u, sm2: %u, sm3: %u\r\n", smooth_ch1 ,smooth_ch2 ,smooth_ch3);
		// Routine for all of 3 pwm-channels (independent)
		for(int i = 0; i < CHANNELS_NUM; ++i){
			if(tim_num & sm_ctrl[i].vtim){
				// Set up PWM value
				PWM_DEC(sm_ctrl[i].curr_power);
				TIM3_PWM_ConfigAndStart(sm_ctrl[i].pwm_channel, sm_ctrl[i].curr_power);
				// Update virtual timer if needed
				if(sm_ctrl[i].curr_power > sm_ctrl[i].dest_power){
					sm_ctrl[i].period_ms = SMOOTH_PERIOD_ADJUSMENT(sm_ctrl[i].curr_power/*sm_ctrl[i].tick_cnt*/);
					TIM2_set_virt_period(sm_ctrl[i].vtim, sm_ctrl[i].period_ms);
				}
				else sm_ctrl[i].powered_up = 0;
			}
		}
		// Switch PWM off when every channel has droped power
		if(!sm_ctrl[0].powered_up && !sm_ctrl[1].powered_up && !sm_ctrl[2].powered_up){
			TIM2_Stop();
			TIM3_PWM_Stop(TIM_CHANNEL_1); 
			TIM3_PWM_Stop(TIM_CHANNEL_2); 
			TIM3_PWM_Stop(TIM_CHANNEL_3);
		}
	}
}


#define POWER_UP()			do{ \
	if(app_data.smooth_control){ \
		smooth_ctrl_init(); \
		TIM2_Start(); \
	} \
	else{ \
		TIM2_Stop(); \
		TIM3_PWM_ConfigAndStart(TIM_CHANNEL_1, app_data.pwm.ch2); \
		TIM3_PWM_ConfigAndStart(TIM_CHANNEL_2, app_data.pwm.ch3); \
		TIM3_PWM_ConfigAndStart(TIM_CHANNEL_3, app_data.pwm.ch1); \
	} \
}while(0)
	

#define POWER_DOWN()		do{ \
	if(app_data.smooth_control){ \
		smooth_ctrl_update(); \
		TIM2_Start(); \
	} \
	else{ \
		TIM2_Stop(); \
		TIM3_PWM_Stop(TIM_CHANNEL_1); \
		TIM3_PWM_Stop(TIM_CHANNEL_2); \
		TIM3_PWM_Stop(TIM_CHANNEL_3); \
	} \
}while(0)


// Call every [N] sec to check if PWM generations must be switched
void check_work_periods(rt_t *rt)
{
	uint64_t rt_sec = rt->hour * 3600 + rt->min * 60 + rt->sec;
	uint64_t start_sec = app_data.wp[0].start_hour * 3600 + 
												app_data.wp[0].start_min * 60 + app_data.wp[0].start_sec;
	uint64_t stop_sec = app_data.wp[0].stop_hour * 3600 + 
												app_data.wp[0].stop_min * 60 + app_data.wp[0].stop_sec; 
	
	//dbgmsg("rt_sec: %llu, start_sec: %llu, stop_sec: %llu\r\n", rt_sec, start_sec, stop_sec);
	
	if ( (rt_sec >= start_sec) && (rt_sec < stop_sec) )
	{
		if(!powered_on){
			powered_on = true;
			POWER_UP();
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			//dbgmsg("PWM: Power ON\r\n");
		}
	}
	else if(powered_on)
	{
		powered_on = false;
		POWER_DOWN();
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		//dbgmsg("PWM: Power Off\r\n");		
	}
}

// General state led power depends on day-time
void adjust_led_power(rt_t *rt)
{
	if(rt->hour > 22) TIM1_PWM_ConfigAndStart(3);
	else if(rt->hour >= 20) TIM1_PWM_ConfigAndStart(6);
	else if(rt->hour >= 17) TIM1_PWM_ConfigAndStart(9);
	else if(rt->hour >= 12) TIM1_PWM_ConfigAndStart(12);
	else if(rt->hour >= 9) TIM1_PWM_ConfigAndStart(9);
	else if(rt->hour >= 6) TIM1_PWM_ConfigAndStart(6);
	else if(rt->hour > 0) TIM1_PWM_ConfigAndStart(3);
	else if(rt->hour == 0) TIM1_PWM_ConfigAndStart(1);
	else TIM1_PWM_ConfigAndStart(12);
}







