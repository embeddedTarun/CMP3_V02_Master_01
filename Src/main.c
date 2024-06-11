/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "lcd1604.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


typedef uint8_t crc_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define WIDTH  (8 * sizeof(crc_t))
#define TOPBIT (1 << (WIDTH - 1))


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/////////////////////

/////////////////////////


#define joy_btn_Pin GPIO_PIN_10
#define jbtn_GPIO_Port  GPIOB

//#define x_channel ADC_Select_4
//#define y_channel ADC_Select_5

uint16_t motor_running = 0;
uint8_t eeprom_mu_mode = 0;            // eeprom variable
long double eeprom_step = 0.0;    // eeprom variable

char working_mode  = 'G';     // define the current working mode (G ,L, P, V, R, so on )
char man_unl_mode;        // =   'U';    // define the MAN and UNL mode of the device
char  limit_status;    // =   '+';    // define the status of the limit mean limt is possitive or negative
uint8_t back_light = 100;     // back light of the led
char callback_mode = 'N';
int mode_data;

uint8_t j_toggle1 = 0;
uint8_t j_toggle = 0;
uint8_t prev_row_pos = 0;    // hold the status of the previous row position
uint32_t adc_value = 0;
uint32_t raw_value = 0;
uint8_t button = 1;          // joy stic button status
uint32_t x_value = 0;  		 // joy stic x value from ADC
uint32_t y_value = 0;   	 // joy stic y value from ADC
uint32_t spd_value = 0;      // speed value
uint32_t damp_value = 0;     // damping value
uint8_t  home = 1;        	 // hold the exact position of the cursor in home screen
uint8_t  hold = 0;   		 // hold the function in while loop
uint8_t  jpos = 0;           // cursor (< >) detection
uint8_t  row_pos = 0;  		 // row position
uint8_t previous_pos = 0;    // Previous row position
uint8_t  row_pos1 = 0;
uint8_t  arrow = 0;   	 	 //  arrow ( > ) position in the UNL mode  function
uint8_t  pressed = 0;
		//long double limit_1 = 0.0;   	     // motor limit_1 value
		//long double limit_2 = 0.0;   	     // motor limit_2 value
long double motor_step = 0.0;      // final motor steps after setting limit_1 and limit_2
		//long double motor_limit = 0.0;     // Difference of 1st limit and 2nd limt in the man mode
long double motor_count = 0.0;     // disply the motor step on the lcd screen
long double fixed_limit = 0.0;     // motor limits after limits setting

int64_t limit_1 = 0;   	     // motor limit_1 value
int64_t limit_2 = 0;
int64_t motor_limit = 0;     // motor limit difference between 1st and 2nd limit in Limited mode
int64_t motor_move_step = 0;  // In animation mode motor step move L or R in


uint8_t motor_dir = 0;        // motor direction status 1 = Left /reverse and 2 = RIGHT / Forward
uint8_t motor_speed_pot = 0;  // speed port value after mapped 0 to 100
uint8_t motor_damp_pot = 0;  //  damping port value after mapped 0 to 100
uint8_t motor_damp_dis = 0;
uint16_t speed_send = 0;     // speed value send to the slave device
char direction;              // direction of the motor Left or Right
char motor_move = 'R';              // motor move L / R in the animation mode
char send_step_status;       // status M / S of the send step function motor will move or stop
long double current_motor_step = 0.0; // hold the current motor step position in every function.

uint16_t current_spd = 0;   // hold the current speed of the motor
uint16_t target_spd = 0;    // hold the target speed of the motor
uint16_t deaccel_rate = 0;  // hold the de-acceleration rate of the motor
float  pree_step = 0.0;       // hold the value of pree step to stop the motor
float  step_error = 0.0;
float  max_point = 0.0;      // upper limit of the motor in limit mode
float  min_point = 0.0;      // lower limit of the motor in limit mode

uint8_t man_mode = 0;       //
uint8_t unl_mode = 0;
uint8_t man_flag = 0;
uint8_t unl_flag = 0;
uint8_t live_flag = 0;
uint8_t fr_flag = 0;
uint8_t frpv_flag = 0;
uint8_t exit_flag = 0;
uint8_t exit1_flag = 0;
uint8_t start_flag;
uint8_t animation_flag = 0;
uint8_t timelapse_flag = 0;
uint8_t cursor_pos = 0;
double dmp_data1 = 0.0;     // used in the damping display function

//********** Animation Mode variables

//unsigned int  home_pos = 0;  // home position in animation mode
int32_t  home_pos;     			// = 0;  // home position in animation mode
float  old_home_pos = 0.0;   	// old position in animation mode
unsigned int new_home_pos = 0;    // new position in animation mode
float current_pos = 0.0;     	// current position in animation mode

//double roundedStep = 0.1;
float roundedStep = 0.1;
float step;          			 // = 0.1;           // double // motor will move step to left or right in animation mode
uint32_t ani_step_data;
char step_inc_dec;           	 // motor step increment or decrement in animation mode.
uint16_t steps;      // = 0;          // total steps of the shots taken in animation mode
uint16_t steps_limit = 0;
uint8_t shots;       // = 0;          // No of shorts to be taken in animation
uint8_t  delay_val;  // = 0;     // delay between the each short
uint8_t  motor_lr =  0;     	// motor direction Left or Right
uint8_t steps_count = 0;   	 	//  steps counter in the animation start function
uint8_t shots_count = 0;   		 // shots counter in animation start function
uint32_t wait_sec = 0;     	 	// wati between on shot in animation start function
int64_t max_limit = 0;     		 // motor move max limit in animation mode (unlimited )
int64_t min_limit = 0;      	// motor move min limit in animation mode ( unlimited )
uint16_t multi_step = 0;   	 	// mult steps data send in animation mode
uint16_t local = 0;         	// step value in animation mode
uint16_t raw = 1;
uint8_t yes_no = 0;


//******** TimeLapse variables

uint16_t interval_val;       // = 1;    // uint16_t  //  interval value in the timelapse mode
uint16_t interval_count = 0;    // uint16_t
uint16_t inter = 0;
uint8_t AM = 0;
uint16_t expos_count = 0.0;
//double time_step_u = 0.5;       //  motor step value in time lapse mode
uint32_t step_data = 0;
double time_step_m = 0.0;       //  motor step in limited mode
float time_step_u;       // = 0.5;       //  motor step value in time lapse mode
float expos_val;         // = 0.2;        // expos value in the timelapse mode
uint16_t expos_data = 2;
char  time_mode = 'C';            // mode (continute of SDS) in timelapse mode
uint32_t time_shots;     // = 4;       // No of shots to be taken in the timelapse mode
uint32_t time_shots_count = 0; // shots counter in start function
int64_t time_duration = 0;    // total time duration of shots and interval.
int64_t time_chek_out = 0;
int16_t time_hour = 0;
uint8_t time_minute = 0;
uint8_t time_second = 0;

//********* Recording and Play back mode Variables

int64_t rec_home = 0;
uint8_t time_period = 0;
uint8_t  speed_buffer[1800] = {0};       //  for recording and play back 360
char direction_buffer[900] =  {0};  //  for recording and play back 180
//uint8_t  index_buffer[600] = {0};
//uint8_t  mstep_buffer[600] = {0};
//uint16_t bindex = 0;
uint16_t s_index = 0;
uint16_t d_index = 0;
uint16_t m_index = 0;
int mills = 0;
uint32_t x_delay = 0;        // delay generate according to the joystic position
uint8_t ac = 0;
uint32_t adc_sum = 0;      // the get the average value of  the adc value
uint8_t length = 0;        // stlen function
uint8_t currentMillis = 0;
int hms_start = 0;
char string[20];
char string1[64];

//******** EEPROM variables
int32_t dumy_data;
//char uart_buf[50];
//int uart_buf_len;
uint8_t spi_buf[10];
uint8_t rec_buf[10];
uint8_t msb_addr = 0x00;
uint8_t lsb_addr = 0x00;
uint8_t wip;
uint32_t rec_data = 0;
int64_t rec_data1 = 0;

const uint8_t EEPROM_READ  = 0b00000011;
const uint8_t EEPROM_WRITE = 0b00000010;
const uint8_t EEPROM_WRDI  = 0b00000100;
const uint8_t EEPROM_WREN  = 0b00000110;
const uint8_t EEPROM_RDSR  = 0b00000101;
const uint8_t EEPROM_WRSR  = 0b00000001;


crc_t POLYNOMIAL = 0xcb;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void home_scr(void);
void auto_unl(void);
void read_joystic();
uint32_t get_adc_value(uint32_t channel);
void read_joystic();
void float_to_string(float value);
void back_dis(void);
void live_fun(void);
void free_ride_fun(void);
void re_calibration(void);
void start_fun(void);
void data_transmit(void);
void get_dam_pot_value(void);
void get_spd_pot_value(void);
void rx_data(void);
void data_display(void);
void step_start_fun(void);
void pause_stop_fun(void);
void disp_spd_val( uint8_t port_data);
void disp_dmp_val( uint8_t dmp_data);
char *Convert_int_to_String(int integer);
void disp_send_spd_val( uint16_t ss_data);
char *Convert_float_to_String(double value1);
void send_motor_limit(void);
void pree_step_cal(void);
void motor_soft_stop(void);
void ani_screen1(void);
void ani_screen2(void);
void arrow_up();
void arrow_down();
void time_screen1();
void time_screen2();
void time_screen3();
void time_duration_cal();



////////// 485 //////////

//void USART1_IRQHandler(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void send_on_rs485(uint8_t* buffer);
void recieve_on_rs485(uint8_t buffer[],size_t buffer_length);

crc_t CRC_generate(crc_t* message, crc_t polynomial, int nBytes );
uint8_t* message_packet_with_crc(crc_t* message2, crc_t polynomial, int nBytes );
crc_t CRC_CHECK_decode(crc_t* message, crc_t polynomial, int nBytes );


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

///////  485     ////

//uint8_t* send_buffer="h---Lcd\r\n";
//uint8_t send_buffer[10]={0x55,'-','-','-','L','c','d','\r','\n',0x01};

uint8_t  rx_motor_dir;
int64_t rx_motor_step = 0;

uint8_t send_buffer[40] = {0x55,'S',0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x01,0x55};

uint8_t Previous_buffer[40];
uint8_t RECIEVE_VALID_DATA[40];
uint8_t recieve_buffer[40];

uint8_t RS_485_Data_validate=0;
int BUFFER_LENGTH=40;

uint8_t state_of_rs485 = 1;

//char buffer1[9]="Driver2\r\n";
//uint8_t buffer[14];

int counter;
uint8_t cnt =0;


/// home screen message /////

//void enableButtonInterrupt(void)
//{
//  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
//  interruptEnabled = 1;
//}
//
//// Disable button interrupt
//void disableButtonInterrupt(void)
//{
//  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
//  interruptEnabled = 0;
//}




double roundToDecimal(double num, int decimalPlaces)
{
    double factor = pow(10, decimalPlaces);
    return round(num * factor) / factor;
}




long map(long x, long in_min, long in_max, long out_min, long out_max)
	{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

//motor_damp_pot = map( damp_value,  0, 4095, 0, 100);
//
//  void home_scr(void)
//  {
//	  lcd_put_cur(1, 1);
//	  lcd_string_new("RE-CALIBRATE?");
//	  lcd_put_cur(2, 2);
//	  lcd_string_new("<NO>");
//	  lcd_put_cur(2, 9);
//	  lcd_string_new(" YES ");
//  }

////  ADC value read function

  uint32_t get_adc_value(uint32_t channel)
  {
  	ADC_ChannelConfTypeDef sConfig = {0};
  	sConfig.Channel = channel;
  	sConfig.Rank =  ADC_REGULAR_RANK_1;    //1;
  	sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;    //ADC_SAMPLETIME_13CYCLES_5;

       if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  	 {
  		 Error_Handler();
  	  }

   	   HAL_ADC_Start(&hadc1);

   if (HAL_ADC_PollForConversion(&hadc1, 10 ) ==  HAL_OK)   //      4095);
    {
	   adc_value = HAL_ADC_GetValue(&hadc1);
     }
     else
    {
	   Error_Handler();
    }

   //  HAL_ADC_Stop(&hadc1);

     return(adc_value);

  }


///// joystic value read function

  void read_joystic()
  {
	y_value = get_adc_value(ADC_CHANNEL_4);                 //(ADC_CHANNEL_4);
	//HAL_Delay(1);
    x_value = get_adc_value(ADC_CHANNEL_3);                 //(ADC_CHANNEL_3);
   // HAL_Delay(1);
  }


  void jx_delay()
  {
  	if(x_value <= 2000)       { x_delay = map(x_value,  0,    900, 0, 250);}
  	else if (x_value >= 3000) { x_delay = map(x_value, 4040, 3000, 0, 250);}

  	if(x_delay >=2)
  	{
      for (int m = 0; m < x_delay; m++)
      {  HAL_Delay(1); }
  	}
  }



 void lcd_int_to_str(int raw_data)  //////// int to string convert function
  {
  	char str1[12];
  	sprintf(str1, "%d", raw_data);
  //	lcd_put_cur(1, 10);
    lcd_string_new(str1);
  	//return str;

  }

 char *Convert_int_to_String(int integer)
 {
  // char string[20];
   sprintf(string1, "%d", integer);
   return (string1);
 }

 char *Convert_float_to_String(double value1)
  {
    sprintf(string1, "%.1f", value1);
    return(string1);
  }

  void float_to_string(float value)
  {
    char str[20];
    sprintf(str, "%.1f", value);
    lcd_string_new(str);
  }


 void rx_data()
 {
	    rx_motor_dir    =      RECIEVE_VALID_DATA[1];

		rx_motor_step =  RECIEVE_VALID_DATA[2] << 56;
		rx_motor_step |= RECIEVE_VALID_DATA[3] << 48;
		rx_motor_step |= RECIEVE_VALID_DATA[4] << 40;
		rx_motor_step |= RECIEVE_VALID_DATA[5] << 32;
		rx_motor_step |= RECIEVE_VALID_DATA[6] << 24;
		rx_motor_step |= RECIEVE_VALID_DATA[7] << 16;
		rx_motor_step |= RECIEVE_VALID_DATA[8] << 8;
		rx_motor_step |= RECIEVE_VALID_DATA[9];
		////////// motor running status
		motor_running = RECIEVE_VALID_DATA[20];


        ////////////////
		motor_step = rx_motor_step / 10.0;  // + ((rx_motor_step % 10) / 10.0);     //2560;       // convert the motor micro steps into float numbers
 }

 // send the Set limit to the slave device
 void send_motor_limit()
 {
	 //direction = 'L';
//	 send_buffer[7] =  working_mode;  // 'L';  // motor limit set
	 send_buffer[8] =  (motor_limit >> 56) & 0xFF;
	 send_buffer[9] =  (motor_limit >> 48) & 0xFF;
	 send_buffer[10] =  (motor_limit >> 40) & 0xFF;
	 send_buffer[11] = (motor_limit >> 32) & 0xFF;
	 send_buffer[12] = (motor_limit >> 24) & 0xFF;
	 send_buffer[13] = (motor_limit >> 16) & 0xFF;
	 send_buffer[14] = (motor_limit >> 8)  & 0xFF;
	 send_buffer[15] =  motor_limit & 0xFF;
	 send_buffer[16] =  limit_status;

//	 while(state_of_rs485 != 1);
//	 send_on_rs485(send_buffer);
//	 HAL_Delay(10);

 }

// Send the motor_step_move to slave device for L or R in animation mode
 void send_step()

 {
	// send_buffer[1]  =  direction;
	 send_buffer[17] =  (motor_move_step >> 56) & 0xFF;
	 send_buffer[18] =  (motor_move_step >> 48) & 0xFF;
	 send_buffer[19] =  (motor_move_step >> 40) & 0xFF;
	 send_buffer[20] =  (motor_move_step >> 32) & 0xFF;
	 send_buffer[21] =  (motor_move_step>> 24) & 0xFF;
	 send_buffer[22] =  (motor_move_step >> 16) & 0xFF;
	 send_buffer[23] =  (motor_move_step >> 8)  & 0xFF;
	 send_buffer[24] =  motor_move_step & 0xFF;
	 send_buffer[25] =  send_step_status;
	 send_buffer[26] =  00; // shot click and zoom value 00, 10, 11

 }



   // calculate the Pree steps to stop the motor

 void pree_step_cal()
 {
	current_spd  = RECIEVE_VALID_DATA[16] << 8;
	current_spd |= RECIEVE_VALID_DATA[17];
	deaccel_rate  = RECIEVE_VALID_DATA[18] << 8;
	deaccel_rate |= RECIEVE_VALID_DATA[19];
//	s= ( ( (y*y) ) / (2*z) )/2;
   target_spd = 0;
//   pree_step =  ( ((current_spd * current_spd)) - (target_spd * target_spd) / ( 2 * deaccel_rate)) / 20;

	pree_step = 	(current_spd * current_spd);
	pree_step =     pree_step  / (2 * deaccel_rate);
	pree_step =     pree_step / 20;
	step_error = pree_step * 10 / 100;
	pree_step = pree_step + step_error;
	max_point = fixed_limit - pree_step;
	min_point = (0.0 + pree_step );

 ////   lcd_put_cur(3, 10);  float_to_string(pree_step); show pree step on the lcd

 }


 //********* transmit the data over RS484 module **************

 void load_buffer()
 {
 	 send_buffer[1] = direction;
 	 send_buffer[2] = motor_speed_pot;
 	 send_buffer[3] = motor_damp_pot;
  	 send_buffer[4] = speed_send>>8;
 	 send_buffer[5] = speed_send & 0x00FF;
 	 send_buffer[6] = man_unl_mode;
 	 send_buffer[7] = working_mode; // 'L';  // motor limit set
 }


 void data_transmit()
 {
 	 send_buffer[1] = direction;
 	 send_buffer[2] = motor_speed_pot;
 	 send_buffer[3] = motor_damp_pot;
  	 send_buffer[4] = speed_send>>8;
 	 send_buffer[5] = speed_send & 0x00FF;
 	 send_buffer[6] = man_unl_mode;
 	 send_buffer[7] = working_mode; // 'L';  // motor limit set

    // limit data
	// send_buffer[7] =  working_mode;
	 send_buffer[8]  =   (motor_limit >> 56) & 0xFF;
	 send_buffer[9]  =   (motor_limit >> 48) & 0xFF;
	 send_buffer[10] =  (motor_limit >> 40) & 0xFF;
	 send_buffer[11] =  (motor_limit >> 32) & 0xFF;
	 send_buffer[12] =  (motor_limit >> 24) & 0xFF;
	 send_buffer[13] =  (motor_limit >> 16) & 0xFF;
	 send_buffer[14] =  (motor_limit >> 8)  & 0xFF;
	 send_buffer[15] =  motor_limit & 0xFF;
	 send_buffer[16] =  limit_status;
 	 // send steps data
	 send_buffer[17] =  (motor_move_step >> 56) & 0xFF;
	 send_buffer[18] =  (motor_move_step >> 48) & 0xFF;
	 send_buffer[19] =  (motor_move_step >> 40) & 0xFF;
	 send_buffer[20] =  (motor_move_step >> 32) & 0xFF;
	 send_buffer[21] =  (motor_move_step>> 24) & 0xFF;
	 send_buffer[22] =  (motor_move_step >> 16) & 0xFF;
	 send_buffer[23] =  (motor_move_step >> 8)  & 0xFF;
	 send_buffer[24] =  motor_move_step & 0xFF;
	 send_buffer[25] =  send_step_status;

 	 while(state_of_rs485 != 1);
 	 send_on_rs485(send_buffer);
 	// HAL_Delay(1);
 }


 void transmit_data()
 {
     HAL_Delay(1);
 	 while(state_of_rs485 != 1);
 	 send_on_rs485(send_buffer);
 	 //HAL_Delay(1);
 	HAL_Delay(6);

 }


   // Display the Speed port value   //

  void disp_spd_val( uint8_t port_data)
 {
   length = strlen(Convert_int_to_String(port_data));
   if (length == 1) {
     lcd_put_cur(2, 0);
     lcd_string_new("  ");
     lcd_put_cur(2, 2);
    // lcd_int_to_str(port_data);
     lcd_string_new(Convert_int_to_String(port_data));
   }
   else if (length == 2) {
     lcd_put_cur(2, 0);
     lcd_string_new(" ");
     lcd_put_cur(2, 1);
  //   lcd_int_to_str(port_data);
     lcd_string_new(Convert_int_to_String(port_data));
   }
   else if (length == 3) {
     lcd_put_cur(2, 0);
   //  lcd_int_to_str(port_data);
     lcd_string_new(Convert_int_to_String(port_data));
   }

   }

  //  display the damping pot value //

 void disp_dmp_val( uint8_t dmp_data)
 {

  dmp_data1 =  dmp_data  / 10.0 ;
   uint8_t length1 = strlen(Convert_float_to_String(dmp_data1));
//   if (length1 == 1)
//   {
//     lcd_put_cur(2, 13);
//     lcd_string_new("  ");
//     lcd_put_cur(2, 15);
//     //lcd_int_to_str(dmp_data);
//     lcd_string_new(Convert_float_to_String(dmp_data));
//   }
    if (length1 == 3)
   {
     lcd_put_cur(2, 12);
     lcd_string_new(" ");
     lcd_put_cur(2, 13);
  //   lcd_int_to_str(dmp_data);
     lcd_string_new(Convert_float_to_String(dmp_data1));
   }
   else if (length1 == 4)
   {
     lcd_put_cur(2, 12);
    // lcd_int_to_str(dmp_data);
     lcd_string_new(Convert_float_to_String(dmp_data1));
   }

 }

 // display the speed value send by joy-stic   //

 void disp_send_spd_val(uint16_t ss_data)   // display the speed send value to slave device
{
  uint8_t length2 = strlen(Convert_int_to_String(ss_data));
  if (length2 == 1) {
    lcd_put_cur(2, 7);   ///0
    lcd_string_new("   ");
    lcd_put_cur(2, 10);   // 2
   // lcd_int_to_str(port_data);
    lcd_string_new(Convert_int_to_String(ss_data));
  }
  else if (length2 == 2) {
    lcd_put_cur(2, 7);     //0
    lcd_string_new("  ");
    lcd_put_cur(2, 9);		//1
 //   lcd_int_to_str(port_data);
    lcd_string_new(Convert_int_to_String(ss_data));
  }
  else if (length2 == 3) {
    lcd_put_cur(2, 7);     //0
    lcd_string_new(" ");
    lcd_put_cur(2, 8);		//1
 //   lcd_int_to_str(port_data);
    lcd_string_new(Convert_int_to_String(ss_data));
  }

  else if (length2 == 4) {
    lcd_put_cur(2, 7);     // 0
  //  lcd_int_to_str(port_data);
    lcd_string_new(Convert_int_to_String(ss_data));
  }

  }

  //  display the No. of motor steps motor move  //

 void disp_motor_step(long double ms_data)   // display the speed send value to slave device
{
  uint8_t length3 = strlen(Convert_float_to_String(ms_data));
  if (length3 == 3)
  {
    lcd_put_cur(1, 9);        ///0
    lcd_string_new("    ");
    lcd_put_cur(1, 13);       // 2
    lcd_string_new(Convert_float_to_String(ms_data));
  }
  else if (length3 == 4) {
    lcd_put_cur(1, 9);     //0
    lcd_string_new("   ");
    lcd_put_cur(1, 12);		//1
    lcd_string_new(Convert_float_to_String(ms_data));
  }
  else if (length3 == 5) {
    lcd_put_cur(1, 9);     //0
    lcd_string_new("  ");
    lcd_put_cur(1, 11);		//1
    lcd_string_new(Convert_float_to_String(ms_data));
  }

  else if (length3 == 6) {
    lcd_put_cur(1, 9);     //0
    lcd_string_new(" ");
    lcd_put_cur(1, 10);		//1
    lcd_string_new(Convert_float_to_String(ms_data));
  }

  else if (length3 == 7) {
    lcd_put_cur(1, 9);     // 0
    lcd_string_new(Convert_float_to_String(ms_data));
  }

  }


 ///// home_pos display of Animation mode

 void disp_home_pos(int32_t hp_data)   // display the speed send value to slave device
 {
   uint8_t length4 = strlen(Convert_int_to_String(hp_data));
   if (length4 == 1) {
     lcd_put_cur(1, 10);   ///0
     lcd_string_new("    ");
     lcd_put_cur(1, 14);   // 2
     lcd_string_new(Convert_int_to_String(hp_data));
   }
   else if (length4 == 2) {
     lcd_put_cur(1, 10);     //0
     lcd_string_new("   ");
     lcd_put_cur(1, 13);		//1
     lcd_string_new(Convert_int_to_String(hp_data));
   }
   else if (length4 == 3) {
     lcd_put_cur(1, 10);     //0
     lcd_string_new("  ");
     lcd_put_cur(1, 12);		//1
     lcd_string_new(Convert_int_to_String(hp_data));
   }
   else if (length4 == 4) {
     lcd_put_cur(1, 10);     //0
     lcd_string_new("  ");
     lcd_put_cur(1, 11);		//1
     lcd_string_new(Convert_int_to_String(hp_data));
   }
    else if (length4 == 5) {
     lcd_put_cur(1, 10);     // 0
     lcd_string_new(Convert_int_to_String(hp_data));
   }

   }



 /// motor step display of Animation mode
 //  motor step display of Time Lapse mode

 void disp_step(double stp_data, int rowp, int curp)   // display the speed send value to slave device
{
  uint8_t length6 = strlen(Convert_float_to_String(stp_data));
  int c1 = curp + 1;  int c0 = curp + 2;
 // int c2 = curp1 + 2; int c3 = curp1 + 1;

   if (length6 == 3)
  {
	lcd_put_cur(rowp, curp);   ///0
	lcd_string_new("  ");
	lcd_put_cur(rowp, c0  );   // 2
	lcd_string_new(Convert_float_to_String(stp_data));
   }

   else if (length6 == 4)
  {
    lcd_put_cur(rowp, curp);   ///0
    lcd_string_new(" ");
    lcd_put_cur(rowp, c1  );   // 2
    lcd_string_new(Convert_float_to_String(stp_data));
  }
  else if (length6 == 5)
  {
    lcd_put_cur(rowp, curp  );     // 0
    lcd_string_new(Convert_float_to_String(stp_data));
  }

}


///////// steps and steps count display of animation mode

  void disp_steps(int32_t sc_data, int rowp1, int curp1)
 {
    uint8_t length7 = strlen(Convert_int_to_String(sc_data));
    int c2 = curp1 + 2; int c3 = curp1 + 1;

    if (length7 == 1) {
      lcd_put_cur(rowp1, curp1);     //0
      lcd_string_new("  ");
      lcd_put_cur(rowp1, c2);		//1
      lcd_string_new(Convert_int_to_String(sc_data));
    }
    else if (length7 == 2) {
      lcd_put_cur(rowp1, 0);     //0
      lcd_string_new(" ");
      lcd_put_cur(rowp1, c3);		//1
      lcd_string_new(Convert_int_to_String(sc_data));
    }
     else if (length7 == 3) {
      lcd_put_cur(rowp1, curp1);     // 0
      lcd_string_new(Convert_int_to_String(sc_data));
    }

    }


/////// Interval display of Time Lapse mode /////////

  void disp_interval( uint16_t int_data)
  {
    uint8_t length8 = strlen(Convert_int_to_String(int_data));
    if (length8 == 1) {
      lcd_put_cur(2, 11);
      lcd_string_new("  ");
      lcd_put_cur(2, 13);
     // lcd_int_to_str(port_data);
      lcd_string_new(Convert_int_to_String(int_data));
    }
    else if (length8 == 2) {
      lcd_put_cur(2, 11);
      lcd_string_new(" ");
      lcd_put_cur(2, 12);
   //   lcd_int_to_str(port_data);
      lcd_string_new(Convert_int_to_String(int_data));
    }
    else if (length8 == 3) {
      lcd_put_cur(2, 11);
    //  lcd_int_to_str(port_data);
      lcd_string_new(Convert_int_to_String(int_data));
    }
  }


//////// Expos display of Time Lapse Mode ///////////

  void disp_expos(float expo_data)   // display the speed send value to slave device
 {
   uint8_t length9 = strlen(Convert_float_to_String(expo_data));

   if (length9 == 3)
   {
     lcd_put_cur(3, 10);   ///0
     lcd_string_new(" ");
     lcd_put_cur(3, 11  );   // 2
     lcd_string_new(Convert_float_to_String(expo_data));
   }
   else if (length9 == 4) {
     lcd_put_cur(3, 10  );     // 0
     lcd_string_new(Convert_float_to_String(expo_data));
   }

   }

 /////// shots display of time lapse mode ///////


  void disp_time_shots(int64_t sht_data, int rowp2, int curp2)   // display the speed send value to slave device
 {
   uint8_t length10 = strlen(Convert_int_to_String(sht_data));
   int c3 = curp2 + 1; int c4 = curp2 + 2; int c5 = curp2 + 3; int c6 = curp2 + 4;

   if (length10 == 1) {
     lcd_put_cur(rowp2, curp2);   ///0
     lcd_string_new("    ");
     lcd_put_cur(rowp2, c6 );   // 2
    // lcd_int_to_str(port_data);
     lcd_string_new(Convert_int_to_String(sht_data));
   }

   else if (length10 == 2) {
     lcd_put_cur(rowp2, curp2);     //0
     lcd_string_new("   ");
     lcd_put_cur(rowp2, c5);		//1
  //   lcd_int_to_str(port_data);
     lcd_string_new(Convert_int_to_String(sht_data));
   }
   else if (length10 == 3) {
     lcd_put_cur(rowp2, curp2);     //0
     lcd_string_new("  ");
     lcd_put_cur(rowp2, c4);		//1
  //   lcd_int_to_str(port_data);
     lcd_string_new(Convert_int_to_String(sht_data));
   }

   else if (length10 == 4) {
     lcd_put_cur(rowp2, curp2);     //0
     lcd_string_new(" ");
     lcd_put_cur(rowp2, c3);		//1
  //   lcd_int_to_str(port_data);
     lcd_string_new(Convert_int_to_String(sht_data));
   }

   else if (length10 == 5) {
     lcd_put_cur(rowp2, curp2);     // 0
   //  lcd_int_to_str(port_data);
     lcd_string_new(Convert_int_to_String(sht_data));
   }

   }


  void disp_hms(uint8_t td_data, int cp)   // display hours, minute and second
 {
    length = strlen(Convert_int_to_String(td_data));

    if (length == 1)
    {
      lcd_put_cur(1, cp);   ///0
      lcd_string_new(" ");
      lcd_put_cur(1, cp+1  );   // 2
      lcd_string_new(Convert_int_to_String(td_data));
    }
    else if (length == 2) {
      lcd_put_cur(1, cp  );     // 0
      lcd_string_new(Convert_int_to_String(td_data));
    }

  }


  void disp_hms1(uint16_t td_data1, int cp1)   // display hours only
 {
	int cp2, cp3;
    length = strlen(Convert_int_to_String(td_data1));
    cp2 = cp1 + 1; cp3 = cp1+2;
    if (length == 1)
    {
      lcd_put_cur(1, cp1);   ///0
      lcd_string_new(" ");
      lcd_put_cur(1, cp3  );   // 2
      lcd_string_new(Convert_int_to_String(td_data1));
    }
    else if (length == 2)
     {
       lcd_put_cur(1, cp1);   ///0
       lcd_string_new("  ");
       lcd_put_cur(1, cp2  );   // 2
       lcd_string_new(Convert_int_to_String(td_data1));
     }
    else if (length == 3) {
      lcd_put_cur(1, cp1  );     // 0
      lcd_string_new(Convert_int_to_String(td_data1));
    }

  }


  void motor_step_disp(double tms_data)   // disply motor steps in timelapse mode
 {
   length = strlen(Convert_float_to_String(tms_data));
   if (length == 3)
   {
     lcd_put_cur(1, 0);        ///0
     lcd_string_new("    ");
     lcd_put_cur(1, 4);       // 2
     lcd_string_new(Convert_float_to_String(tms_data));
   }
   else if (length == 4) {
     lcd_put_cur(1, 0);     //0
     lcd_string_new("   ");
     lcd_put_cur(1, 3);		//1
     lcd_string_new(Convert_float_to_String(tms_data));
   }
   else if (length == 5) {
     lcd_put_cur(1, 0);     //0
     lcd_string_new("  ");
     lcd_put_cur(1, 2);		//1
     lcd_string_new(Convert_float_to_String(tms_data));
   }

   else if (length == 6) {
     lcd_put_cur(1, 0);     //0
     lcd_string_new(" ");
     lcd_put_cur(1, 1);		//1
     lcd_string_new(Convert_float_to_String(tms_data));
   }

   else if (length == 7) {
     lcd_put_cur(1, 0);     // 0
     lcd_string_new(Convert_float_to_String(tms_data));
   }

   }

  void disp_time_period(uint8_t tp_data, int c_p)   // display hours, minute and second
 {
    length = strlen(Convert_int_to_String(tp_data));

    if (length == 1)
    {
      lcd_put_cur(2, c_p);   ///0
      lcd_string_new(" ");
      lcd_put_cur(2, c_p+1  );   // 2
      lcd_string_new(Convert_int_to_String(tp_data));
    }
    else if (length == 2) {
      lcd_put_cur(2, c_p  );     // 0
      lcd_string_new(Convert_int_to_String(tp_data));
    }

  }




//  void eeprom_fun(void)
//  {
// 	       rec_buf[0] = 0;
// 	 	   rec_buf[1] = 0;
// 	 	   rec_buf[2] = 0;
// 	   // CS pin should default high
// 	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_SET);
// 	   // Enable write enable latch (allow write operations)
// 	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_RESET);
// 	   HAL_SPI_Transmit(&hspi1, (uint8_t *)&EEPROM_WREN, 1, 100);
// 	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_SET);
//
// 	   // Read status register
// 	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_RESET);
// 	   HAL_SPI_Transmit(&hspi1, (uint8_t *)&EEPROM_RDSR, 1, 100);
// 	   HAL_SPI_Receive(&hspi1, (uint8_t *)spi_buf, 1, 100);
// 	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_SET);
//
// 	   printf("Reg_status: %d", spi_buf[0]);
// 	   HAL_Delay(500);
//
// 	   // Print out status register
//
//
// 	   // Test bytes to write to EEPROM
// 	   spi_buf[0] = 'A';    //0xAB;
// 	   spi_buf[1] = 'B';    //0xCD;
// 	   spi_buf[2] = 'C';    //0xEF;
//
// 	   HAL_Delay(500);
//
// 	   // Set starting address
// 	   addr = 0x00;      //0x05;
// 	   addr1 = 0x00;
//
//
// 	   // Write 3 bytes starting at given address
// 	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_RESET);
// 	   HAL_SPI_Transmit(&hspi1, (uint8_t *)&EEPROM_WRITE, 1, 100);
//
// 	   HAL_SPI_Transmit(&hspi1, (uint8_t *)&addr, 1, 100);
// 	   HAL_SPI_Transmit(&hspi1, (uint8_t *)&addr1, 1, 100);
//
// 	   HAL_SPI_Transmit(&hspi1, (uint8_t *)spi_buf, 3, 100);
// 	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_SET);
//
// 	   // Clear buffer
// 	   //spi_buf[0] = 0;
// 	   //spi_buf[1] = 0;
// 	   //spi_buf[2] = 0;
//
// 	   // Wait until WIP bit is cleared
// 	   wip = 1;
// 	   while (wip)
// 	   {
// 	  // Read status register
// 	     HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_RESET);
// 	     HAL_SPI_Transmit(&hspi1, (uint8_t *)&EEPROM_RDSR, 1, 100);
// 	     HAL_SPI_Receive(&hspi1, (uint8_t *)spi_buf, 1, 100);
// 	     HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_SET);
//
// 	     // Mask out WIP bit
// 	     wip = spi_buf[0] & 0b00000001;
// 	   }
//
// 	   printf("Wip: %d", wip);
// 	   HAL_Delay(500);
//
// 	   // Read the 3 bytes back
//
// 	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_RESET);
// 	   HAL_SPI_Transmit(&hspi1, (uint8_t *)&EEPROM_READ, 1, 100);
//
// 	   HAL_SPI_Transmit(&hspi1, (uint8_t *)&addr, 1, 100);
// 	   HAL_SPI_Transmit(&hspi1, (uint8_t *)&addr1, 1, 100);
//
// 	   HAL_SPI_Receive(&hspi1, (uint8_t *)rec_buf, 3, 100);
// 	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_SET);
//
// 	   // Print out bytes read
// 	   printf("Buf0: %d", rec_buf[0]);
// 	   printf("Buf1: %d", rec_buf[1]);
// 	   printf("buf2: %d", rec_buf[2]);
// 	   HAL_Delay(500);
//
// 	   // Read status register
// 	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_RESET);
// 	   HAL_SPI_Transmit(&hspi1, (uint8_t *)&EEPROM_RDSR, 1, 100);
// 	   HAL_SPI_Receive(&hspi1, (uint8_t *)spi_buf, 1, 100);
// 	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_SET);
//
//
// 	   // Print out status register
// 	   printf("Reg_status: %d", spi_buf[0]);
// 	   HAL_Delay(1000);
// }



  void eeprom_write(uint8_t address, uint8_t byte_len)
  {
	   uint8_t length = byte_len;
	   lsb_addr = address;

	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_SET);
	        // Enable write enable latch (allow write operations)
	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_RESET);
	   HAL_SPI_Transmit(&hspi1, (uint8_t *)&EEPROM_WREN, 1, 100);
	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_SET);
//	       // Read status register
//	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_RESET);
//	   HAL_SPI_Transmit(&hspi1, (uint8_t *)&EEPROM_RDSR, 1, 100);
//	   HAL_SPI_Receive(&hspi1, (uint8_t *)spi_buf, 1, 100);
//	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_SET);

 	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_RESET);
 	   HAL_SPI_Transmit(&hspi1, (uint8_t *)&EEPROM_WRITE, 1, 100);
 	   HAL_SPI_Transmit(&hspi1, (uint8_t *)&msb_addr, 1, 100);
 	   HAL_SPI_Transmit(&hspi1, (uint8_t *)&lsb_addr, 1, 100);

 	   HAL_SPI_Transmit(&hspi1, (uint8_t *)spi_buf, length, 100);                //3, 100);
 	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_SET);

 	   HAL_Delay(4);
  }


  void eeprom_read(uint8_t address1,  uint8_t byte_len1)
  {
       uint8_t length1 = byte_len1;
       lsb_addr = address1;

	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_RESET);
	   HAL_SPI_Transmit(&hspi1, (uint8_t *)&EEPROM_READ, 1, 100);
	   HAL_SPI_Transmit(&hspi1, (uint8_t *)&msb_addr, 1, 100);
	   HAL_SPI_Transmit(&hspi1, (uint8_t *)&lsb_addr, 1, 100);
       HAL_SPI_Receive(&hspi1, (uint8_t *)rec_buf, length1, 100);
	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_SET);

//	   // Read status register
//	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_RESET);
//	   HAL_SPI_Transmit(&hspi1, (uint8_t *)&EEPROM_RDSR, 1, 100);
//	   HAL_SPI_Receive(&hspi1, (uint8_t *)spi_buf, 1, 100);
//	   HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin, GPIO_PIN_SET);

	   HAL_Delay(2);

  }

void    spi_buf_load(int32_t send_data)
	{
        dumy_data  = send_data;
		spi_buf[0] = (send_data >> 24) & 0xFF;
		spi_buf[1] = (send_data >> 16) & 0xFF;
		spi_buf[2] = (send_data >> 8)  & 0xFF;
		spi_buf[3] =  send_data & 0xFF;

	}

int32_t rec_buf_data()
	{

        rec_data  =  rec_buf[0] << 24;
        rec_data |= rec_buf[1] << 16;
        rec_data |= rec_buf[2] << 8;
        rec_data |= rec_buf[3];

        return(rec_data);
	}


void spi_buf_load2(int64_t send_data1)
	{
        dumy_data  = send_data1;
        memset(spi_buf, 0, sizeof(spi_buf));

        spi_buf[0] = (send_data1 >> 56)  & 0xFF;
		spi_buf[1] = (send_data1 >> 48)  & 0xFF;
		spi_buf[2] = (send_data1 >> 40)  & 0xFF;
		spi_buf[3] = (send_data1 >> 32)  & 0xFF;
		spi_buf[4] = (send_data1 >> 24)  & 0xFF;
		spi_buf[5] = (send_data1 >> 16)  & 0xFF;
		spi_buf[6] = (send_data1 >> 8)   & 0xFF;
		spi_buf[7] =  send_data1 & 0xFF;

	}

  void spi_buf_load1()
{
	 // dumy_data  = send_data1;
	  memset(spi_buf, 0, sizeof(spi_buf));
	  spi_buf[0] =  (motor_limit >> 56) & 0xFF;
	  spi_buf[1] =  (motor_limit >> 48) & 0xFF;
	  spi_buf[2] =  (motor_limit >> 40) & 0xFF;
	  spi_buf[3] =  (motor_limit >> 32) & 0xFF;
	  spi_buf[4] =  (motor_limit >> 24) & 0xFF;
	  spi_buf[5] =  (motor_limit >> 16) & 0xFF;
	  spi_buf[6] =  (motor_limit >> 8)  & 0xFF;
	  spi_buf[7] =  motor_limit & 0xFF;
	  spi_buf[8] =  limit_status;

}




int64_t rec_buf_data1()
	{

        rec_data1  = rec_buf[0]  << 56;
        rec_data1 |= rec_buf[1]  << 48;
        rec_data1 |= rec_buf[2]  << 40;
        rec_data1 |= rec_buf[3]  << 32;
        rec_data1 |= rec_buf[4]  << 24;
        rec_data1 |= rec_buf[5]  << 16;
        rec_data1 |= rec_buf[6]  << 8;
        rec_data1 |= rec_buf[7];

        return(rec_data1);
	}

//////////////////////////


 void data_display()
 {
  	 disp_motor_step(motor_count);
 	 disp_spd_val(motor_speed_pot);
 	 disp_dmp_val(motor_damp_dis);
 	 disp_send_spd_val(speed_send);
 	 lcd_put_cur(0, 15);  lcd_send_data(direction);
  }


  void get_dam_pot_value()
 {
    adc_sum = 0;
    for (ac = 0; ac < 50; ac++)
    {
 	damp_value = get_adc_value(ADC_CHANNEL_1);             //(ADC_CHANNEL_2);
     adc_sum = adc_sum + damp_value;
    }
     damp_value = 0;
     damp_value = adc_sum / 50;

     if (damp_value > 4000) { damp_value = 4000;}

    motor_damp_pot = map( damp_value,  0, 4000, 145, 10);
    motor_damp_dis = map( damp_value,  0, 4000, 1, 100);

 //	motor_damp_pot = map( damp_value,  0, 4000, 0, 100);

 //	motor_damp_pot1 = (damp_value - 1) * (100 - 1) /  ( 4020 - 1) + 1;

 }


 void get_spd_pot_value()
 {
       adc_sum = 0;
 	  for (ac = 0; ac < 50; ac++)
 	   {
 		spd_value = get_adc_value(ADC_CHANNEL_2);               //(ADC_CHANNEL_1);
 	    adc_sum += spd_value;
 	   }
      spd_value = 0;
 	  spd_value = adc_sum / 50;

 	  if (spd_value > 4000) { spd_value = 4000;}
       motor_speed_pot = map( spd_value,  0, 4000, 1, 100);

 	//	motor_speed_pot = (spd_value - 1) * (100 - 1) /  ( 4020 - 1) + 1;

 }


    // Display the arrow on the LCD screen

 void arrow_down()
 {
	 j_toggle1 = 0;
     lcd_put_cur(row_pos,0); lcd_string_new(" ");
     row_pos++; if (row_pos >= 4) { row_pos = 0; }
     lcd_put_cur(row_pos,0); lcd_string_new(">");

 }

 void arrow_up()
 {
	 j_toggle1 = 0;
     lcd_put_cur(row_pos,0); lcd_string_new(" ");
     row_pos--;  if(row_pos == 255) { row_pos = 3; }
     lcd_put_cur(row_pos,0); lcd_string_new(">");

 }


void get_row_pos()
{
    read_joystic();
	if ((y_value >= 3000 ) && (y_value <= 4095) && ( j_toggle1 == 1) )
	{
		j_toggle1 = 0;
		arrow_down();
		cursor_pos++;
	}

	else if ((y_value <= 900) && (y_value >= 0 ) && ( j_toggle1 == 1))
	{
		j_toggle1 = 0;
		arrow_up();
		cursor_pos--;
	}

	else if ((y_value <= 2400) && (y_value > 1300))
	{
		 j_toggle1 = 1;
	}
   HAL_Delay(50);
}


 /////////////////////////////////////////////////////////////////////////////


 //************ RECORDING FUNCTION  **************//

void rec_display()
{
	 button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
     get_dam_pot_value();
	 get_spd_pot_value();
	 load_buffer();
	 transmit_data();
	 rx_data();
	 //pree_step_cal();
	 motor_count = motor_step;
	 data_display();
	 read_joystic();  //HAL_Delay(1);
	 /////
	 lcd_put_cur(0, 0);
	 lcd_int_to_str(time_duration);
}


  void  Recording_fun()
  {

   //working_mode = '';
	working_mode = 'R';
	//load_buffer();
//	transmit_data();
//	d_index = 0;
//	s_index = 0;
//	m_index = 0;

	callback_mode = 'R';   // For timer callback function
	time_duration = 0;
	direction = 'S';
	//load_buffer();
	//transmit_data();
	rx_data();
	HAL_Delay(150);
	lcd_clear();
	lcd_put_cur(1, 0); lcd_string_new("Vodeo Rec");
    button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
//	read_joystic(); HAL_Delay(5);
	load_buffer();
	transmit_data();
	rx_data();
    rec_home = rx_motor_step;

    AM = 0;   // clear the millisecond counter
   HAL_TIM_Base_Start_IT(&htim1);  // start timer

	while ( button != 0  && time_duration < 89 )
	{
	 	 button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	     get_dam_pot_value();
		 get_spd_pot_value();
		 load_buffer();
		 transmit_data();
		 rx_data();
		 //pree_step_cal();
		 motor_count = motor_step;
		 data_display();
		 read_joystic();  //HAL_Delay(1);
		 /////
		 lcd_put_cur(0, 0);
		 lcd_int_to_str(time_duration);
         ////////

	//	rec_display();
		read_joystic();
	   if ((x_value <= 2400 ) && ( x_value >= 1000))
		 {
			//read_joystic();
			 speed_send = 15;
			 direction = 'S';
			// rec_display();
		 }

	   else if  ((x_value <= 900) && ( x_value >= 0 ))  //// 1st limit increment
		 {
			// read_joystic();
			 speed_send = map(x_value,  1800, 0, 15, 1550);
			 speed_send = speed_send * motor_speed_pot / 100;
			 direction = 'F';
			// rec_display();
		 }

	   else if  ((x_value >= 3000) && (x_value < 4095)) // 1st limit decrement
		 {
			 //read_joystic();
			 if(x_value >= 4000) {x_value = 4000;}
			 speed_send = map( x_value,  2200, 4000, 15, 1550) * motor_speed_pot / 100;
			 direction = 'B';
			 //rec_display();
		 }
	}

//	HAL_TIM_Base_Stop_IT(&htim1);
	direction = 'S';
	load_buffer();
	transmit_data();
	rx_data();
	//rec_home = rx_motor_step;

	if (time_duration >= 89 )
	{
		lcd_put_cur(2, 0); lcd_string_new("  Memory Full   ");
		button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
		while( button != 0)
		{
		  HAL_Delay(5); button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
		}

	}

	prev_row_pos = 1;
	time_period = time_duration;   // total time of recording

	spi_buf[0] = time_period;
	eeprom_write(32, 1);          // store Recording time  in EEPROM

    live_fun();   // Back to live function

  }




  	  //************ PLAYBACK FUNCTION  **************//

 void Playback_fun()
  {

	     HAL_TIM_Base_Stop_IT(&htim1);
	     d_index = 0;
	     s_index = 0;
    //  working_mode = 'P';
   //  transmit_data();
	//    callback_mode = 'P';  // for timer callback function
	     exit_flag = 1;
	 //   time_duration = 0;
		direction = 'S';
		load_buffer();
		transmit_data();
		rx_data();
		HAL_Delay(150);
		lcd_clear();
		lcd_put_cur(1, 0); lcd_string_new("Playback");
		disp_motor_step(motor_step);
	    lcd_put_cur(2, 0); lcd_string_new("Go Home? N<- ->Y");

	  while (exit_flag == 1)       //((x_value <= 2900) && (x_value > 1000))
	   {
	      read_joystic();
	      if ((x_value >= 3000 ) && (x_value <= 4095) )   // N is pressed
	         {
	    	   lcd_put_cur(2, 0);  lcd_string_new("               ");
	    	   working_mode = 'N';
			   load_buffer();
			   transmit_data();
			//   rx_data();
			//  disp_motor_step(motor_step);
			   exit_flag = 0;
	         }

	      else if ((x_value <= 800) && (x_value >= 0 ))        // Y is pressed
	         {
	           lcd_put_cur(2, 0);  lcd_string_new("   Go To Home?   ");
	           working_mode = 'Y';
	    	   load_buffer();
	    	   transmit_data();
	    	   rx_data();
	     	   disp_motor_step(motor_step);
	     	   transmit_data();
	     	   rx_data();
	     	   HAL_Delay(100);
	     	   transmit_data();
	     	   rx_data();
	    	   while(  motor_running != 0  )  //&& motor_move_step !=  rx_motor_step)
	    	    	//   while ( rx_motor_step > rec_home )
	    	    	{
	    		     // load_buffer();
	    	    	  transmit_data();
	    	    	  rx_data();
	    	    	  disp_motor_step(motor_step);
	    	    	  HAL_Delay(6);
	    	    	}

	    	      button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	    	      lcd_put_cur(2, 0);  lcd_string_new("press OK to Play");

	    	   while ( button != 0  )
	    	  	   {
	    	  	      button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	    	          HAL_Delay(10);
	    	  	   }

	    	   HAL_Delay(200);   // De-bounce delay
	           exit_flag = 0;
	         }
//	     	 working_mode = 'P';
//	     	 direction = 'S';
//	     	 load_buffer();
//	     	 transmit_data();
//	     	 rx_data();
//	     	 disp_motor_step(motor_step);
	   }

	    time_duration = 0;
     	lcd_put_cur(2, 0);  lcd_string_new("                ");
	  	lcd_put_cur(2, 0);  lcd_string_new("Time:");
	  	disp_time_period(time_period, 9);
	  	lcd_put_cur(2, 11);  lcd_string_new("s");
	  	lcd_put_cur(2, 8);  lcd_string_new("/");
	  	disp_time_period(time_duration, 6);
	  	working_mode = 'P';    // code to start Play back mode
	  	load_buffer();
	  	transmit_data();
	    AM = 0;   // clear the millisecond counter
        rx_data();
        callback_mode = 'P';
        HAL_TIM_Base_Start_IT(&htim1);  // start timer
        button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
        while (time_duration <= time_period && button != 0 )
          {
        	disp_time_period(time_duration, 6);
        	load_buffer();
        	transmit_data();
        	rx_data();
        	button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
        	// motor_count = motor_step;
        	// data_display();
        	// lcd_put_cur(0, 0);
        	//lcd_int_to_str(time_duration);
        	disp_motor_step(motor_step);
 	      }

        HAL_TIM_Base_Stop_IT(&htim1);
        direction = 'S';
        working_mode = 'G';    // code to start Play back mode
        load_buffer();
        transmit_data();
        lcd_put_cur(2, 0); lcd_string_new("press OK to exit");
        //HAL_Delay(150);
        button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);

        while (button != 0 )
        {
		  button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	      HAL_Delay(10);
        }
          time_duration = 0;
          live_fun(); /// call again to the live function
    }


 	 	 //************ VIDEOLOOP FUNCTION  **************//

 void Videoloop_fun()
  {

	working_mode  = 'V';
	direction = 'S';
	send_motor_limit();
	load_buffer();
	transmit_data();
    HAL_Delay(200);
    transmit_data();
    load_buffer();
    lcd_clear();

	switch(man_unl_mode)
   {
	case 'U':   // Unlimited mode
			lcd_put_cur(1, 0); lcd_string_new("In Man Mode Only");
			HAL_Delay(150);      // button de-bounce dealy
			button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
    	while(button != 0 )
    	{
			button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
			transmit_data();
			HAL_Delay(1);
		}


    break;

	case 'M':     // Limited mode
		    transmit_data();
			rx_data();
			motor_count = motor_step;
        	lcd_put_cur(1, 0);   lcd_string_new("VideoLoop");
        	data_display();
        	lcd_put_cur(1, 15);  lcd_send_data(direction);           //float_to_string(motor_limit);
        	button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);

    	while(button != 0 )
	     {
			 get_dam_pot_value();
			 get_spd_pot_value();
			 speed_send = motor_speed_pot * 15.50;
		//	 data_transmit();
			 load_buffer();
			 transmit_data();
			 rx_data();
			 pree_step_cal();
			 motor_count = motor_step;
			 data_display();
			 button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	     }



    break;
   }

		current_motor_step = motor_step;    // motor current position
		prev_row_pos = 3;
		live_fun(); /// call again to the live function

 }


     //************ FREE-RIDE FUNCTION  **************//


void free_ride_fun()
{
    working_mode = 'F';
    direction = 'S';
    send_motor_limit();
    load_buffer();
    transmit_data();
	fr_flag = 0;
	HAL_Delay(250);
	transmit_data();
	lcd_clear();


 // if (man_flag == 1)   // //
  switch(man_unl_mode)
  {
  case 'M':             //   man_unl_mode == 'm'  Limited mode
	  	   transmit_data();
	  	   rx_data();
	  	   motor_count = motor_step;
	  	   lcd_put_cur(1, 0); lcd_string_new("FreeRide  ");
//    lcd_put_cur(0, 10);  float_to_string(motor_limit);  // show to limit on list line
	  	   data_display();

	  	   button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
     while (button != 0 )
	   {
		     get_dam_pot_value();
			 get_spd_pot_value();
		     load_buffer();
		     transmit_data();
			 rx_data();
			 pree_step_cal();
			 motor_count = motor_step;
			 data_display();
			 button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
			 read_joystic();
			// speed_send = map( x_value,  2100, 4000, 15, 1550) * motor_speed_pot / 100;

	 	    if ((x_value <= 2098 ) && ( x_value >= 1902))
			  {
			    speed_send = 15;
			    direction = 'S';
			  }

	 	    else if ((x_value <= 1900) && ( x_value >= 0 ))  //// 1st limit increment
	 	      {
	 	    	speed_send = map(x_value,  1800, 0, 15, 1550);
	 	    	speed_send = speed_send * motor_speed_pot / 100;
       		    direction = 'F';
	   		  }

   	     else if ((x_value >= 2100) && (x_value < 4095)) // 1st limit decrement
	 	      {
	 	    	if(x_value >= 4000) {x_value = 4000;}
	 	    	speed_send = map( x_value,  2200, 4000, 15, 1550) * motor_speed_pot / 100;
 	            direction = 'B';
	 	      }

	    //current_motor_step = motor_step;  // current position of the motor
	  }

   break;

                                                             //else if (man_flag == 0)   //  unlimited mode
  case 'U':    //   man_unl_mode == U   Unlimited mode

	  	  	  transmit_data();
	  	  	  rx_data();
	  	  	  motor_count = motor_step;
	  	  	  lcd_put_cur(1, 0); lcd_string_new("FreeRide  ");
	  	  	  data_display();

	  	  button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	  while (button != 0 )
	      {
	        	 get_dam_pot_value();
	        	 get_spd_pot_value();
	        	 data_transmit();
	        	 rx_data();
				 motor_count = motor_step;
				 data_display();
				 button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
				 read_joystic();
	         if ((x_value <= 2400 ) && ( x_value >= 1902))
	           {
	        	 speed_send = 15;
	        	 direction = 'S';
	           	 HAL_Delay(1);
	           }

	    else if ((x_value <= 900) && ( x_value >= 0 ))  //// 1st limit increment
	      	   {
	      		direction = 'F';
	            speed_send = map( x_value,  1800, 0, 15, 1550) * motor_speed_pot / 100;
	        	HAL_Delay(1);
	     	   }

	     else if ((x_value >= 3000) && (x_value < 4095)) // 1st limit decrement
	      		{
	    	    direction = 'B';
	    	    if(x_value >= 4000) {x_value = 4000;}
	   	      	speed_send = map( x_value,  2200, 4000, 15, 1550) * motor_speed_pot / 100;
	       		HAL_Delay(1);
	      		}
	    }

	  break;

    }

	  lcd_clear();
 }




//void back_dis()
//{
//  //  int b = 1;
//	row_pos1 = 0;
//	lcd_clear(); //lcd_put_cur(0, 1); lcd_string_new("[Back]");
////	lcd_put_cur(0,0); lcd_string_new(">");
//	lcd_put_cur(0,0); lcd_string_new(">");
//	lcd_put_cur(0, 1); lcd_string_new("[Back]");
//   // pressed = 1;
//	HAL_Delay(300);
//    button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
//	read_joystic(); //HAL_Delay(5);
//
//  while ( button != 0 &&  y_value  <= 3000  )
//   {
//	   button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
//       read_joystic(); HAL_Delay(1);
//	}
//
//   if (button == 0)
//	   {
//		lcd_clear();
//		live_flag = 0;
//		pressed = 0;
//		prev_row_pos = 0;
//		auto_unl();  /// call back to auto unlimted limt function
//	   //HAL_Delay(0);
//	  }
//  else if ( button == 1)
//  {
//   lcd_clear();
//   row_pos = 4;
//   pressed = 0;
////  // prev_row_pos = 0;
//////   lcd_put_cur(0, 1);lcd_string_new("[Free Ride]");lcd_put_cur(1, 1); lcd_string_new("[Recording]");
//////   lcd_put_cur(2, 1);lcd_string_new("[Playback]  0s");lcd_put_cur(3, 1); lcd_string_new("[VideoLoop]");
//////   lcd_put_cur(0,0);lcd_string_new(">");
//////   row_pos = 0;
////
//  }
//
//}



//***********************************************************//

				//******** LIVE FUNCTION   ***********//

//***********************************************************//


void live_screen2()
{
		lcd_clear();
		lcd_put_cur(0,0); lcd_string_new(">");
		lcd_put_cur(0, 1); lcd_string_new("[Back]");
		HAL_Delay(300);
	    button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
		//read_joystic();
	  while ( button != 0 &&  cursor_pos == 5 )
	   {
		   button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	       get_row_pos(); //read_joystic();
	       HAL_Delay(1);
	    }
	  if (button == 0 )
	    {
		 lcd_clear();
		 live_flag = 0;
		 pressed = 0;
		 prev_row_pos = 0;
		 auto_unl();  /// call back to auto unlimted limt function
	    }
	  if ((cursor_pos <= 4 ) || (cursor_pos >= 6))
			{ cursor_pos = 1; prev_row_pos = 0; }
	   pressed = 0;
	  lcd_clear();
	  live_fun();
}



  void live_fun()                  //FRPV_fun()
{
/////
	  eeprom_read(32, 1);
	  time_period =  rec_buf[0];    // Read Recording time from EEPROM
	  callback_mode = 'N';
      lcd_clear();
    //  working_mode = 'G';  27/3
	//  direction = 'S';     27/3
	//  data_transmit();     27/3
      live_flag = 1;
    //  pressed = 1;
      exit_flag = 0;
    //  row_pos1 = 0;
      prev_row_pos = 0;
      cursor_pos = 1;
    //  unl_flag = 1;
  while (live_flag == 1)
  {
	  working_mode = 'G';
	  direction = 'S';
	  data_transmit();
	  HAL_Delay(200);
	  row_pos = prev_row_pos;
	  lcd_put_cur(0, 1);lcd_string_new("[Free Ride]");lcd_put_cur(1, 1); lcd_string_new("[Recording]");
	  lcd_put_cur(2, 1);lcd_string_new("[Playback]    s"); disp_time_period(time_period, 13);
	  lcd_put_cur(3, 1); lcd_string_new("[VideoLoop]");
      lcd_put_cur(row_pos,0);lcd_string_new(">");
    //  HAL_Delay(200);
      button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);

       while( button != 0 )
     {
	      button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	   // read_joystic();
          get_row_pos();
          if ((cursor_pos >= 5) || (cursor_pos <= 0)) { cursor_pos = 5; live_screen2();}
          HAL_Delay(1);
     }

      /////////////////

	   switch(row_pos)
	   {
	   	 case 0:
	   	 free_ride_fun();
	   	 break;

	   	 case 1:
	   	 Recording_fun();
	     prev_row_pos = 1;
	   	 break;

	   	 case 2:
	   	 Playback_fun();
	   	 prev_row_pos = 2;
	   	 break;

	   	 case 3:
	   	 Videoloop_fun();
	     prev_row_pos = 3;
	   	 break;

	   	 case 4:
	   	 prev_row_pos = 0;
	   	// pressed = 1;
         break;
        }
	 //  prev_row_pos = row_pos;
   // if (exit_flag == 1)  { HAL_Delay(100); auto_unl(); }  // call back to auto unl function for loop
       }
    lcd_clear();
    prev_row_pos = 0;
   // HAL_Delay(50);
 }


  //***********************************************************//

       	//******** TIMELAPSE FUNCTION  ***********//

  //------------------ START --------------------//

  void time_stop()  // stop the time lapse mode
  {
	   hms_start = 0;
	   callback_mode = 'N';  //
	   HAL_TIM_Base_Stop_IT(&htim1);
	   time_shots_count = 0;
	   direction = 'S';
	   load_buffer();    //send_step();
//	   send_buffer[26] =  0;
	   transmit_data();
	 //  send_step();
	 //  transmit_data();
	   lcd_clear();
	   lcd_put_cur(1, 0);		lcd_string_new(" TimeLapse End  ");
	   lcd_put_cur(2, 3);		lcd_string_new("OK  To Exit");
	   lcd_put_cur(2, 2);		lcd_send_data(34);
	   lcd_put_cur(2, 5);		lcd_send_data(34);
//	   working_mode = 'G'; // Working mode is G
//	   load_buffer();
//	   send_buffer[26] =  0;  // stop to shot clicking byte
	   transmit_data();
	   HAL_Delay(150);
	   button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
 	   while (button != 0 )
	     {   button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin); }
//	   HAL_Delay(10);
	   working_mode = 'G'; // Working mode is G
	   load_buffer();
	   send_buffer[26] =  0;  // stop to shot clicking byte
	   transmit_data();
	   lcd_clear();    prev_row_pos = 3;
	   time_screen2();
  }

  void time_start_disp()       // display function
  {
  //	lcd_clear();
  //disp_motor_step(motor_step);
  	time_duration_cal();
  	lcd_put_cur(2, 0);		lcd_string_new("Shot:");
  	disp_time_shots(time_shots_count, 2, 5 );
  	lcd_put_cur(2, 10);		lcd_string_new("/");
  	lcd_put_cur(2, 11);     lcd_int_to_str(time_shots);
//	transmit_data();
//  	rx_data();
//  	motor_step_disp(motor_step);
     //HAL_Delay(1000);
  }


  void time_cancle_stop()   // Time lapse STOP, PAUSE , CANCLE function
  {
	  	uint8_t spc = 1;
	  	lcd_clear();
	  	lcd_put_cur(1, 2); lcd_string_new("PAUSE/STOP ?");
	  	lcd_put_cur(2, 3); lcd_string_new("< CANCEL >");
	  	j_toggle1 = 1;
	  	HAL_Delay(250);
	  	currentMillis = 0;

//	    button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
//      while ( j_toggle1 == 1)
//     {
//    	button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
//    	read_joystic();
//      while ((x_value <= 2400) && (x_value > 1000) && j_toggle1 == 1)
         while( j_toggle1 == 1 && time_duration != 0 )
       {
           read_joystic();
    	   button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
    	   if ( button == 0) { j_toggle1 = 0;}
    	// currentMillis = HAL_GetTick();
    	   lcd_put_cur(2, 15); lcd_int_to_str(currentMillis);
    	   if( currentMillis >= 3) { j_toggle1 = 0; spc = 1;}

        if ((x_value >= 3000 ) && (x_value <= 4095))
  		 {
           spc--;
   		   if ( spc <= 0 ) {spc = 3;}
   		   HAL_Delay(200);
   		   currentMillis = 0;
  		 }

  		else if ((x_value <= 900) && (x_value >= 0 ))      // && ( j_toggle1 == 1))
  		 {
  		   spc++;
  		   if (spc >= 4) {spc = 1;}
  		   HAL_Delay(200);
  		   currentMillis = 0;
  		 }

         if (spc == 1)        { lcd_put_cur(2, 3);  lcd_string_new("< CANCEL >");}
         else if  (spc == 2 ) { lcd_put_cur(2, 3);  lcd_string_new("<  STOP  >");}
         else if  (spc == 3 ) { lcd_put_cur(2, 3);  lcd_string_new("< PAUSE  >");}
       }

        if( time_duration == 0) { spc = 2;}  // Time finished to go stop

      switch(spc)
     {
      case 1: //  Cancel and do nothing

    	  break;

      case 2:  // stop the operation of timelapse function
          time_stop();

    	  break;

      case 3:  // pause the funciton
    	  HAL_TIM_Base_Stop_IT(&htim1);  // Stop timer
    	  send_buffer[33] =  'P';  //  Pause the operation
          transmit_data();
          lcd_clear();
          lcd_put_cur(1, 0); lcd_string_new("TIMELAPSE PAUSED");
          lcd_put_cur(2, 0); lcd_string_new(" press OK to go ");
          button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
        while (button != 0)
         { button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin); }

          HAL_TIM_Base_Start_IT(&htim1);  // start timer
          send_buffer[33] = 'R';  // Resume the operation
          transmit_data();

          break;

      }
      currentMillis = 0;
      lcd_clear();
      time_start_disp();
    //disp_motor_step(motor_step);
 }

void time_shot_count()   // Time shot count
{
	HAL_TIM_Base_Start_IT(&htim1);  // start timer
	send_buffer[26] =  3;   // start shot clicking in side the expose time
	transmit_data();

	while ( inter < time_chek_out )         //time_duration > time_chek_out )
  {
	 button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	 if(button == 0 ) { time_cancle_stop(); }

//	 if(time_mode == 'S') { if (expos_count >= expos_data) { motor_step_disp(motor_count);} }
	 if (time_mode == 'C') {  motor_step_disp(motor_step); }

	 if (expos_count <= expos_data )    //19  // one second delay for between interval count
	  {
		lcd_put_cur(0, 14); lcd_string_new("S");
		lcd_put_cur(0, 5); float_to_string(expos_count);
	  }
	else if (expos_count > expos_data )
	  {
		lcd_put_cur(0, 14); lcd_string_new(" ");
		lcd_put_cur(0, 5); lcd_string_new("  ");
	  }
	 	time_duration_cal();
		get_spd_pot_value();
		speed_send = motor_speed_pot * 15.50;
		load_buffer();
		transmit_data();
		rx_data();
	}

	if(time_mode == 'S') { motor_step_disp(motor_count);}
	expos_count = 0;
	inter = 0;
	currentMillis = 0;
//	HAL_TIM_Base_Stop_IT(&htim1);  // stop timer
	time_duration_cal();
 }




void time_start()  // Timelapse star function to run the motor according to set parameters
{
	hms_start = 1;
	AM = 0;    // clear the millisecond counter.
	callback_mode = 'T';  // for timer callback function
	//if (time_mode == 'C')      { working_mode = 'T'; }  // for continute mode
	//else if (time_mode == 'S') { working_mode = 't'; }  // for sds mode
	direction = motor_move;
	send_buffer[27] =  (time_shots >> 8)  & 0xFF;
	send_buffer[28] =  time_shots & 0xFF;
	send_buffer[29] =  (interval_val >> 8)  & 0xFF;
	send_buffer[30] =  interval_val & 0xFF;
	send_buffer[31] =  (expos_data >> 8)  & 0xFF;
	send_buffer[32] =   expos_data & 0xFF;
	load_buffer();
	transmit_data();
	rx_data();
	motor_count = motor_step;  // Current position of the motor
	lcd_clear();
	time_start_disp();
	motor_step_disp(motor_step);



	switch(time_mode)  //      (man_unl_mode)
  {
	case'C':  //  time Mode is continuous
		working_mode = 'T';
		transmit_data();
		time_chek_out = expos_data/10;   // count up to Expose value
      if( man_unl_mode == 'U')   //  Unlimited Mode
     {
         while (time_shots_count != time_shots)                //
	 	{
		  time_shot_count();
		  time_chek_out = interval_val;
		  time_shots_count++;
		  disp_time_shots(time_shots_count, 2, 5 );
		}
	    HAL_Delay(500);
	  }

     else if( man_unl_mode == 'M')          // (time_mode == 'C')        // Continuous Mode
     {
    	send_motor_limit(); // Send the Set limit
    	direction = motor_move;
    	load_buffer();
    	transmit_data();
      while (time_shots_count != time_shots )  // &&  rx_motor_step != motor_limit )               //
	   {
		  time_shot_count();
		  time_chek_out = interval_val;
		  time_shots_count++;
		  disp_time_shots(time_shots_count, 2, 5 );
 	   }
       HAL_Delay(50);
     }

    break;

/////////////////////////////

	case'S':  // Time mode is SDS
		working_mode = 't';
		transmit_data();
		time_chek_out = expos_data/10;   // count up to Expose value
     if( man_unl_mode == 'U')     //  Unlimited Mode
      {
    	// time_duration = time_duration - interval_val; // To make 00 at the end of shot
    	// time_chek_out = interval_val;  // count up to Interval
    	// rx_data();
       	 while (time_shots_count != time_shots)                //
     	{
    	  rx_data();
		  local = (time_step_u * 10);
      	  if(motor_move == 'R') {
      		  motor_move_step = ( rx_motor_step + local);
      		  motor_count = motor_count + time_step_u;

      	    }
   		  else if(motor_move == 'L') {
   			  motor_move_step = ( rx_motor_step - local);
              motor_count = motor_count - time_step_u;

   		    }
          if (motor_move_step < 0)       { send_step_status =  '-'; motor_move_step = (labs(motor_move_step)); }  // find out the limit is in negative or in possitive direction
 	      else if (motor_move_step >= 0) { send_step_status =  '+';  }
             send_step();
    	     time_shot_count();
    	     time_chek_out = interval_val;  // count up to Interval
    	     time_shots_count++;
    	     disp_time_shots(time_shots_count, 2, 5 );
        }
       HAL_Delay(500);
       }

     else if( man_unl_mode == 'M')     //  Limited Mode ( Limits are set )
      {
    	 // time_duration = time_duration - interval_val; // To make 00 at the end of shot
    	 // time_chek_out = interval_val;  // count up to Interval
    	 // rx_data();
    	 send_motor_limit(); // Send the Set limit
    	 direction = motor_move;
    	 load_buffer();
    	 transmit_data();

	   while (time_shots_count != time_shots)                //
		{
		  transmit_data();
		  rx_data();
		  local = (time_step_m * 10);
		  if(motor_move == 'R') {
			  motor_move_step = ( rx_motor_step + local);
			  motor_count = motor_count + time_step_m;

			}
		  else if(motor_move == 'L') {
			  motor_move_step = ( rx_motor_step - local);
		      motor_count = motor_count - time_step_m;

			}
		  if (motor_move_step < 0)       { send_step_status =  '-';  }  // find out the limit is in negative or in possitive direction
		  else if (motor_move_step >= 0) { send_step_status =  '+';  }
		     motor_move_step = (labs(motor_move_step));
			 send_step();
			 time_shot_count();
			 time_chek_out = interval_val;  // count up to Interval
			 time_shots_count++;
			 disp_time_shots(time_shots_count, 2, 5 );
		 }
         HAL_Delay(500);
       }

	 break;
  }
}



void time_duration_cal()   // Time Lapse total time durationn calculation
{

    uint32_t rest_data = 0;
 //	time_duration =  interval_count * time_shots;              // interval_val * time_shots;
	time_hour   = time_duration / 3600;          // calculate the hour
    rest_data   = time_duration % 3600;
	time_minute = rest_data  / 60;        // calculate the minute
    time_second = rest_data % 60;        // calculate the minute

switch(hms_start)
{
 	 case 0:

		if (time_hour <=  0 )
		{
		lcd_put_cur(1, 12);		lcd_string_new("m");
		lcd_put_cur(1, 15);		lcd_string_new("s");
		disp_hms( time_minute, 10);
		disp_hms( time_second, 13);
		}

		else if ((time_hour >= 1) && (time_hour <=99))
		{
		lcd_put_cur(1, 12);	lcd_string_new("h");
		lcd_put_cur(1, 15);	lcd_string_new("m");
		disp_hms( time_hour,   10);
		disp_hms( time_minute, 13);
		}

		else if (time_hour >= 100)
		{
		lcd_put_cur(1, 13);	lcd_string_new("h");
		disp_hms1( time_hour, 10);
		disp_hms( time_minute, 14);
		}

		break;
 	 case 1:
 		if (time_hour <=  0 )
 		{
 		lcd_put_cur(1, 12);		lcd_string_new("m");
 		lcd_put_cur(1, 15);		lcd_string_new("s");
 		disp_hms( time_minute, 10);
 		disp_hms( time_second, 13);
 		}

 		else if ((time_hour >= 1) && (time_hour <=9 ))
 		{
 		lcd_put_cur(1, 9);	lcd_string_new("h");
 		lcd_put_cur(1, 12);	lcd_string_new("m");
 		lcd_put_cur(1, 15);	lcd_string_new("s");
 		disp_hms( time_hour,   7);
 		disp_hms( time_minute, 10);
 		disp_hms( time_second, 13);

 		}

 		else if ((time_hour >= 10) && (time_hour <=99 ))
		{
		lcd_put_cur(1, 10);	lcd_string_new("h");
		lcd_put_cur(1, 13);	lcd_string_new("m");
		disp_hms( time_hour,   8);
		disp_hms( time_minute, 11);
		disp_hms( time_second, 14);
		}

 		else if (time_hour >= 100)
 		{

 		lcd_put_cur(1, 11);	lcd_string_new("h");
 		lcd_put_cur(1, 14);	lcd_string_new("m");
 	//	disp_hms( time_hour,   8);
 		disp_hms1( time_hour, 8);
 		disp_hms( time_minute, 12);
 		disp_hms( time_second, 15);

 		}
	break;
}
}


void time_step_cal()   // Time Lapse step calculation
{
  transmit_data();
  rx_data();
  switch(man_unl_mode)
  {
   case 'U':         // Unlimited Mode
		  read_joystic();
		if (x_value <= 900)
		 {
		    if (time_step_u < 360.0 )  { time_step_u += 0.5;}     //lcd_put_cur(2,10);    float_to_string(time_step);
		    disp_step(time_step_u, 2, 10);  jx_delay();
		 }
        else if (x_value >= 3000)
         {
			if ( time_step_u > 0.5)   { time_step_u -= 0.5; }   //lcd_put_cur(2,10); float_to_string(time_step);
			disp_step(time_step_u, 2, 10);  jx_delay();
         }

	break;

   case 'M':     // Man Mode fixed limt mode
	   switch(motor_move)
	   {
	     case'R':   // Motor direction to Right

	    	if (rx_motor_step == motor_limit)  {  time_step_m = 0.0; }
	    	else if(rx_motor_step != motor_limit)
	    	{
             time_step_m = ( ((motor_limit - rx_motor_step) / (time_shots - 1) ) / 10.0 );
	    	}
	    	disp_step(time_step_m, 2, 10);

		    //time_step = ((motor_limit / (time_shots - 1))) / 10.0;
	 	   //disp_step(time_step, 2, 10);
	     break;

	     case'L':   // Motor Direction to Left

	    	if (rx_motor_step == 0) { time_step_m = 0.0;  }
			else if(rx_motor_step != 0)
			{
			  time_step_m = ( ( rx_motor_step / (time_shots - 1) ) / 10.0 );
			}
	    	disp_step(time_step_m, 2, 10);

	     break;

	   }
	break;
   }
}


void time_screen3()    //  screen 3 of timelapse function
{
	lcd_clear();  row_pos = prev_row_pos;
	lcd_put_cur(0,0);  lcd_string_new(">");  lcd_put_cur(0, 1); lcd_string_new("<BACK>");
	button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);

	while( button != 0 &&  cursor_pos == 9 )
	  {
		  button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
		  get_row_pos(); //get the row position on the lcd

	   }

      if (button == 0) {lcd_clear(); prev_row_pos = 1; auto_unl();} // go back to the main function
	  if (cursor_pos >=10 ) { prev_row_pos = 0; cursor_pos = 1; time_screen1();}
	  else if (cursor_pos <= 8 ) {prev_row_pos = 3; cursor_pos = 8; time_screen2();}

}

void time_screen2()     // screen 2 of timelaspe function
{

    	lcd_clear();
	    row_pos = prev_row_pos;
		lcd_put_cur(0, 1); 		  lcd_string_new("Shots:  <     >"); 	disp_time_shots(time_shots, 0, 10 );//lcd_put_cur(0,10); lcd_int_to_str(time_shots);
		lcd_put_cur(1, 1);		  lcd_string_new("Duration:      ");
		///// time Duration calculation ///
	    time_duration =  interval_val * time_shots;
	    time_duration_cal();//time_duration_cal();   // lcd_put_cur(1, 11); lcd_int_to_str(home_pos);
		lcd_put_cur(2, 1);		  lcd_string_new("Step:   <     >");  	//lcd_put_cur(2,10); float_to_string(step);
		if(time_mode == 'C')      {lcd_put_cur(2, 10);lcd_string_new(" --- ");}
	//	else if(time_mode == 'S') { time_step_cal();}            //{ disp_step(time_step, 2, 10); }

		else if(time_mode == 'S') {    if(man_unl_mode == 'U') { time_step_cal(); disp_step(time_step_u, 2, 10);}
								  else if(man_unl_mode == 'M') { time_step_cal(); disp_step(time_step_m, 2, 10);} }  //{ disp_step(time_step, 2, 10); }

		lcd_put_cur(3, 1); 		 lcd_string_new("[Start]");
		lcd_put_cur(row_pos,0);  lcd_string_new(">");

		while( cursor_pos >= 5 &&  cursor_pos <= 8 )
	   {
		     get_row_pos();  // get the row position on the lcd
			 switch(row_pos)
		   {
			case 0:   //  set the No of shots to be taken
					read_joystic();
					if (x_value <= 900) {
					   if ( time_shots < (3600000 / interval_val)) { time_shots++; }
					        disp_time_shots(time_shots, 0, 10 );  //lcd_put_cur(0,10); lcd_int_to_str(time_shots);
					        jx_delay(); } // shots to be taken

					else if (x_value >= 3000) {
						if ( time_shots > 4) { time_shots--; }
						     disp_time_shots(time_shots, 0, 10 );  //lcd_put_cur(0,10); lcd_int_to_str(time_shots);
						     jx_delay();  }
					      ///// time Duration calculation ///
					         time_duration =  interval_val * time_shots;
					         time_duration_cal();
					       if( time_mode == 'S') { if(man_unl_mode == 'M') { time_step_cal(); disp_step(time_step_m, 2, 10);} }


				    break;

		  	 case 1: //  calculation of the time duration


			    break;

			 case 2: // set to No of steps to be motor move forward or backward
				 if (time_mode == 'C')
				    { lcd_put_cur(2, 10);lcd_string_new(" --- ");}

				 else if(time_mode == 'S')
				    {
					  time_step_cal();
					}


	            break;

			 case 3:   // go to the START Function of the timelapse
				   button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
				   if (button == 0)
				   { HAL_Delay(50);
				     time_start();  // start the time lapse function
				     time_stop();   // stop the time lapse function
				   }
				break;
			}

			//}
		  }

		    spi_buf_load(time_shots);
			eeprom_write(6, 4);         // store time_shots in EEPROM         //  store time_step_u in EEPROM
		 		//HAL_Delay(5);

		 	step_data = time_step_u * 10;
		 	spi_buf_load(step_data);
		 	eeprom_write(14, 4);        // store time_step_u in EEPROM         //  store time_step_u in EEPROM
		 		//HAL_Delay(5);


			if (cursor_pos <= 4 ) { prev_row_pos = 3; time_screen1();}
		    else if (cursor_pos >= 9) { prev_row_pos = 0; cursor_pos = 9; time_screen3(); }

}


void time_screen1()     // screen 1 of timelapse function
{
	    float expos_max = 0.0;
        lcd_clear();
	    row_pos = prev_row_pos;
		lcd_put_cur(0, 1); 		  lcd_string_new("Mode:"); 			 lcd_put_cur(0, 6);
	if  (time_mode == 'C')        lcd_string_new("Continuous");  	 else if (time_mode == 'S') lcd_string_new("       SDS");
		lcd_put_cur(1, 1); 		  lcd_string_new("Direction:");  	 lcd_put_cur(1, 11); 				//lcd_int_to_str(home_pos);
	if  (motor_move == 'L')		  lcd_string_new(" Left");  		 else if (motor_move == 'R') lcd_string_new("Right");
		lcd_put_cur(2, 1); 		  lcd_string_new("Interval:<   s>");  disp_interval(interval_val);    	//lcd_put_cur(2, 11); lcd_int_to_str(interval_val);
		lcd_put_cur(3, 1);		  lcd_string_new("Expos:  <    s>");  disp_expos(expos_val);          	//lcd_put_cur(3, 11); float_to_string(expos_val);
		lcd_put_cur(row_pos,0);   lcd_string_new(">");
	//	cursor_pos =1; row_pos = 0;

		 while( cursor_pos >= 1 &&  cursor_pos <= 4 )
	   {
		    	 get_row_pos();   //get the row position on the lcd
				 switch(row_pos)
		   {
			case 0:   //  Set the mode of timelapse Continuous / SDS
					read_joystic();
					if (x_value <= 900)
					{ time_mode = 'S';   lcd_put_cur(0,6); lcd_string_new("       SDS"); }
					else if (x_value >= 3000)
					{ time_mode = 'C';   lcd_put_cur(0,6); lcd_string_new("Continuous");}

			 break;

			 case 1: //  Set the direction of the motor Right / Left
			    	 read_joystic();          // HAL_Delay(3);
					if (x_value <= 900)
					{ motor_move = 'R';  lcd_put_cur(1, 11); lcd_string_new("Right"); }
					else if (x_value >= 3000)
					{ motor_move = 'L';  lcd_put_cur(1, 11); lcd_string_new(" Left"); }

					 break;

			 case 2: // Set the Interval value

				    read_joystic();
					if (x_value <= 900)
					     {
						    if (interval_val < 600)
						       { interval_val++; interval_count = interval_val; }   // lcd_put_cur(2,11); lcd_int_to_str(interval_val);
						       disp_interval(interval_val); jx_delay();
					     }

					else if (x_value >= 3000)
					    {
						   if (interval_val > 1 )
						      { interval_val--;  interval_count = interval_val;}
						      //lcd_put_cur(2,11);    //lcd_int_to_str(interval_val);
						      disp_interval(interval_val);  jx_delay();
					    }

				    expos_max = (interval_val / 1.0 ) - 0.5;

					if (expos_val > expos_max)
					   {
					     expos_val  = expos_max; disp_expos(expos_val);
					     expos_data = (expos_max * 10);
					   }
					else if (expos_val <= expos_max) { disp_expos(expos_val); }

			   	    break;

			 case 3:  // Set the Expos value

					read_joystic();
					if (x_value <= 900)
					   {
						 if( (expos_val < 100.0) && (expos_val < expos_max) ) //(interval_val - 0.2)))
						    {
						      expos_val += 0.1; expos_data++;
						      disp_expos(expos_val);
						     }                           //lcd_put_cur(3,10);     //float_to_string(expos_val);
						     jx_delay();
					    }
					else if (x_value >= 3000)
					   {
//						    if(expos_val > 0.2)
//					   	    {
//							expos_val -= 0.1; expos_data--; disp_expos(expos_val); }    //lcd_put_cur(3,10); float_to_string(expos_val);
//						    jx_delay();
						    expos_val -= 0.1; expos_data--;
						 if(expos_val < 0.2)
						   {
							 expos_val = 0.2; expos_data = 2;
						   }
						     disp_expos(expos_val);     //lcd_put_cur(3,10); float_to_string(expos_val);
						     jx_delay();
					   }

				//	spi_buf_load(expos_val);
				//	eeprom_write(3, 4);        // store inteval_val into eeprom

					break;
			}

			//}
		  }

		  spi_buf_load(interval_val);
	      eeprom_write(2, 4);          // store inteval_val in EEPROM
          	  //HAL_Delay(5);

          spi_buf_load(expos_data);
          eeprom_write(10, 4);        // store expos_val  in EEPROM
          	  //HAL_Delay(5);


		  if (cursor_pos <= 0 ) { cursor_pos = 9; prev_row_pos = 0; time_screen3(); }
		  else if (cursor_pos >= 5) { prev_row_pos = 0; time_screen2();  }
}


void timelapse_fun()
{
		working_mode = 'G';
		direction = 'S';
	//	data_transmit();
		timelapse_flag = 1;
		cursor_pos = 1;
		prev_row_pos = 0;

  //////// read data from EEPROM
		eeprom_read(2, 4);
		interval_val = rec_buf_data();
		    //HAL_Delay(2);
		eeprom_read(6, 4);
        time_shots =   rec_buf_data();
        	//HAL_Delay(2);
        eeprom_read(10, 4);
        expos_data =  rec_buf_data();
        expos_val = expos_data / 10.0;
        	//HAL_Delay(2);
        eeprom_read(14, 4);
        step_data =  rec_buf_data();
        time_step_u = step_data / 10.0;
        	//HAL_Delay(2);

	while (timelapse_flag == 1)
	{
		time_screen1();
	}

}


//------------------ TIMELAPSE ENG ------------------//



//***********************************************************//

//******** ANIMATION FUNCTION  ***********//

//***********************************************************//

//------------------ START --------------------//


    //      Animation Mode commented due to Low Flash 3-4-24


void ani_step_display()  // display function
{
	 lcd_put_cur(2,0);   lcd_string_new("   /          / ");
	 lcd_put_cur(2,0);   disp_steps(steps_count, 2, 0);           //  lcd_int_to_str(steps_count);
	 lcd_put_cur(2,4);   lcd_int_to_str(steps);
	 //disp_step(step, 2, 8);                         // lcd_put_cur(2,8);   float_to_string(step);
	 lcd_put_cur(2,13);  lcd_int_to_str(shots_count);
	 lcd_put_cur(2,15);  lcd_int_to_str(shots);
}



void pause_stop_fun()   //  animation pause and stop function during the animation mode
{
		lcd_clear();
		lcd_put_cur(1, 5); lcd_string_new("PAUSE");
		lcd_put_cur(2, 0); lcd_string_new("Press OK To Exit");
		HAL_Delay(200);

	for (int m =0; m < 1500; m++) /////   button press second time to exit    /////
	{
		button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	    if(button == 0)
	  {
		  lcd_clear();   	   prev_row_pos = 2;
		  shots_count =   0;     steps_count = 0;
		  working_mode = 'G';  direction = 'S';
		  load_buffer();
		  transmit_data();     //		  data_transmit();     yes_no = 0;
		// HAL_Delay(10);       // wait to transmit data
		  ani_screen2();       // back to the start function in screen 2
	   }

		HAL_Delay(1);
	}
		transmit_data();  HAL_Delay(10);
		rx_data();
	    lcd_clear();
		lcd_put_cur(1, 0); lcd_string_new("ANIM dir");
		lcd_put_cur(1,8);  lcd_send_data(motor_move);
		ani_step_display();
		disp_motor_step(motor_step);
}



void step_count()    // steps count and display function
{
	HAL_Delay(10);
	transmit_data();               //data_transmit();
	rx_data();
	disp_motor_step(motor_step);
	start_flag = 0;
	shots_count = 0;  lcd_put_cur(2,13); lcd_int_to_str(shots_count);

	for( int j =0; j < wait_sec; j++)  // Delay count after motor move to next position
	{
		HAL_Delay(10);  button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
		if(button == 0){ pause_stop_fun(); }
	 //   transmit_data();  // Transmit buffer continue		// data_transmit(); //	rx_data();
	}

	for ( int k = 0;  k < shots; k++ )
	{

		send_buffer[26] =   11;    // zoom bit
	//  send_buffer[27] = '0';    // shot bit
		transmit_data();
		HAL_Delay(250);
	//	send_buffer[26] =  11;    // zoom bit
	//	send_buffer[27] = '1';    // shot bit
		transmit_data();
		HAL_Delay(250);

		lcd_put_cur(1, 0); lcd_string_new(" ");
		button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);

	 if(button == 0)   {pause_stop_fun();}

 	 //   send_buffer[26] =  00;   // zoom bit
	 //	send_buffer[27] = '0';   // shot bit  send_step_status;
	    send_buffer[26] =  00;
	    transmit_data();
	    HAL_Delay(250);

	   // transmit_data();
	    HAL_Delay(250);

		shots_count++;   lcd_put_cur(2,13);  lcd_int_to_str(shots_count);
		lcd_put_cur(1, 0); lcd_string_new("A");
		button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);

     if(button == 0)   {pause_stop_fun(); }

       //transmit_data();  			// Transmit buffer continue
       //data_transmit();   //rx_data();
   }
      //send_buffer[26] = '0';    // zoom bit
      //send_buffer[27] = '0';    // shot bit
      //transmit_data();
	   HAL_Delay(10);

}



void step_move()    //  Motor step move function   //
{
     for (int jj = 0; jj < 2; jj++)  { transmit_data();  HAL_Delay(10); rx_data(); }  //{ data_transmit();  rx_data(); HAL_Delay(10); }
    // HAL_Delay(10);
    // transmit_data();
    // rx_data();
    // while  ( motor_move_step !=  rx_motor_step)   //(  motor_running != 0 )
   	   while  (  motor_running != 0 && motor_move_step !=  rx_motor_step)
   	   {
   		   disp_motor_step(motor_step);
   		   button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
   		   	if(button == 0)
   		   	{
   		   	 pause_stop_fun();
   		   	}
   		   	 HAL_Delay(10);
   		   	 transmit_data();               //data_transmit();
   		   	 rx_data();
   		   	 disp_motor_step(motor_step);
       }

}




void start_fun(void)        //  Animation start function
{
	 if ( man_unl_mode == 'M')
	 {
	    if (motor_move == 'R')
	     { max_limit = ((home_pos * 10) + (step * steps_limit * 10 ));   min_limit =  home_pos * 10; }
	    else if  (motor_move == 'L')
	     { min_limit = ((home_pos * 10) - ( step * steps_limit * 10 ));  max_limit =  home_pos * 10; }
	 }

      // transmit_data();
      // HAL_delay(10);
	 	 rx_data();
	 	 motor_move_step =  rx_motor_step;
	 	 working_mode = 'A';    //
	 	 direction = 'S';
	 	 load_buffer();
	 	 wait_sec = delay_val * 100;  // 1000
	 	 exit_flag = 1;
	 	 start_flag = 0;
	 	 animation_flag = 1;
	 	 lcd_clear();
	 	 disp_motor_step(motor_step);
	 	 lcd_put_cur(1, 0); lcd_string_new("ANIM dir");
	 	 lcd_put_cur(1,8);  lcd_send_data(motor_move);

   if  ( rx_motor_step != (home_pos * 10))  // check out the home pos is equal to the current posiotn or not
	   {
	  	  lcd_put_cur(2, 0); lcd_string_new("Go Home? N<- ->Y");
	  	  HAL_Delay(5);
  	       while (exit_flag == 1)       //((x_value <= 2900) && (x_value > 1000))
	       {
		     read_joystic();
		     if ((x_value >= 3000 ) && (x_value <= 4095) )   // N is pressed
		      {
		    	 yes_no = 0; 	 exit_flag = 0;
		      }

		     else if ((x_value <= 800) && (x_value >= 0 ))        // Y is pressed
		     {
	           if (home_pos < 0)       { send_step_status =  '-';  }  // find out the limit is in negative or in possitive direction
	           else if (home_pos >= 0) { send_step_status =  '+';  }
	           motor_move_step = (labs(home_pos) * 10);	 //home_pos = labs(home_pos);  // if limit is negative then convert it into possitive
		       yes_no = 1;
		       lcd_put_cur(2, 0);  lcd_string_new("Go To Home?     ");
		       send_step();
		   //  transmit_data();
		       step_move(); // going to the set home step
		       exit_flag = 0;
    		}
	     }
  	   }

    while(animation_flag == 1 )    //( button != 0 )
    {
    	ani_step_display();     	// display step, steps, shots,
    	step_count(); 				 // count the steps 				//step_start_fun();
    	start_flag = 0;
     	exit_flag = 1;
     	yes_no = 1;          // no  usage now

	   while ( exit_flag == 1)   //   (x_value <= 2400) && (x_value >= 1200))
		{
	      read_joystic();
	        while ( x_value <= 2400  && x_value >= 1200 )   // Joy stic in center
	    	 {
	    	    HAL_Delay(10);
	    	    read_joystic();
	    		button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	    		if(button == 0)     {pause_stop_fun();}
	    		if(start_flag == 1) { step_move();  step_count(); } //tep_start_fun();}
	    		multi_step = 0;                  // Reset the multi step variable
	    		transmit_data();				 // transmit data continue 	//data_transmit();
	    		rx_data();                       // receive data continue
	    		disp_motor_step(motor_step);     // display the motor steps
	    	}

		   if ((x_value <= 900) && (x_value >= 0 ))     // Motor step increment
		    {
			   if(steps_count < steps)
			   {
				 steps_count++;
				 multi_step++;
    		     disp_steps(steps_count, 2, 0);
       		  // rx_data();
       		     local = ((step * 10) * multi_step);
       		  // step_send_cal();
       		      if(motor_move == 'R')          { motor_move_step = ( rx_motor_step + local); }       ///( rx_motor_step + ((step * 10) * multi_step));
    		      else if(motor_move == 'L')     { motor_move_step = ( rx_motor_step - local); }
    		   // jx_delay();
                  HAL_Delay(200);
    		      send_step();   // load motor step to buffer
    		      start_flag = 1;
    	       }
            }

		  else if ((x_value >= 3000 ) && (x_value <= 4095))    // Motor step in decrement
		   {
			  if(steps_count > 0)
			   {
				 steps_count--;
				 multi_step++;
				 disp_steps(steps_count, 2, 0);
			//	 rx_data();
				 local = ((step * 10) * multi_step);
				// step_send_cal();
			      if(motor_move == 'R')      { motor_move_step = ( rx_motor_step - local); }       ///( rx_motor_step + ((step * 10) * multi_step));
			      else if(motor_move == 'L') { motor_move_step = ( rx_motor_step + local); }
				// else if(motor_move == 'L') {motor_move_step = ( rx_motor_step + ((step * 10) * multi_step) );}
				// jx_delay();
			      HAL_Delay(200);
				  send_step();     // load motor step to buffer
				  start_flag = 1;
			   }
	       }
	  }
   }
}



void ani_steps_cal()      // calculate the steps in animation mode
{
	 switch(man_unl_mode)
	  {
		 case 'U':  // UNL mode unlimited limits
		    read_joystic();
		    if (x_value <= 900) { if ( steps < 999 ) { steps++;}     disp_steps(steps, 3, 12);       //lcd_put_cur(3,10);   lcd_int_to_str(steps);
		       jx_delay(x_value); }   //HAL_Delay(50);  }
			else if (x_value >= 3000) { if (steps > 0 ) {steps--;}   disp_steps(steps, 3, 12);    //lcd_put_cur(3,10); lcd_int_to_str(steps);
			   jx_delay(); }    //HAL_Delay(50); }

		  break;


		 case 'M':  // MAN mode limits are set
			 // motor will move to Left direction home position
			if ( motor_move == 'L') {
				steps_limit = home_pos / step;  // steps = steps_limit;                   //lcd_put_cur(3,10); lcd_int_to_str(steps);
			   }
			// motor will move to Right Direction from home position
			else if (motor_move == 'R') {
		            steps_limit = ( fixed_limit - home_pos) / step;  //steps = steps_limit;    //lcd_put_cur(3,10); lcd_int_to_str(steps);
			}

			read_joystic();

			if (x_value <= 900) {
				if ( steps > steps_limit) {steps = 0;} else if (steps < steps_limit) { steps++;}
				   disp_steps(steps, 3, 12);  jx_delay(); }       //  lcd_put_cur(3,10);  lcd_int_to_str(steps); jx_delay(x_value); }  // HAL_Delay(100); }
			else if (x_value >= 3000) { if (steps > 0 ) { steps--;}         // lcd_put_cur(3,10); lcd_int_to_str(steps);
			       disp_steps(steps, 3, 12);  jx_delay(); }   //HAL_Delay(100); }

			break;
	 }

}



void ani_home_pos_cal()    // set the home position in animation mode
{
	 switch(man_unl_mode)
	  {

		case 'U':  // UNL mode unlimited limits
			read_joystic();
			if (x_value <= 900)
			  {
				home_pos++;   disp_home_pos(home_pos); jx_delay();
			  }     //lcd_put_cur(1,10);    //          lcd_int_to_str(home_pos); jx_delay(); }  // HAL_Delay(50); }
			else if (x_value >= 3000)
			 {
				home_pos--;  disp_home_pos(home_pos); jx_delay();
			 }    //lcd_put_cur(1,10);  disp_home_pos(); jx_delay(); }  //lcd_int_to_str(home_pos); jx_delay(); }  // HAL_Delay(50);}
		   break;

	   case 'M':  // MAN mode limits are set
			   read_joystic();
			if (x_value <= 900) {
				if (home_pos < fixed_limit) {home_pos++;}
				disp_home_pos(home_pos); jx_delay(); }  //  lcd_put_cur(1,10); lcd_int_to_str(home_pos);   jx_delay(); }  //HAL_Delay(50); }
			else if (x_value >= 3000) {
				if (home_pos > 0) {home_pos--;}
				 disp_home_pos(home_pos); jx_delay(); }    //lcd_put_cur(1,10);lcd_int_to_str(home_pos);     jx_delay(); }  //HAL_Delay(50); }
		        // steps_limit = home_pos / step; steps = steps_limit; // calculate the steps according to the Home_pos and step of motor
		   break;
     }
}




void ani_screen2()   // second screen of the Animation mode
{
		lcd_clear();
		row_pos = prev_row_pos;
		lcd_put_cur(0, 1); lcd_string_new("Shots:      < >");  lcd_put_cur(0,14); lcd_int_to_str(shots);
		lcd_put_cur(1, 1); lcd_string_new("Delay:     < s>");  lcd_put_cur(1,13); lcd_int_to_str(delay_val);
		lcd_put_cur(2, 1); lcd_string_new("[Start]");
		lcd_put_cur(3, 1); lcd_string_new("<BACK>");
		lcd_put_cur(row_pos,0);lcd_string_new(">");

	    HAL_Delay(150);

		while( cursor_pos >= 5 &&  cursor_pos <= 8 )
		 {
		    get_row_pos();  //get the row position on the lcd
		    switch(row_pos)
		     {
			 	 case 0:    // set the no. of shots to be taken
			 		 	 read_joystic();
			 		 	 if (x_value <= 900) { if ( shots < 9 ) {shots++;}       lcd_put_cur(0,14); lcd_int_to_str(shots);  HAL_Delay(150); }
			 		 	 else if (x_value >= 3000) { if ( shots > 0 ) {shots--;} lcd_put_cur(0,14); lcd_int_to_str(shots);  HAL_Delay(150); }
			 	         break;

			     case 1:  //  set the  delay between shots data
			    	 	 read_joystic();
			    	 	 if (x_value <= 900) { if ( delay_val < 9 ) {delay_val++;}        lcd_put_cur(1,13); lcd_int_to_str(delay_val);  HAL_Delay(150); }
			    	 	 else if (x_value >= 3000) { if ( delay_val > 0 ) { delay_val--;} lcd_put_cur(1,13); lcd_int_to_str(delay_val);  HAL_Delay(150); } /////////////////

				         break;

			     case 2:    // call to the start function
			    	 	 button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
			    	 	 if (button == 0) { lcd_clear(); start_flag = 1; start_fun(); }
			             break;

			     case 3:  // call back to live function
			    	 	 //HAL_Delay(1);  //read_joystic(); HAL_Delay(2);
			    	 	 //while( y_value <= 2000 && y_value > 1000 && exit_flag == 1 ) {
			    	 	 //read_joystic();      //HAL_Delay(2);
			    	 	 button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
			    	 	 if (button == 0)
			    	 	  {
			    	 		 animation_flag = 0; exit_flag = 0;
			    	 		 lcd_clear();
			    	 		 cursor_pos = 0; prev_row_pos = 2; auto_unl();
			    	 	  }
					//    }  //HAL_Delay(50);
				         break;
			}
		 // }

	  }

	  spi_buf[0] = shots;
	  eeprom_write(30, 1);          // store shots data in EEPROM

	  spi_buf[0] = delay_val;
	  eeprom_write(31, 1);          // store delay_val in EEPROM

	  if (cursor_pos >= 9 ) { cursor_pos = 1; prev_row_pos = 0; ani_screen1(); }
	  else if (cursor_pos <= 4) { prev_row_pos = 3; ani_screen1();  }

}


 void ani_screen1()   //  first screen of the Animation mode
 {

	    lcd_clear();
        row_pos = prev_row_pos;
	  	lcd_put_cur(0, 1); lcd_string_new("Direction:     "); lcd_put_cur(0,11);
	  	if (motor_move == 'L') lcd_string_new(" Left"); else if (motor_move == 'R') lcd_string_new("Right");
		lcd_put_cur(1, 1); lcd_string_new("HomePos:<     >");  disp_home_pos(home_pos);    //    lcd_put_cur(1, 10); lcd_int_to_str(home_pos);
		lcd_put_cur(2, 1); lcd_string_new("Step:   <     >");  disp_step(step, 2, 10);                //      lcd_put_cur(2, 10); float_to_string(step);
		lcd_put_cur(3, 1); lcd_string_new("Steps:  <     >");  disp_steps(steps, 3, 12);         //      lcd_put_cur(3, 10); lcd_int_to_str(steps);
		lcd_put_cur(row_pos,0);  lcd_string_new(">");

	    while( cursor_pos >= 1 &&  cursor_pos <= 4 )
	     {
	    	 get_row_pos();  //get the row position on the lcd
	        switch(row_pos)
		    {
		    	case 0:   //  set the direction of the motor Left or Right
		    			read_joystic();
		    			if (x_value <= 900) { motor_move = 'R'; lcd_put_cur(0,11); lcd_string_new("Right");}
		    			else if (x_value >= 3000) { motor_move = 'L';  lcd_put_cur(0,11); lcd_string_new(" Left");}

		    			break;

		    	case 1: //  set the home position of the motor
		    			ani_home_pos_cal();

		    			break;

		    	case 2: // set the motor step to increment or decrement during movement
		    			read_joystic();
		    			// if (x_value <= 900) { if (step < 99.9 ) { step += 0.1;} lcd_put_cur(2,10); float_to_string(step);      jx_delay(); }  //   HAL_Delay(10); }
		    			// else if (x_value >= 3000) { if ( step >= 0.2) { step -= 0.1;} lcd_put_cur(2,10);float_to_string(step);  jx_delay(); }  //steps_limit = home_pos / step; steps = steps_limit; // calculate the steps according to the Home_pos and step of motor
		    			// if (x_value <= 900)       { if (step < 99.9 ) { step = raw++ / 10.0;}   disp_step(step, 2, 11);  jx_delay(); }        //  lcd_put_cur(2,10);   float_to_string(step);  jx_delay(); }  //   HAL_Delay(10); }
		    			// else if (x_value >= 3000) { if ( step >= 0.2) { step = raw-- / 10.0;}   disp_step(step, 2, 11);  jx_delay(); }        //lcd_put_cur(2,10);   float_to_string(step);  jx_delay(); }  //steps_limit = home_pos / step; steps = steps_limit; // calculate the steps according to the Home_pos and step of motor
		    			if (x_value <= 900)       { if (step < 99.9 ) { step += 0.1;}
		    			   roundedStep = roundToDecimal(step, 1);
		    			   step = roundedStep;
		    			   disp_step(step, 2, 10);  jx_delay(); }        //  lcd_put_cur(2,10);   float_to_string(step);  jx_delay(); }  //   HAL_Delay(10); }
		    			else if (x_value >= 3000) { if ( step >= 0.2) { step -= 0.1;}
		    			   roundedStep = roundToDecimal(step, 1);
		    			   step = roundedStep;
		    			   disp_step(step, 2, 10);  jx_delay(); }        //lcd_put_cur(2,10);   float_to_string(step);  jx_delay(); }  //steps_limit = home_pos / step; steps = steps_limit; // calculate the steps according to the Home_pos and step of motor

		    			break;

		    	case 3:  // Set the no of steps for the animation mode
		    			ani_steps_cal();

		    			break;
		   }
       }

		spi_buf_load(home_pos);
		eeprom_write(18 , 4);          // store inteval_val in EEPROM

        ani_step_data = step * 10;
		spi_buf_load(ani_step_data);
		eeprom_write(22, 4);          // store step data  in EEPROM

		spi_buf_load(steps);
		eeprom_write(26, 4);          // store steps data in EEPROM

		if (cursor_pos <= 0 ) { cursor_pos = 8; prev_row_pos = 3; ani_screen2(); }
	    else if (cursor_pos >= 5) { prev_row_pos = 0; ani_screen2();  }

 }



void animation_fun()
{
	working_mode = 'G';
	direction = 'S';
	load_buffer();
	transmit_data();                        //          data_transmit();
    cursor_pos = 1;
    animation_flag = 1;
	row_pos = 0;
	exit_flag = 1;
	lcd_clear();
	prev_row_pos = 0;
//////// read data from EEPROM

	eeprom_read(18, 4);
	home_pos = rec_buf_data();

	eeprom_read(22, 4);
	ani_step_data =  rec_buf_data();
	step = ani_step_data / 10.0;

	eeprom_read(26, 4);
	steps =  rec_buf_data();

	eeprom_read(30, 1);
	shots =  rec_buf[0];

    eeprom_read(31, 1);
	delay_val =  rec_buf[0];

	while ( animation_flag == 1  )
      {
	    // row_pos = prev_row_pos;
	    ani_screen1();
	    //screen_second();
      }
}


        // Animation Mode commented due to Low Flash  3-4-24

//----------------ANIMATION MODE END ------------------//




//***********************************************************//

//******** CONFIGURATION MODE  ***********//

//***********************************************************//


void config_fun(void)
{
		lcd_clear();
		exit_flag = 1;
		row_pos = 0;
		lcd_put_cur(0, 1);  lcd_string_new("Info");  lcd_put_cur(1, 1); lcd_string_new("Back-Light");
		lcd_put_cur(1, 12); lcd_string_new("   %");
		lcd_put_cur(1, 12); lcd_int_to_str(back_light);
		lcd_put_cur(2, 1);  lcd_string_new("Calibration");    lcd_put_cur(3, 1);   lcd_string_new("Back");
		lcd_put_cur(row_pos,0);lcd_string_new(">");
		HAL_Delay(250); // switch de bounce dealy

		while (exit_flag == 1)
		{
			button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
			while( button != 0 )   // scanning button is pressed or not
			 {
				button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
				read_joystic();
				if ((y_value < 3450) && (y_value > 850))
				  {
					j_toggle1 = 1;
					if ( row_pos == 1 )
					{
						if (x_value <= 900)
						{
						   read_joystic();
						   if (back_light <= 90) {back_light = back_light + 10 ;}
						   lcd_put_cur(1, 12); lcd_string_new("   %"); lcd_put_cur(1, 12); lcd_int_to_str(back_light);
						   HAL_Delay(200);
					    }

				       else if (x_value >= 3000)
				        {
				    	   if (back_light >= 10 ) {back_light = back_light - 10;}
				    	   lcd_put_cur(1, 12); lcd_string_new("   %"); lcd_put_cur(1, 12); lcd_int_to_str(back_light);
				    	   HAL_Delay(200);
				        }
						__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,  back_light );
			        }
			           HAL_Delay(50);
			     }

				else if ((y_value >= 3500 ) && (y_value <= 4095) && ( j_toggle1 == 1) )
				 {
					arrow_down();
					j_toggle1 = 0;
					HAL_Delay(50);
				 }

				else if ((y_value <= 800) && (y_value >= 0 ) && ( j_toggle1 == 1))
				 {
					arrow_up();
					j_toggle1 = 0;
					HAL_Delay(50);
				 }
             }

    	read_joystic();
    	HAL_Delay(250);
    	switch(row_pos)
		  {
			 case 0:
				 // info();
				   break;

			 case 1:
					  read_joystic();  // back_light();
					  break;

			 case 2:
					   man_flag = 0;
					   unl_flag = 0;    // clear the unl flag to go back to calibration mode
					   live_flag = 0;
					   animation_flag = 0;
					   exit_flag = 1;
					   re_calibration();
					   break;

			 case 3:  //back();
					   exit_flag = 0;
					   break;
		  }
	 }
            lcd_clear();
            prev_row_pos = row_pos;

}

//----------------CONFIGURATION MODE END ------------------//



//***********************************************************//

//******** MAIN FUNCTION OF THE PROJECT ***********//

//***********************************************************//

//----------------START ------------------//

void auto_unl()
{
	working_mode = 'G';
    direction = 'S';
    load_buffer();
    transmit_data();
//	data_transmit();
	exit_flag = 0;
	unl_flag = 1;
//	hold = 1;
//	row_pos = 0;
	//lcd_put_cur(0, 0); lcd_string_new("** limits are **");
	//lcd_put_cur(1, 0); lcd_string_new("** unlimited  **");
	//HAL_Delay(500);
	lcd_clear();

	  while ( unl_flag == 1)
	  {
		  row_pos = prev_row_pos;
		  lcd_put_cur(0, 1);lcd_string_new("LIVE");lcd_put_cur(1, 1); lcd_string_new("TIMELAPSE");
		  lcd_put_cur(2, 1);lcd_string_new("ANIMATION");lcd_put_cur(3, 1); lcd_string_new("CONFIG");
		  lcd_put_cur(row_pos,0);lcd_string_new(">");
		  HAL_Delay(500); // switch de bounce dealy

		  button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);

	     while( button != 0 )   // scanning button is pressed or not
	      {
	    	 button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
//			 read_joystic(); //HAL_Delay(1);
//			 if ((y_value <= 2400) && (y_value > 1300))
//			 { j_toggle1 = 1; }
//
//			 else if ((y_value >= 3000 ) && (y_value <= 4095) && ( j_toggle1 == 1) )
//			 { arrow_down(); j_toggle1 = 0;  }
//
//			 else if ((y_value <= 900) && (y_value >= 0 ) && ( j_toggle1 == 1))
//			 {  arrow_up(); j_toggle1 = 0;  }
			 get_row_pos();
			 HAL_Delay(1);
		  }

  	     switch(row_pos)
  	      {
  	  	  	  case 0:   	// int raw = 0; // live_fun(raw);
  	  	  		   	   live_fun();
  	  	  		   	   break;

  	  	  	  case 1:
  	  	  		  	  timelapse_fun();
  	  	  		  	  break;

  	  	  	  case 2:
  	  	  		  	  animation_fun();  //  Animation Mode stop due to Low Flash 3-4-24
  	  	  		  	  break;

  	  	  	  case 3:
  	  	  		  	  config_fun();
  	  	  		  	  break;
  	      }
	}
	lcd_clear();
}


//******** LIMTI SETTING OF THE MOTOR ***********//


void man_fun()
{
   // man_unl_mode = 'M';
	//working_mode = 'L';
    direction = 'S';
    data_transmit();
	lcd_clear();
	HAL_Delay(250);  // button de bounce delay
	//man_mo = current_motde = 0;
    motor_count = current_motor_step ;   // limit 1 hold the current position of the motor
	lcd_put_cur(1, 0); lcd_string_new("1st Limit ");  //lcd_put_cur(1, 1); float_to_string(limit_1);
//	lcd_put_cur(2, 0); lcd_string_new("<Move> & PressOK");
	button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
   // motor_limit = motor_step;
	 while ( button != 0 )
      {
		 button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	//   data_transmit();
		 get_dam_pot_value();
		 get_spd_pot_value();
		 load_buffer();
		 transmit_data();
		 rx_data();
		 motor_count = motor_step;
		 data_display();
		 read_joystic();      // HAL_Delay(2);
	  // HAL_Delay(1);
		 if ((x_value <= 2098 ) && ( x_value >= 1902))
		   {
			 speed_send = 15;
			 direction = 'S';
   		   }

		 else if ((x_value <= 900) &&  (x_value >= 0 ))  //// 1st limit increment
		  {
	    	direction = 'F';
       	    speed_send = map( x_value,  1900, 0, 15, 1550) * motor_speed_pot / 100;
		  }

		 else if ((x_value >= 3000)  &&  (x_value <= 4095)) // 1st limit decrement
	      {
			 direction = 'B';
			 speed_send = map( x_value,  2100, 4000, 15, 1550) * motor_speed_pot / 100;
	      }
     }

	    limit_1 = rx_motor_step;

	    //  limit_2 = limit_1;
   ///////////// second limit setting

	     lcd_put_cur(1, 0);lcd_string_new("2nd Limit "); //lcd_put_cur(1, 10); float_to_string(limit_2);
	     HAL_Delay(1000);
	     button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);

      while ( button != 0 )    /// setting the value of second limit
       {
    	  button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
    	  // data_transmit();
    	  get_dam_pot_value();
    	  get_spd_pot_value();
    	  load_buffer();
    	  transmit_data();
    	  rx_data();
    	  motor_count = motor_step;
    	  data_display();
    	  read_joystic();
   	    // HAL_Delay(1);

    	  if ((x_value <= 2098 ) && ( x_value >= 1902))
   		   {
    		  speed_send = 15;
    		  direction = 'S';
   		   }

    	  else if ((x_value <= 900) &&  (x_value >= 0 ))  //// 1st limit increment
    	   {
   	    	 direction = 'F';
   	    	 speed_send = map( x_value,  1900, 0, 15, 1550) * motor_speed_pot / 100;
    	   }

    	  else if ((x_value >= 3000)  &&  (x_value <= 4095)) // 1st limit decrement
    	   {
    		  direction = 'B';
    		  speed_send = map( x_value,  2100, 4000, 15, 1550) * motor_speed_pot / 100;
    	   }
       }

      //   limit_2 = motor_count;    // note the maximum limit of the motor
          limit_2 = rx_motor_step;
 // ****** calculation of the motor steps from limit_1 to limit_2
         //current_motor_step = motor_step;  // motor current position store
         motor_limit = (limit_2 - limit_1);

      //   spi_buf_load1(motor_limit); //(motor_limit);
   	  //   eeprom_write(34, 8);        // store motor_limit  in EEPROM

      // find out the limit is in negative or in possitive direction
         if (motor_limit < 0) { limit_status =  '-'; }
         else if (motor_limit >= 0) { limit_status = '+'; }

         // if limit is negative then convert it into possitive
         motor_limit = labs(motor_limit);
         fixed_limit = motor_limit / 10.0;


         lcd_clear();
         lcd_put_cur(1, 0);  lcd_string_new("** limits are **");
         lcd_put_cur(2, 2);  lcd_string_new("0  to");
         lcd_put_cur(2, 10); float_to_string(fixed_limit);

 //*********  motor direction change function string ************ //

         if(motor_step > 0.0)
          {
        	 motor_dir = 1; lcd_put_cur(2, 13); lcd_string_new("M L");
        	// direction  = 'L';
          }
         if (motor_step < 0.0)
          {
        	 motor_dir = 2; lcd_put_cur(2, 13); lcd_string_new("M R");
        	// direction = 'R';
          }

     /////////////// send set limit to the slave device///////
             working_mode = 'L';
             send_motor_limit();
             load_buffer();
             transmit_data();
     	     rx_data();
     	   //  current_motor_step = motor_step;
     	  //   int64_t dummy_data = motor_limit;

     	     spi_buf_load1();         //(motor_limit); //(motor_limit);
     	     eeprom_write(34, 8);        // store motor_limit  in EEPROM

     	     spi_buf[0] = limit_status;
     	     eeprom_write(44, 1);        // store motor_limit  in EEPROM


     	    //HAL_Delay(5);

    	     HAL_Delay(700);
             lcd_clear();

// ********  call to free_ride_fun function
             man_flag = 1;
             prev_row_pos = 0;
          //  auto_unl();    // unl_flag = 1;  // set the unl flag 1 to call the unlimited limit function
           // HAL_Delay(1000);

}



//********  SELECT LIMTI OR UNLIMITED MODE  ***********//



 void work_range()
 {

  //  hold = 1;
    int cur_pos = 1;
    lcd_put_cur(1, 3);lcd_string_new("WORK");lcd_put_cur(1, 8); lcd_string_new("RANGE");
    lcd_put_cur(2, 2);lcd_string_new("<MAN>");lcd_put_cur(2, 9); lcd_string_new(" UNL ");
    HAL_Delay(500);
    button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);

    while (  button != 0 )
     {
    	button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
    	read_joystic(); HAL_Delay(1);

    	if ((x_value <= 700)  && (x_value >= 0))
         {
    	   cur_pos = 2;
  	       lcd_put_cur(2, 2);lcd_string_new(" MAN ");lcd_put_cur(2, 9); lcd_string_new("<UNL>");
         }

    	else if ((x_value >=3000) && (x_value <= 4095))
         {
    	   cur_pos = 1;
  		   lcd_put_cur(2, 2); lcd_string_new("<MAN>"); lcd_put_cur(2, 9); lcd_string_new(" UNL ");
         }

     }

     switch (cur_pos)
      {
         case 1:
        	    spi_buf[0] =  0;
        	    spi_buf[0] =  77;        //'M';
        	    eeprom_write(1, 1);      //  store man_unl_mode in EEPROM
        	    man_unl_mode = 'M';     //   limited mode is selected
           	    man_fun();    			// call to limits setting function
        		auto_unl();  			// call to auto Unl function
        		break;

         case 2:
        	    spi_buf[0] =  0;
        	    spi_buf[0] =  85;       //'U';
        	    eeprom_write(1, 1);     // store man_unl_mode in EEPROM
        	    man_unl_mode = 'U';     //  Unlimited mode is selected
        		lcd_clear();
        		lcd_put_cur(1, 0); lcd_string_new("** limits are **");
        		lcd_put_cur(2, 0); lcd_string_new("** unlimited  **");
        		HAL_Delay(350);
        		auto_unl();
        		//unl_flag = 1; man_flag = 0;  break;
        		break ;
      }

//        if ( cur_pos == 1 )   // curson on the MAN
//           {   man_flag = 1; unl_flag = 0;  //  man_fun();
//    	   }  ///////// call to the man function
//
//        if (cur_pos  == 2)
//           { unl_flag = 1; man_flag = 0;  unl_fun();
//           }   //////   call to the unl function


}






//***********************************************************//

//*********** RE-CALIBRATION FUNCTION **************//

//***********************************************************//

 //----------------START ------------------//

 void re_calibration()
 {

	  lcd_clear();
      working_mode = 'C';
      direction = 'S';
     // data_transmit();
      load_buffer();
      transmit_data();
      transmit_data();
   	  lcd_put_cur(1, 1);  lcd_string_new("RE-CALIBRATE?");
   	  lcd_put_cur(2, 2);  lcd_string_new("<NO>");
   	  lcd_put_cur(2, 8);  lcd_string_new(" YES ");

	  HAL_Delay(100);  // button de bounce dealy
	  button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
	  j_toggle = 1;

	  while ( button != 0 )
	  {
	     read_joystic();
		 button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin); read_joystic(); HAL_Delay(2);

		 if((x_value <= 2900) && (x_value > 1000))
     	      {
			   	 j_toggle = 1;
     	      }

		 else if ((x_value <=700) && (x_value >= 0) && ( j_toggle == 1))
	 		 {
			     j_toggle =  0;
			     home = 2; lcd_put_cur(2, 2); lcd_string_new(" NO "); lcd_put_cur(2, 8); lcd_string_new("<YES>");
	 		    // HAL_Delay(1);
			 }

		 else if ((x_value >= 3000) && (x_value <= 4095) && (j_toggle == 1))
	 		 {
	 		     j_toggle = 0;
	 		     home = 1; lcd_put_cur(2, 2);  lcd_string_new("<NO>"); lcd_put_cur(2, 8); lcd_string_new(" YES ");
	 		     //HAL_Delay(1);
	 		 }
	  }

	 switch (home)
	  {
	 	 case 1:
	 			 lcd_clear();
	 			 //lcd_put_cur(1, 0); lcd_string_new("** limits are **");
	 			 //lcd_put_cur(2, 0); lcd_string_new("** unlimited  **");
	 			 // HAL_Delay(350);    man_unl_mode = 'U';  auto_unl();    //  HAL_Delay(5);
	 			 rec_buf[0] = 0;
	 			 eeprom_read(1, 1);  		//  Read man_unl_mode from EEPROM
	 			 mode_data = rec_buf[0];

                 if(mode_data == 85)
                   {
                	  man_unl_mode = 'U';        // current mode is unlimited
                	  lcd_put_cur(1, 0); lcd_string_new("** limits are **");
                	  lcd_put_cur(2, 0); lcd_string_new("** unlimited  **");
                	  HAL_Delay(350);
                   }

                 else if (mode_data == 77)
                   {
                	 man_unl_mode = 'M';       //  current mode is limited
           	 		 eeprom_read(34, 8);  		//  Read motor_limit from EEPROM
             		 motor_limit = rec_buf_data1();
             		 motor_limit = labs(motor_limit);

             		 eeprom_read(44, 1);  		//  Read limit_status from EEPROM
             		 limit_status = rec_buf[0];

             		 working_mode = 'L';
             		 send_motor_limit();
             		 load_buffer();
             		 transmit_data();
             		 transmit_data();
                   }

                 else if (mode_data != 77 && mode_data != 85)
					{
                	  lcd_put_cur(2, 2);  lcd_string_new("  Mode Data Error   ");
					}

                 auto_unl();    //  HAL_Delay(5);
	 			 break;

	 	 case 2:
	 			 lcd_clear(); jpos = 1;  work_range();  // callling to the work range function
//	 		     if ( man_flag == 1 ) { lcd_clear(); man_flag = 0;   man_fun(); } /// calling to the man function
//	 		     if ( unl_flag == 0 )  { lcd_clear(); auto_unl(); }
   			     break;
	 }
 }


 //----------------END ------------------//


 ///////////////

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  lcd_init();
  lcd_clear();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, back_light);


  HAL_GPIO_WritePin(GPIOB, LCD_BRIGHTNESS_Pin, 1);
  HAL_GPIO_WritePin(jbtn_GPIO_Port, joy_btn_Pin, 1);

  lcd_put_cur(1, 0); lcd_string_new("CAMERA SLIDER 03");
  lcd_put_cur(2, 0); lcd_string_new("ROUND SHAPE SLIDE");
  HAL_Delay(500); lcd_clear();
//  lcd_put_cur(0, 2); lcd_string_new("Device-Info");
//  lcd_put_cur(2, 0); lcd_string_new("                     ");
//  HAL_Delay(1000); lcd_clear();

//  HAL_GPIO_WritePin(GPIOB, RJ45_LED_G_Pin, 0);
//  HAL_Delay(500);
//  HAL_GPIO_WritePin(GPIOB, RJ45_LED_G_Pin, 1);
//  HAL_Delay(500);

  while (1)
  {

	  //*****************

//      current_motor_step = eeprom_step;  // get previous motor position  from EEPROM
//      man_unl_mode =  eeprom_mu;             // get the previous status of  the MAN or UNL mode
//      data_transmit();
//      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, back_light);
// 		26-4
//      eeprom_fun();

//	  eeprom_read(32, 1);
//	  time_period =  rec_buf[0];    // Read Recording time from EEPROM

	  /////////// clear the EEPROM from 0 to 100

//	    for(int i = 0; i < 100; i++)
//	    {
//	    	spi_buf[0] = 0;
//	    	eeprom_write(i, 1);
//
//	    }


      ///////////////



		 rec_buf[0] = 0;
		 eeprom_read(1, 1);  		//  Read man_unl_mode from EEPROM
		 mode_data = rec_buf[0];

		 if(mode_data == 85)
		   {
			 man_unl_mode = 'U';        // current mode is unlimited
		   }
		 else if (mode_data == 77)
		   {
			 man_unl_mode = 'M';       //  current mode is limited
			 eeprom_read(34, 8);  		//  Read motor_limit from EEPROM
			 motor_limit = rec_buf_data1();
			 motor_limit = labs(motor_limit);

			 eeprom_read(44, 1);  		//  Read limit_status from EEPROM
			 limit_status = rec_buf[0];
		   }

			 working_mode = 'L';
			 send_motor_limit();
			 load_buffer();
			 transmit_data();
			 HAL_Delay(10);
			 transmit_data();
			 HAL_Delay(10);

	  re_calibration();


	    // 		26-4


	  //*****************


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	        ////////////  485   ///////////
	  	  //if (buffer[0]=='U')
	  	  //{HAL_GPIO_TogglePin(RJ45_LED_Y_GPIO_Port, RJ45_LED_Y_Pin);HAL_Delay(500);
	  	  //HAL_GPIO_TogglePin(RJ45_LED_Y_GPIO_Port, RJ45_LED_Y_Pin);HAL_Delay(500);
	  	  //HAL_GPIO_TogglePin(RJ45_LED_Y_GPIO_Port, RJ45_LED_Y_Pin);HAL_Delay(500);
	  	  //	  HAL_GPIO_TogglePin(RJ45_LED_Y_GPIO_Port, RJ45_LED_Y_Pin);HAL_Delay(500);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7200-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|LCD_BRIGHTNESS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin|RJ45_LED_G_Pin|RJ45_LED_Y_Pin|LCD_RS_Pin
                          |LCD_E_Pin|LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin
                          |LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MAX_EN_GPIO_Port, MAX_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin LCD_BRIGHTNESS_Pin */
  GPIO_InitStruct.Pin = LED_Pin|LCD_BRIGHTNESS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_Pin RJ45_LED_G_Pin RJ45_LED_Y_Pin LCD_RS_Pin
                           LCD_E_Pin LCD_D4_Pin LCD_D5_Pin LCD_D6_Pin
                           LCD_D7_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|RJ45_LED_G_Pin|RJ45_LED_Y_Pin|LCD_RS_Pin
                          |LCD_E_Pin|LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin
                          |LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : JS_SW_Pin */
  GPIO_InitStruct.Pin = JS_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(JS_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MAX_EN_Pin */
  GPIO_InitStruct.Pin = MAX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MAX_EN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


//////// one second dealy ///////

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim1)
{

  if (htim1->Instance == TIM1)
  {
	//  expos_count =  expos_count + 0.1;
	  switch(callback_mode)
	{
	  case 'T':  //  Time Lapse Mode
  	 expos_count++;

	  AM++;
	if ( AM == 10)
	  {
		inter++;
		time_duration--;      //counter++; // Increment counter every second
	    AM = 0;
	    currentMillis++;
	  }
	break;

	case 'R':       // Recording Mode
		  AM++;   //mills++;
		if ( AM == 10)
		  {
			time_duration++;      //counter++; // Increment counter every second
			AM = 0;
		  }

//		direction_buffer[d_index++] = direction;
//		speed_buffer[s_index++] =  speed_send >> 8;   //  1  8
//		speed_buffer[s_index++] =  speed_send & 0x00FF;   // 2  8

		break;

	case 'P':       // Play back mode
	  AM++;   //mills++;
	if ( AM == 10)
	  {
		time_duration++;      //counter++; // Increment counter every second
		AM = 0;
	  }

//		direction = direction_buffer[d_index++];
//		speed_send  = speed_buffer[s_index++] << 8;
//		speed_send |= speed_buffer[s_index++];

	break;
	  // Increment AM every 100 milliseconds

    }
  }
}


///////////////


void recieve_on_rs485(uint8_t buffer[],size_t buffer_length)
{
	HAL_GPIO_WritePin(MAX_EN_GPIO_Port, MAX_EN_Pin, 0);
	HAL_GPIO_WritePin(MAX_EN_GPIO_Port, MAX_EN_Pin, 0);
	HAL_UART_Receive_IT(&huart1, buffer, buffer_length);
}
void send_on_rs485(uint8_t* buffer)
{

     	HAL_GPIO_WritePin(MAX_EN_GPIO_Port, MAX_EN_Pin, 1);
		HAL_GPIO_WritePin(MAX_EN_GPIO_Port, MAX_EN_Pin, 1);
		message_packet_with_crc(buffer,  POLYNOMIAL,(BUFFER_LENGTH-1));
		memcpy( Previous_buffer,buffer,BUFFER_LENGTH );
//		if(cnt<1){
//		buffer[2]=0x0d;cnt++;}
//		else{buffer[2]=0x2d;}
		HAL_UART_Transmit(&huart1, buffer,BUFFER_LENGTH,100);
		//HAL_UART_Transmit(&huart1, "driver1\r\n", 9,100);
	//	HAL_Delay(1);
		recieve_on_rs485(recieve_buffer,BUFFER_LENGTH);

}


crc_t CRC_generate(crc_t* message1, crc_t polynomial, int nBytes )
{
	crc_t  remainder = 0;
	for (int byte = 0; byte < nBytes; ++byte)
	{
		remainder ^= ((*(message1+byte)) << (WIDTH - 8));/* Bring the next byte into the remainder.   */
		//printf("Hello World %x \n" ,remainder);
		/* Perform modulo-2 division, a bit at a time. */
		for (uint8_t bit = 8; bit > 0; --bit )
		{
			/*Try to divide the current data bit.*/
			if (remainder & TOPBIT){remainder = (remainder ^ POLYNOMIAL)<<1;}
			else{remainder = (remainder << 1);}
		}
	}
	return (remainder);/* The final remainder is the CRC result. */
}

uint8_t* message_packet_with_crc(crc_t* message2, crc_t polynomial, int nBytes )
{
	uint8_t crc_value=0;
	crc_value = CRC_generate( message2,  polynomial,  nBytes );
	message2+=nBytes;

	*message2=crc_value;
	return message2-=nBytes;
}


crc_t CRC_CHECK_decode(crc_t* message, crc_t polynomial, int nBytes )
{
	crc_t  remainder = 0;
	for (int byte = 0; byte < nBytes; ++byte)
	{
		remainder ^= ((*(message+byte)) << (WIDTH - 8));/* Bring the next byte into the remainder.   */
		//printf("Hello World %x \n" ,remainder);
		/* Perform modulo-2 division, a bit at a time. */
		for (uint8_t bit = 8; bit > 0; --bit )
		{
			/*Try to divide the current data bit.*/
			if (remainder & TOPBIT){remainder = (remainder ^ POLYNOMIAL)<<1;}
			else{remainder = (remainder << 1);}
		}
	}
	//printf(" xcv %x \n" ,remainder);
	if(remainder!=0){return 1;}
	else {return 0;}
//	return (remainder);/* The final remainder is the CRC result. */
}



//void USART1_IRQHandler(void)
//{
//counter++;
//	HAL_UART_Receive_IT(&huart1, buffer, 16);
//	//HAL_GPIO_TogglePin(RJ45_LED_Y_GPIO_Port, LED_Pin);
////	if (buffer == 0x55)
////	{
////		HAL_GPIO_TogglePin(RJ45_LED_Y_GPIO_Port, LED_Pin);
////
////	}
//
//
//
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//volatile uint8_t rx_index = 0;
		counter++;
		RS_485_Data_validate = CRC_CHECK_decode(recieve_buffer,  POLYNOMIAL, BUFFER_LENGTH); //returns 0 if valid ,1 if invalid.
     if(RS_485_Data_validate==0)
			{
				if(recieve_buffer[0]== 0x55)// if acknowledgment is valid
				{
					memcpy( RECIEVE_VALID_DATA,recieve_buffer, BUFFER_LENGTH );//store data into actual data buffer
					send_buffer[0]=0x55;//0x55 is acknowledgment for a valid data
					//send_on_rs485(send_buffer);

					//  Seprate  the buffer byte according to the their value
					state_of_rs485=1;
				}

				else if(recieve_buffer[0]== 0x65)// if acknowledgment is 0x65 means data is not received properly
				{

							//send previous data
					send_buffer[0]=0x55;
					Previous_buffer[0]=0x55;
					send_on_rs485(Previous_buffer);
					state_of_rs485=2;
				}

				// send acknowledgment of valid data
		//		send_buffer[0]=0x55;//0x55 is acknowledgment for a valid data


			}
			else
			{
				//neglect the data
				// send acknowledgment of invalid data
				send_buffer[0]=0x65;
				send_on_rs485(send_buffer);
				state_of_rs485=3;
			}
		//	send_on_rs485(send_buffer);


}

//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
//	while(1){
//	HAL_GPIO_TogglePin(RJ45_LED_Y_GPIO_Port, LED_Pin);HAL_Delay(100);
//
//
//	}
//
//
//}

////////////////////////////////
  // button interrupt calback function
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
// // if (GPIO_Pin == joy_btn_Pin )   // /&& interruptEnabled) // Check if interrupt is from the button pin and interrupt is enabled
// // {
//    // Toggle the LED
//  //  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
//	  //	pressed = 1;
//	  	lcd_clear();
//	  	//direction = 'P';
//	  //	data_transmit();
//	  	lcd_put_cur(1, 5); lcd_string_new("PAUSE");
//	  	lcd_put_cur(2, 0); lcd_string_new("Press OK To Exit");
//	      HAL_Delay(200);
//	    for (int m =0; m < 1300; m++) /////   button press second time to exit    /////
//	    {
//	  	button = HAL_GPIO_ReadPin(jbtn_GPIO_Port, joy_btn_Pin);
//	  	if(button == 0) { lcd_clear(); prev_row_pos = 2; shots_count = 0; ani_screen2(); } //  animation_fun();}
//	  	HAL_Delay(1);
//	    }
//
//	  	 lcd_clear();
//	  	 lcd_put_cur(1, 0); lcd_string_new("ANIM dir");
//	  	 lcd_put_cur(1,8);  lcd_send_data(motor_move);
//	  	 ani_step_display();
//	  	 disp_motor_step(motor_step);
//
//	  //	 direction = 'R';
//	  //	 data_transmit();
//	  	 // step_start_fun();           //step_start_fun();
//
// // }
//}



////////////////////////




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
