/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"

#include "modbus_conf.h"
#include "modbus.h"
#include "mat_lib.h"

/* Private variables ---------------------------------------------------------*/
LTDC_HandleTypeDef hltdc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart;
UART_HandleTypeDef huartmb;

/* Private variables ---------------------------------------------------------*/
int tims[7] = {0};
TS_StateTypeDef TS_State;
uint8_t touch_index = 0;
char text[100] = {0};

/* Private variables ---------------------------------------------------------*/
uint8_t *resp;
uint16_t resplen;
MB_RESPONSE_STATE respstate;
uint8_t fan_on[] =     {0x00, 0x01, 0x03, 0xE8};
uint8_t fan_half[] =   {0x00, 0x00, 0x01, 0xF4};
uint8_t fan_off[] =    {0x00, 0x01, 0x00, 0x00};
uint8_t heater_on[] =  {0x00, 0x04, 0x03, 0xE8};
uint8_t heater_half[] ={0x00, 0x04, 0x01, 0xF4};
uint8_t heater_var[] = {0x00, 0x04, 0x01, 0xF4};
uint8_t heater_off[] = {0x00, 0x04, 0x00, 0x00};
uint8_t get_temp[] = {0x00, 0x00, 0x00, 0x01};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_LTDC_Init(void);
static void LCD_Config(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_UART_PC_Init(void);
static void MX_UART_MB_Init(void);
bool Touched_Button(int button);

/* Private function prototypes -----------------------------------------------*/
__IO uint32_t input = 0;
__IO uint32_t output = 0;

uint8_t UART_MB_rcvd = 0;
__IO uint8_t UART_MB_sending = 0;

char txt[200] = {0};

void Communication_Mode(bool rx, bool tx){
	if(rx) HAL_UART_Receive_IT(&huartmb, &UART_MB_rcvd, 1);
	
	if(tx && UART_MB_sending == 0) {
		UART_MB_sending = 1;
		SetCharacterReadyToTransmit();
	} 
	if(!tx) UART_MB_sending = 0;
}
void Communication_Put(uint8_t ch){
	HAL_UART_Transmit_IT(&huartmb, &ch, 1);
}

uint8_t Communication_Get(void){
	uint8_t tmp = UART_MB_rcvd;
	UART_MB_rcvd = 0;
	SetCharacterReceived(false);
	return tmp;
}

void Enable50usTimer(void){
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void Disable50usTimer(void){
  HAL_NVIC_DisableIRQ(TIM4_IRQn);
}

/* Private function prototypes -----------------------------------------------*/

uint16_t v_range[2] = {0, 4095};

/*=================================*/
/* Text variables */
uint16_t y_text;

uint16_t left_x_text;
uint16_t right_x_text;


/*=================================*/
/* Plot variables */
uint16_t plot_x_min;
uint16_t plot_x_max;
uint16_t plot_y_min;
uint16_t plot_y_max;

uint16_t 	base_y;

uint16_t	x;
uint16_t	y_wyj;
uint16_t	y_ster;
uint16_t	y_zad;


/*=================================*/
/* Buttons variables */
uint16_t button_width;
uint16_t button_height;
	
uint16_t left_buttons_x;
uint16_t right_buttons_x;
	
uint16_t up_buttons_y;
uint16_t down_buttons_y;


/*=================================*/
/* Alarm and failure variables */
uint16_t alarm_x;
uint16_t alarm_y;

uint16_t sensor_err_x;
uint16_t sensor_err_y;

uint16_t modbus_err_x;
uint16_t modbus_err_y; 

uint16_t circle_rad;


/*=================================*/
/* Other variables */

float Sterowanie;
float Wart_zadana;
float Wyjscie;
float U_set;
float Y_zad;

bool Temp_alarm;
bool Temp_alarm_changed;
bool Temp_sensor_fail;
bool Temp_sensor_fail_changed;

#define REAL_OBJ		0
#define SIMULATION	1
int mode;

#define AUTO		0
#define MANUAL	1
int  control_mode;
bool control_mode_changed;

// Przyciski do manipulacji wartosci
#define STER_PLUS		0
#define STER_MINUS	1
#define ZAD_PLUS		2
#define ZAD_MINUS		3

#define U_MIN			 -50.0f
#define U_MAX				50.0f

/*===========================================================================*/

/*===============================*/
// Zmienne dla regulatora 
float e;
float u_past;
int D = 500;
float Ke = 2.8490e-04;;
float Ku[499] = {0};
void init_matixes();
// float Ku_mul_dUp[1];
float dUp[499];

float Uk[21]; //wektor przeszlych sterowan do opoznienia
float du = 0;
float y = 0, y_past = 0;
float a = 0.003468f;
float b = 0.9884f;
float Ku_mul_dUp[1];
#define u_max 50.0f
#define u_min -50.0f
/*===============================*/

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_LTDC_Init();
		
  LCD_Config(); 
	MX_UART_PC_Init();
	MX_UART_MB_Init();
	
	BSP_LED_Init(LED1);
	BSP_LED_On(LED1);
	
	BSP_PB_Init(BUTTON_TAMPER, BUTTON_MODE_EXTI);   
	BSP_TS_Init(0, 0);
	
	MB_Config(115200);
		
  HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
  HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);
  HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
  HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
	
	while(HAL_UART_GetState(&huart) == HAL_UART_STATE_BUSY_TX);
	while(HAL_UART_GetState(&huartmb) == HAL_UART_STATE_BUSY_TX);
	
	MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();	
		
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
	
	HAL_Delay(100);
	
	/* setting the fan W1 on 50% of its power */
	MB_SendRequest(14, FUN_WRITE_SINGLE_REGISTER, fan_half, 4);
	respstate = MB_GetResponse(14, FUN_WRITE_SINGLE_REGISTER, &resp, &resplen, 1000);
	if(respstate != RESPONSE_OK) while(1);
	
	HAL_Delay(900);
	
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	
	//	//====================================================================================
	/*==============================================================*/
	/* Buttons data */
	button_width  = BSP_LCD_GetXSize() * 0.15; // 	/ 15; 
	button_height = BSP_LCD_GetYSize() * 0.15;	// 	/ 20;
	
	left_buttons_x  = 30; //(uint16_t)BSP_LCD_GetXSize / 32; 	// 	/ 4 - (button_width / 2);
	right_buttons_x = left_buttons_x + button_width + 30;(uint16_t)BSP_LCD_GetXSize / 32 + 1.2 * button_width;		//	/ 4 - (button_width / 2);
	
	up_buttons_y   = 4 * BSP_LCD_GetYSize() / 16; 	// - button_height;
	down_buttons_y = 8 * BSP_LCD_GetYSize() / 16;  // - button_height;
	
	
	/*==============================================================*/
	/* Plot data */
	plot_x_min = BSP_LCD_GetXSize() / 2;
	plot_x_max = BSP_LCD_GetXSize();
	plot_y_min = BSP_LCD_GetYSize()   -   0.10*BSP_LCD_GetYSize();
	plot_y_max = BSP_LCD_GetYSize()   -   0.60*BSP_LCD_GetYSize();
	
	x = plot_x_min;
	
	base_y = (plot_y_max + 3*plot_y_min)/4;
	
	y_wyj = 0;
	y_ster = 0;
	y_zad = 0;
	
	
	/*==============================================================*/
	/* Displaying text */
	y_text = BSP_LCD_GetYSize() / 20;

	left_x_text = (uint16_t)BSP_LCD_GetXSize() / 32;
	right_x_text =(uint16_t)BSP_LCD_GetXSize() / 32 + BSP_LCD_GetXSize() / 2;
	
	BSP_LCD_SetFont(&Font16);
	
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_DisplayStringAt(left_x_text, y_text, "PANEL STEROWANIA", LEFT_MODE);
	BSP_LCD_DisplayStringAt(right_x_text, y_text, "WYKRES I DANE", LEFT_MODE);

	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/32, up_buttons_y   - 20, "Wartosc zadana:", LEFT_MODE);
	BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/32, down_buttons_y - 20, "Sterowanie:", LEFT_MODE);
	
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_DrawLine(BSP_LCD_GetXSize()/2, BSP_LCD_GetYSize(), BSP_LCD_GetXSize()/2, 0);
	
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_DrawLine(BSP_LCD_GetXSize()/2, plot_y_max, BSP_LCD_GetXSize(), plot_y_max);
	BSP_LCD_DrawLine(BSP_LCD_GetXSize()/2, plot_y_min+1, BSP_LCD_GetXSize(), plot_y_min+1);
	
	
	/*===============================*/
	/* Alarms and err data */
	alarm_x = left_buttons_x + 10;
	alarm_y = BSP_LCD_GetYSize() - BSP_LCD_GetYSize() / 5;

	sensor_err_x = left_buttons_x + 10;
	sensor_err_y = BSP_LCD_GetYSize() - BSP_LCD_GetYSize() / 10;

	modbus_err_x = 200;
	modbus_err_y = 20; 

	circle_rad = 10;
	
	/*===============================*/
	/* Other variables init */
	
	mode 				 = REAL_OBJ;
	control_mode = MANUAL;
	Sterowanie  = 0.0;
	Y_zad				= 30.0;
	
	Temp_alarm = false;
	Temp_alarm_changed = false;
	
	Temp_sensor_fail = false;
	Temp_sensor_fail_changed = false;
		
	control_mode_changed = false;
	
	/*==============================================================*/
	
	void init_matixes();
  while (1)
	{
		BSP_TS_GetState(&TS_State);
		for(touch_index=0;touch_index<TS_State.touchDetected;++touch_index)
		{
			if(TS_State.touchX[touch_index] < BSP_LCD_GetXSize() && TS_State.touchX[touch_index] > BSP_LCD_GetXSize() / 2)
			{
				for(int i = BSP_LCD_GetXSize() / 2; i < BSP_LCD_GetXSize(); i++)
				{
					BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
					BSP_LCD_DrawLine(i, plot_y_min, i, plot_y_max);
					x = plot_x_min;
				}
			} 
			else if( (control_mode == MANUAL) && Touched_Button(STER_PLUS) )
			{			
				//while(TS_State.touchDetected);
				Sterowanie++;
				if(Sterowanie >   50.0f) Sterowanie =  50.0f;			
			}
			else if(  (control_mode == MANUAL) && Touched_Button(STER_MINUS) )
			{
				//while(TS_State.touchDetected);
				Sterowanie--;
				if(Sterowanie <  -50.0f) Sterowanie = -50.0f;
			}
			else if( (control_mode == AUTO) && Touched_Button(ZAD_PLUS) )
			{
				//while(TS_State.touchDetected);
				Wart_zadana++;
				//Y_zad--;
				if(Y_zad >  80.0f) Y_zad = 80.0f;
			}
			else if( (control_mode == AUTO) && Touched_Button(ZAD_MINUS) )
			{
				Wart_zadana--;
				//Y_zad--;
				if(Y_zad <  24.0f) Y_zad = 24.0f;
			}
		}
	
		
		/*==============================================================*/
		/* PushButton usage */
		if(BSP_PB_GetState(BUTTON_TAMPER))
		{
			if(control_mode == AUTO)
			{
				control_mode = MANUAL;
				control_mode_changed = true;
			}
			else
			{
				control_mode = AUTO;
				control_mode_changed = true;
			}
		}												 
		
		/*==============================================================*/
		/* Displaying texts */
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	
		sprintf(text,"Wart zadana: %2.1f", Wart_zadana);
		BSP_LCD_DisplayStringAt(plot_x_min+10, y_text+15, (uint8_t*)text, LEFT_MODE);
		
		sprintf(text,"Wyjscie    : %2.1f", Wyjscie);
		BSP_LCD_DisplayStringAt(plot_x_min+10, y_text+30, (uint8_t*)text, LEFT_MODE);
		
		sprintf(text,"Sterowanie : %2.1f", Sterowanie);
		BSP_LCD_DisplayStringAt(plot_x_min+10, y_text+45, (uint8_t*)text, LEFT_MODE);
		
//		if( mode == REAL_OBJ)	sprintf(text,"Czujnik sprawny. Sterowanie: obiekt");
//		else 									sprintf(text,"Awaria czujnika. Sterowanie: model");
//		BSP_LCD_DisplayStringAt(plot_x_min+10, y_text+50, (uint8_t*)text, LEFT_MODE);
//		
//		if( control_mode == AUTO)	sprintf(text, "Tryb pracy: AUTO");
//		else 											sprintf(text, "Tryb pracy: MANUAL");
//		BSP_LCD_DisplayStringAt(10, 100, (uint8_t*)text, LEFT_MODE);
		
		
		/*==============================================================*/
		/* Displaying buttons	*/
//		if(control_mode_changed)
//		{
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			if(control_mode == AUTO)
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			else
				BSP_LCD_SetTextColor(LCD_COLOR_RED);
		
			BSP_LCD_DrawRect(left_buttons_x , up_buttons_y  , button_width, button_height);		// lewy górny
			sprintf(text," + 1 ");
			BSP_LCD_DisplayStringAt(left_buttons_x + 10, up_buttons_y + button_height/2, (uint8_t*)text, LEFT_MODE);
			
			BSP_LCD_DrawRect(right_buttons_x, up_buttons_y  , button_width, button_height);		// prawy górny
			sprintf(text," - 1 ");
			BSP_LCD_DisplayStringAt(right_buttons_x + 10, up_buttons_y + button_height/2, (uint8_t*)text, LEFT_MODE);

			if(control_mode == AUTO)
				BSP_LCD_SetTextColor(LCD_COLOR_RED);
			else
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			
			BSP_LCD_DrawRect(left_buttons_x , down_buttons_y, button_width, button_height);		// lewy dolny
			sprintf(text," + 1 ");
			BSP_LCD_DisplayStringAt(left_buttons_x + 10, down_buttons_y + button_height/2, (uint8_t*)text, LEFT_MODE);
			
			BSP_LCD_DrawRect(right_buttons_x, down_buttons_y, button_width, button_height);		// prawy dolny
			sprintf(text," - 1 ");
			BSP_LCD_DisplayStringAt(right_buttons_x + 10, down_buttons_y + button_height/2, (uint8_t*)text, LEFT_MODE);
			
//			control_mode_changed = false;
//		}	
		
		/*==============================================================*/
		/* Displaying alarm */
//		if(Temp_alarm_changed)
//		{
			if(Temp_alarm)
			{
				BSP_LCD_SetTextColor(LCD_COLOR_RED); 
				BSP_LCD_FillCircle(alarm_x, alarm_y, circle_rad);
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				BSP_LCD_SetFont(&Font8);
				sprintf(text,"ALARM\nTemp. za niska!");
				BSP_LCD_DisplayStringAt(alarm_x,alarm_y+20, (uint8_t*)text, LEFT_MODE);
			}
			else
			{
				BSP_LCD_SetTextColor(LCD_COLOR_WHITE); 
				BSP_LCD_FillCircle(alarm_x, alarm_y, circle_rad);
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK); 
				BSP_LCD_DrawCircle(alarm_x, alarm_y, circle_rad);
				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
					BSP_LCD_SetFont(&Font8);
				sprintf(text,"ALARM\nTemp. za niska!");
				BSP_LCD_DisplayStringAt(alarm_x,alarm_y+20, (uint8_t*)text, LEFT_MODE);
			}	
			BSP_LCD_SetFont(&Font16);
//			Temp_alarm_changed = false;
//		}
			
		/*==============================================================*/
		/* Displaying failure */
//		if(Temp_sensor_fail_changed)
//		{
			if(Temp_sensor_fail)
			{
				BSP_LCD_SetTextColor(LCD_COLOR_RED); 
				BSP_LCD_FillCircle(sensor_err_x, sensor_err_y, circle_rad);
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
					BSP_LCD_SetFont(&Font8);
				sprintf(text,"Temp. sensor ERROR!");
				BSP_LCD_DisplayStringAt(sensor_err_x,sensor_err_y+20, (uint8_t*)text, LEFT_MODE);
			}
			else
			{
				BSP_LCD_SetTextColor(LCD_COLOR_WHITE); 
				BSP_LCD_FillCircle(sensor_err_x, sensor_err_y, circle_rad);
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK); 
				BSP_LCD_DrawCircle(sensor_err_x, sensor_err_y, circle_rad);
				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
				sprintf(text,"Temp. sensor ERROR!");
				BSP_LCD_DisplayStringAt(sensor_err_x,sensor_err_y+20, (uint8_t*)text, LEFT_MODE);
			}	
			BSP_LCD_SetFont(&Font1);
//			Temp_alarm_changed = false;
//		}
		
		
	}
}



bool Touched_Button(int button)
{
	bool ret = false;
	bool x_left, x_right, y_up, y_down;
	
	x_left  = (TS_State.touchX[0] > left_buttons_x ) && (TS_State.touchX[0] < (left_buttons_x  + button_width));
	x_right = (TS_State.touchX[0] > right_buttons_x) && (TS_State.touchX[0] < (right_buttons_x + button_width));
	y_up		= (TS_State.touchY[0] > up_buttons_y   ) && (TS_State.touchY[0] < (up_buttons_y 	+ button_height));
	y_down	= (TS_State.touchY[0] > down_buttons_y ) && (TS_State.touchY[0] < (down_buttons_y  + button_height));
	
	switch(button)
	{
		case STER_PLUS:
			if(x_left && y_down)	ret = true;	
			else 									ret = false;
			break;
		case STER_MINUS:
			if(x_right && y_down)	ret = true;
			else 									ret = false;
			break;
		case ZAD_PLUS:
			if(x_left && y_up)	ret = true;
			else 								ret = false;
			break;
		case ZAD_MINUS:
			if(x_right && y_up)	ret = true;
			else 								ret = false;
			break;
		default:
			break;
	}
	
	return ret;
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{	
	if(huart->Instance == USART6){
		SetCharacterReceived(true);
		HAL_UART_Receive_IT(&huartmb, &UART_MB_rcvd, 1);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6)
		UART_MB_sending = 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	// Wywolywane w chwili wcisniecia przycisku User
}





void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	
	if(htim->Instance == TIM2){
		static uint16_t raw_y = 2345;
		static uint16_t raw_u = 0;
		static float y = 0.0f;
		static float u = 0.0f;
		MB_SendRequest(14, FUN_READ_INPUT_REGISTER, get_temp, 4);
		respstate = MB_GetResponse(14, FUN_READ_INPUT_REGISTER, &resp, &resplen, 1000);
		if(respstate != RESPONSE_OK) while(1);
		else {
			raw_y = resp[1]*0x100+resp[2];
			y = raw_y/100.0f;
		}
		//============================================================================
		
		if(control_mode == MANUAL)
		{
			u = Sterowanie;
		}
		else if(control_mode == AUTO)
		{
			Y_zad = Wart_zadana;
			
			if(mode == SIMULATION) {
                //===================================================
                // Symulacja obiektu
                y = y_past * b + Uk[20] * a; // jako pomiar czujnika
                //===================================================
            }
            //===================================================
            // Regulator DMC
            e = Y_zad - y;
						
//						for(int i = 0; i < D - 1; i++)
//						{
//							Ku_mul_dUp[0] += Ku[i] * dUp[i];
//						}
						
            mat_mul(Ku, 1, D - 1, dUp, D - 1, 1, Ku_mul_dUp);

            du = Ke * e - Ku_mul_dUp[0];

            u = u_past + du;

            // przesun w dól wektor dUp i wpisz na pierwsze miejsce róznice sterowan
//						for(int i = D - 1; i > 0; i--)
//						{
//							dUp[i] = dUp[i - 1];
//						}
						
            mat_move_down(dUp, D - 1, 1, 0, dUp);

            // ograniczenie sterowania
            if (u > u_max) {
                u = u_max;

            } else if (u < u_min) {
                u = u_min;
            }

            dUp[0] = du; // trzeba zaaplikowac po uwzglednieniu ograniczen

            mat_move_down(Uk, 21, 1, 0, Uk);
//						for(int i = 21; i > 0; i--)
//						{
//							Uk[i] = Uk[i - 1];
//						}
						
            Uk[0] = u;

            u_past = u;
            y_past = y;
            //===================================================

			Sterowanie = u;
		}
		
		if( (y < -55.0f) || (y > 125.0f) )
			Temp_sensor_fail = true;
		else
			Temp_sensor_fail = false;
		
		
		if(y < 24)
		{
			Temp_alarm = true;
//			Temp_alarm_changed = true;
		}
		else
		{
			Temp_alarm = false;
//			Temp_alarm_changed = true;
		}
		
		Wyjscie = y;
		
		
		//============================================================================
		
		
//		/* przyklady tego, jak nalezy interpretowac poszczegolne wartosci sterowania */
//		u = -10.0; // grzanie z moca (-10+50)% =  40%
//		u =   0.0; // grzanie z moca (  0+50)% =  50%
//		u =  50.0; // grzanie z moca ( 50+50)% = 100%
		
				
//		/* aplikacja ograniczen na sygnal sterujacy */
//		if(u >   50.0f) u =  50.0f;
//		if(u <  -50.0f) u = -50.0f;
		
		/* skalowanie z -50..50 do 0..1000 */
		raw_u = (uint16_t)(u+50.0f)*10; // przejscie z -2048 - 2047 do 0 - 4095
		
		/* przygotowanie wiadomosci MODBUS */
		heater_var[2] = (raw_u&0xFF00)>>8; // pierwszy bajt
		heater_var[3] = (raw_u&0x00FF)>>0; // drugi bajt
		
		/* wyslanie wiadomosci */
		MB_SendRequest(14, FUN_WRITE_SINGLE_REGISTER, heater_var, 4);
		
		/* odczyt odpowiedzi i sprawdzenie jej poprawnosci */
		respstate = MB_GetResponse(14, FUN_WRITE_SINGLE_REGISTER, &resp, &resplen, 1000);
		if(respstate != RESPONSE_OK) while(1);
		
		/* komunikacja z komputerem */
		while(HAL_UART_GetState(&huart) == HAL_UART_STATE_BUSY_TX);
		sprintf(txt,"T=%5.2f\r\n",u);
		if(HAL_UART_Transmit_IT(&huart,   (uint8_t*)txt, 9)!= HAL_OK) Error_Handler();
	} 
	if (htim->Instance == TIM3){ // timer odpowiedzialny za aktualizacje MB i odliczanie timeout'u
		MB();
		TimeoutTick();
	}
	if (htim->Instance == TIM4){ // timer odpowiedzialny za odliczanie kwantow 50us
		Timer50usTick();
	}
	if (htim->Instance == TIM5)
	{
		x = (x == plot_x_max)? plot_x_min : x + 1;
		
		y_wyj  =  base_y 	 +   (Wyjscie/90.0)     * ((3*plot_y_max + 5*plot_y_min)/4);
		y_ster =  base_y 	 + 	 (Sterowanie/90.0)  * ((3*plot_y_max + 5*plot_y_min)/4);
		y_zad  =  base_y 	 - 	 (Wart_zadana/90.0) * ((3*plot_y_max + 5*plot_y_min)/4);

		
//		y_wyj  = (plot_y_max - plot_y_min) / 4; 				//+ Wyjscie;
//		y_ster = (plot_y_max - plot_y_min) / 4 + 20;	  //+ (Sterowanie*(plot_y_max - plot_y_min))/100; 
//		y_zad  = (plot_y_max - plot_y_min) / 4 - 20;		//+ Wart_zadana;
		
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	  BSP_LCD_DrawLine(x, plot_y_min, x, plot_y_max+1);
		BSP_LCD_DrawPixel(x, y_wyj, LCD_COLOR_RED);
		BSP_LCD_DrawPixel(x, y_ster, LCD_COLOR_BLUE);
		if(control_mode == AUTO)	BSP_LCD_DrawPixel(x, y_zad, LCD_COLOR_YELLOW);
	}
}

void HAL_LTDC_LineEvenCallback(LTDC_HandleTypeDef *hltdc){
	// for lolz
}

static void LCD_Config(void)
{
  /* LCD Initialization */ 
  BSP_LCD_Init();

  /* LCD Initialization */ 
  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
  BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS+(BSP_LCD_GetXSize()*BSP_LCD_GetYSize()*4));

  /* Enable the LCD */ 
  BSP_LCD_DisplayOn(); 
  
  /* Select the LCD Foreground Layer  */
  BSP_LCD_SelectLayer(1);

  /* Clear the Foreground Layer */ 
  BSP_LCD_Clear(LCD_COLOR_WHITE);
	
  /* Configure the transparency for foreground and background :
     Increase the transparency */
  BSP_LCD_SetTransparency(1, 0xFF);
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 307;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 1);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

void MX_UART_PC_Init(void){
	huart.Instance        = USART1;
  huart.Init.BaudRate   = 115200;
  huart.Init.WordLength = UART_WORDLENGTH_8B;
  huart.Init.StopBits   = UART_STOPBITS_1;
  huart.Init.Parity     = UART_PARITY_NONE;
  huart.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  huart.Init.Mode       = UART_MODE_TX_RX;
  huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if(HAL_UART_DeInit(&huart) != HAL_OK)
  {
    Error_Handler();
  }  
  if(HAL_UART_Init(&huart) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_UART_MB_Init(void){
	huartmb.Instance        = USART6;
  huartmb.Init.BaudRate   = 115200;
  huartmb.Init.WordLength = UART_WORDLENGTH_9B;
  huartmb.Init.StopBits   = UART_STOPBITS_1;
  huartmb.Init.Parity     = UART_PARITY_EVEN;
  huartmb.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  huartmb.Init.Mode       = UART_MODE_TX_RX;
  huartmb.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if(HAL_UART_DeInit(&huartmb) != HAL_OK)
  {
    Error_Handler();
  }  
  if(HAL_UART_Init(&huartmb) != HAL_OK)
  {
    Error_Handler();
  }
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{
	
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2; 
  htim2.Init.Prescaler = 10800-1; // 108 000 000 / 10 800 = 10 000
	htim2.Init.Period = 10000;    		// 10 000 / 10 000 = 1 Hz (1/1 s)
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

	HAL_TIM_Base_Init(&htim2);     // Init timer
	HAL_TIM_Base_Start_IT(&htim2); // start timer interrupts
	//HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
	//HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */ // <- odstep w kodzie po to, aby miedzy MX_TIMx_Init mozna bylo przechodzic 
/* ***************** */ //    przy uzyciu Page Up i Page Down.
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3; 
  htim3.Init.Prescaler = 108-1; // 108 000 000 / 108 = 1 000 000
	htim3.Init.Period = 10;       // 1 000 000 / 10 = 100 000 Hz = 100 kHz(1/100000 s = 1/100 ms = 10 us)
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

	HAL_TIM_Base_Init(&htim3);     // Init timer
	HAL_TIM_Base_Start_IT(&htim3); // start timer interrupts
	//HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
	//HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */ // <- odstep w kodzie po to, aby miedzy MX_TIMx_Init mozna bylo przechodzic 
/* ***************** */ //    przy uzyciu Page Up i Page Down.
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4; 
  htim4.Init.Prescaler = 108-1; // 108 000 000 / 108 = 1 000 000
	htim4.Init.Period = 50;       // 1 000 000 / 50 = 20 000 Hz = 20 kHz (1/20000 s = 1/20 ms = 50 us)
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

	HAL_TIM_Base_Init(&htim4);     // Init timer
	HAL_TIM_Base_Start_IT(&htim4); // start timer interrupts
	//HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
	//HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */ // <- odstep w kodzie po to, aby miedzy MX_TIMx_Init mozna bylo przechodzic 
/* ***************** */ //    przy uzyciu Page Up i Page Down.
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5; 
  htim5.Init.Prescaler = 10800-1; // 108 000 000 / 10 800 = 10 000
	htim5.Init.Period = 10000;    // 10 000 / 10 000 = 1 Hz (1/1 s)
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

	HAL_TIM_Base_Init(&htim5);     // Init timer
	HAL_TIM_Base_Start_IT(&htim5); // start timer interrupts
	//HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);
	//HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

/* LTDC init function */
static void MX_LTDC_Init(void)
{

  LTDC_LayerCfgTypeDef pLayerCfg;

  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = (RK043FN48H_HSYNC - 1);
  hltdc.Init.VerticalSync = (RK043FN48H_VSYNC - 1);
  hltdc.Init.AccumulatedHBP = (RK043FN48H_HSYNC + RK043FN48H_HBP - 1);
  hltdc.Init.AccumulatedVBP = (RK043FN48H_VSYNC + RK043FN48H_VBP - 1);
  hltdc.Init.AccumulatedActiveH = (RK043FN48H_HEIGHT + RK043FN48H_VSYNC + RK043FN48H_VBP - 1);
  hltdc.Init.AccumulatedActiveW = (RK043FN48H_WIDTH + RK043FN48H_HSYNC + RK043FN48H_HBP - 1);
  hltdc.Init.TotalHeigh = (RK043FN48H_HEIGHT + RK043FN48H_VSYNC + RK043FN48H_VBP + RK043FN48H_VFP - 1);
  hltdc.Init.TotalWidth = (RK043FN48H_WIDTH + RK043FN48H_HSYNC + RK043FN48H_HBP + RK043FN48H_HFP - 1);
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;

  
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }

  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 1) != HAL_OK)
  {
    Error_Handler();
  }
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PE2   ------> QUADSPI_BK1_IO2
     PG14   ------> ETH_TXD1
     PE1   ------> FMC_NBL1
     PE0   ------> FMC_NBL0
     PB5   ------> USB_OTG_HS_ULPI_D7
     PB4   ------> S_TIM3_CH1
     PD7   ------> SPDIFRX_IN0
     PC12   ------> SDMMC1_CK
     PA15   ------> S_TIM2_CH1_ETR
     PE5   ------> DCMI_D6
     PE6   ------> DCMI_D7
     PG13   ------> ETH_TXD0
     PB7   ------> USART1_RX
     PB6   ------> QUADSPI_BK1_NCS
     PG15   ------> FMC_SDNCAS
     PG11   ------> ETH_TX_EN
     PD0   ------> FMC_D2_DA2
     PC11   ------> SDMMC1_D3
     PC10   ------> SDMMC1_D2
     PA12   ------> USB_OTG_FS_DP
     PI4   ------> SAI2_MCLK_A
     PG10   ------> SAI2_SD_B
     PD3   ------> DCMI_D5
     PD1   ------> FMC_D3_DA3
     PA11   ------> USB_OTG_FS_DM
     PF0   ------> FMC_A0
     PI5   ------> SAI2_SCK_A
     PI7   ------> SAI2_FS_A
     PI6   ------> SAI2_SD_A
     PG9   ------> DCMI_VSYNC
     PD2   ------> SDMMC1_CMD
     PI1   ------> SPI2_SCK
     PA10   ------> USB_OTG_FS_ID
     PF1   ------> FMC_A1
     PH14   ------> DCMI_D4
     PI0   ------> S_TIM5_CH4
     PA9   ------> USART1_TX
     PC9   ------> SDMMC1_D1
     PA8   ------> S_TIM1_CH1
     PF2   ------> FMC_A2
     PC8   ------> SDMMC1_D0
     PC7   ------> USART6_RX
     PF3   ------> FMC_A3
     PH4   ------> USB_OTG_HS_ULPI_NXT
     PG8   ------> FMC_SDCLK
     PC6   ------> USART6_TX
     PF4   ------> FMC_A4
     PH5   ------> FMC_SDNWE
     PH3   ------> FMC_SDNE0
     PF5   ------> FMC_A5
     PD15   ------> FMC_D1_DA1
     PB13   ------> USB_OTG_HS_ULPI_D6
     PD10   ------> FMC_D15_DA15
     PC3   ------> FMC_SDCKE0
     PD14   ------> FMC_D0_DA0
     PB12   ------> USB_OTG_HS_ULPI_D5
     PD9   ------> FMC_D14_DA14
     PD8   ------> FMC_D13_DA13
     PC0   ------> USB_OTG_HS_ULPI_STP
     PC1   ------> ETH_MDC
     PC2   ------> USB_OTG_HS_ULPI_DIR
     PB2   ------> QUADSPI_CLK
     PF12   ------> FMC_A6
     PG1   ------> FMC_A11
     PF15   ------> FMC_A9
     PD12   ------> QUADSPI_BK1_IO1
     PD13   ------> QUADSPI_BK1_IO3
     PH12   ------> DCMI_D3
     PA1   ------> ETH_REF_CLK
     PA4   ------> DCMI_HSYNC
     PC4   ------> ETH_RXD0
     PF13   ------> FMC_A7
     PG0   ------> FMC_A10
     PE8   ------> FMC_D5_DA5
     PD11   ------> QUADSPI_BK1_IO0
     PG5   ------> FMC_A15_BA1
     PG4   ------> FMC_A14_BA0
     PH7   ------> I2C3_SCL
     PH9   ------> DCMI_D0
     PH11   ------> DCMI_D2
     PA2   ------> ETH_MDIO
     PA6   ------> DCMI_PIXCLK
     PA5   ------> USB_OTG_HS_ULPI_CK
     PC5   ------> ETH_RXD1
     PF14   ------> FMC_A8
     PF11   ------> FMC_SDNRAS
     PE9   ------> FMC_D6_DA6
     PE11   ------> FMC_D8_DA8
     PE14   ------> FMC_D11_DA11
     PB10   ------> USB_OTG_HS_ULPI_D3
     PH6   ------> S_TIM12_CH1
     PH8   ------> I2C3_SDA
     PH10   ------> DCMI_D1
     PA3   ------> USB_OTG_HS_ULPI_D0
     PA7   ------> ETH_CRS_DV
     PB1   ------> USB_OTG_HS_ULPI_D2
     PB0   ------> USB_OTG_HS_ULPI_D1
     PE7   ------> FMC_D4_DA4
     PE10   ------> FMC_D7_DA7
     PE12   ------> FMC_D9_DA9
     PE15   ------> FMC_D12_DA12
     PE13   ------> FMC_D10_DA10
     PB11   ------> USB_OTG_HS_ULPI_D4
     PB14   ------> SPI2_MISO
     PB15   ------> SPI2_MOSI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
	
  /*Configure GPIO pin : OTG_HS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_HS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_HS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_D2_Pin */
  GPIO_InitStruct.Pin = QSPI_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_D2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TXD1_Pin RMII_TXD0_Pin RMII_TX_EN_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin|RMII_TXD0_Pin|RMII_TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PE1 PE0 FMC_D5_Pin FMC_D6_Pin 
                           FMC_D8_Pin FMC_D11_Pin FMC_D4_Pin FMC_D7_Pin 
                           FMC_D9_Pin FMC_D12_Pin FMC_D10_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0|FMC_D5_Pin|FMC_D6_Pin 
                          |FMC_D8_Pin|FMC_D11_Pin|FMC_D4_Pin|FMC_D7_Pin 
                          |FMC_D9_Pin|FMC_D12_Pin|FMC_D10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_D7_Pin ULPI_D6_Pin ULPI_D5_Pin ULPI_D3_Pin 
                           ULPI_D2_Pin ULPI_D1_Pin ULPI_D4_Pin */
  GPIO_InitStruct.Pin = ULPI_D7_Pin|ULPI_D6_Pin|ULPI_D5_Pin|ULPI_D3_Pin 
                          |ULPI_D2_Pin|ULPI_D1_Pin|ULPI_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D3_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(ARDUINO_PWM_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPDIF_RX0_Pin */
  GPIO_InitStruct.Pin = SPDIF_RX0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_SPDIFRX;
  HAL_GPIO_Init(SPDIF_RX0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SDMMC_CK_Pin SDMMC_D3_Pin SDMMC_D2_Pin PC9 
                           PC8 */
  GPIO_InitStruct.Pin = SDMMC_CK_Pin|SDMMC_D3_Pin|SDMMC_D2_Pin|GPIO_PIN_9 
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D9_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARDUINO_PWM_D9_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE5 PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : VCP_RX_Pin */
  GPIO_InitStruct.Pin = VCP_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(VCP_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_NCS_Pin */
  GPIO_InitStruct.Pin = QSPI_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(QSPI_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PG15 PG8 PG1 PG0 
                           FMC_BA1_Pin FMC_BA0_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_1|GPIO_PIN_0 
                          |FMC_BA1_Pin|FMC_BA0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_INT_Pin */
  GPIO_InitStruct.Pin = Audio_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Audio_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_D2_Pin FMC_D3_Pin FMC_D1_Pin FMC_D15_Pin 
                           FMC_D0_Pin FMC_D14_Pin FMC_D13_Pin */
  GPIO_InitStruct.Pin = FMC_D2_Pin|FMC_D3_Pin|FMC_D1_Pin|FMC_D15_Pin 
                          |FMC_D0_Pin|FMC_D14_Pin|FMC_D13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_P_Pin OTG_FS_N_Pin OTG_FS_ID_Pin */
  GPIO_InitStruct.Pin = OTG_FS_P_Pin|OTG_FS_N_Pin|OTG_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SAI2_MCLKA_Pin SAI2_SCKA_Pin SAI2_FSA_Pin SAI2_SDA_Pin */
  GPIO_InitStruct.Pin = SAI2_MCLKA_Pin|SAI2_SCKA_Pin|SAI2_FSA_Pin|SAI2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : SAI2_SDB_Pin */
  GPIO_InitStruct.Pin = SAI2_SDB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(SAI2_SDB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D7_Pin ARDUINO_D8_Pin LCD_DISP_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D7_Pin|ARDUINO_D8_Pin|LCD_DISP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_Detect_Pin */
  GPIO_InitStruct.Pin = uSD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 PF3 
                           PF4 PF5 PF12 PF15 
                           PF13 PF14 PF11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_12|GPIO_PIN_15 
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PG9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDMMC_D0_Pin */
  GPIO_InitStruct.Pin = SDMMC_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(SDMMC_D0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TP3_Pin NC2_Pin */
  GPIO_InitStruct.Pin = TP3_Pin|NC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_SCK_D13_Pin */
  GPIO_InitStruct.Pin = ARDUINO_SCK_D13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(ARDUINO_SCK_D13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_PWR_EN_Pin */
  GPIO_InitStruct.Pin = DCMI_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DCMI_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PH14 PH12 PH9 PH11 
                           PH10 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_12|GPIO_PIN_9|GPIO_PIN_11 
                          |GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_CS_D10_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_CS_D10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(ARDUINO_PWM_CS_D10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VCP_TX_Pin */
  GPIO_InitStruct.Pin = VCP_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(VCP_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D5_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(ARDUINO_PWM_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_INT_Pin */
  GPIO_InitStruct.Pin = LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_RX_D0_Pin ARDUINO_TX_D1_Pin */
  GPIO_InitStruct.Pin = ARDUINO_RX_D0_Pin|ARDUINO_TX_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_NXT_Pin */
  GPIO_InitStruct.Pin = ULPI_NXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_SDNME_Pin PH3 */
  GPIO_InitStruct.Pin = FMC_SDNME_Pin|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D4_Pin ARDUINO_D2_Pin EXT_RST_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_STP_Pin ULPI_DIR_Pin */
  GPIO_InitStruct.Pin = ULPI_STP_Pin|ULPI_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_D1_Pin QSPI_D3_Pin QSPI_D0_Pin */
  GPIO_InitStruct.Pin = QSPI_D1_Pin|QSPI_D3_Pin|QSPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_RXER_Pin */
  GPIO_InitStruct.Pin = RMII_RXER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RMII_RXER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_SCL_Pin LCD_SDA_Pin */
  GPIO_InitStruct.Pin = LCD_SCL_Pin|LCD_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_CLK_Pin ULPI_D0_Pin */
  GPIO_InitStruct.Pin = ULPI_CLK_Pin|ULPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D6_Pin */
  //GPIO_InitStruct.Pin = ARDUINO_PWM_D6_Pin;
  //GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  //GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
  //HAL_GPIO_Init(ARDUINO_PWM_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_MISO_D12_Pin ARDUINO_MOSI_PWM_D11_Pin */
  //GPIO_InitStruct.Pin = ARDUINO_MISO_D12_Pin|ARDUINO_MOSI_PWM_D11_Pin;
  //GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  //GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  //HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /*Configure GPIO pin : PI0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
	
  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, ARDUINO_D7_Pin|ARDUINO_D8_Pin|LCD_DISP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DCMI_PWR_EN_GPIO_Port, DCMI_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
	/* ^                                              ^ */
	/*  ^                                            ^  */
	/* - ^                                          ^ - */
	/* -- ^                                        ^ -- */
	/* --- ^                                      ^ --- */
	/* ---- ^                                    ^ ---- */
	/* ----- ^                                  ^ ----- */
	/* ------ ^                                ^ ------ */
	/* ------- ^                              ^ ------- */
	/* -------- ^                            ^ -------- */
	/* --------- ^                          ^ --------- */
	/* ---------- ^                        ^ ---------- */
	/* ----------- ^                      ^ ----------- */
	/* ------------ ^                    ^ ------------ */
	/* ------------- ^                  ^ ------------- */
	/* -------------- ^                ^ -------------- */
	/* --------------- ^              ^ --------------- */
	/* ---------------- ^            ^ ---------------- */
	/* ----------------- ^          ^ ----------------- */
	/* ------------------ ^        ^ ------------------ */
	/* ------------------- ^      ^ ------------------- */
	/* -------------------- ^    ^ -------------------- */
	/* --------------------- ^  ^ --------------------- */
	/* ---------------------- ^^ ---------------------- */
	/* --------------------- ^^ --------------------- */
	/* -------------------- ^^ -------------------- */
	/* ------------------- ^^ ------------------- */
	/* ------------------ ^^ ------------------ */
	/* ----------------- ^^ ----------------- */
	/* ---------------- ^^ ---------------- */
	/* --------------- ^^ --------------- */
	/* -------------- ^^ -------------- */
	/* ------------- ^^ ------------- */
	/* ------------ ^^ ------------ */
	/* ----------- ^^ ----------- */
	/* ---------- ^^ ---------- */
	/* --------- ^^ --------- */
	/* -------- ^^ -------- */
	/* ------- ^^ ------- */
	/* ------ ^^ ------ */
	/* ----- ^^ ----- */
	/* ---- ^^ ---- */
	/* --- ^^ --- */
	/* -- ^^ -- */
	/* - ^^ - */
	/*  ^^  */
	/* ^^ */

void init_matixes()
{
		
Ku[0] = 0.507820f;
Ku[1] = 0.512982f;
Ku[2] = 0.518085f;
Ku[3] = 0.523128f;
Ku[4] = 0.528114f;
Ku[5] = 0.533041f;
Ku[6] = 0.537912f;
Ku[7] = 0.542726f;
Ku[8] = 0.547485f;
Ku[9] = 0.552188f;
Ku[10] = 0.556837f;
Ku[11] = 0.561433f;
Ku[12] = 0.565975f;
Ku[13] = 0.570465f;
Ku[14] = 0.574903f;
Ku[15] = 0.579289f;
Ku[16] = 0.583625f;
Ku[17] = 0.587911f;
Ku[18] = 0.592147f;
Ku[19] = 0.596334f;
Ku[20] = 0.600473f;
Ku[21] = 0.593531f;
Ku[22] = 0.586669f;
Ku[23] = 0.579886f;
Ku[24] = 0.573182f;
Ku[25] = 0.566556f;
Ku[26] = 0.560006f;
Ku[27] = 0.553532f;
Ku[28] = 0.547132f;
Ku[29] = 0.540807f;
Ku[30] = 0.534555f;
Ku[31] = 0.528374f;
Ku[32] = 0.522266f;
Ku[33] = 0.516228f;
Ku[34] = 0.510260f;
Ku[35] = 0.504360f;
Ku[36] = 0.498529f;
Ku[37] = 0.492766f;
Ku[38] = 0.487069f;
Ku[39] = 0.481438f;
Ku[40] = 0.475872f;
Ku[41] = 0.470370f;
Ku[42] = 0.464932f;
Ku[43] = 0.459556f;
Ku[44] = 0.454243f;
Ku[45] = 0.448992f;
Ku[46] = 0.443801f;
Ku[47] = 0.438669f;
Ku[48] = 0.433598f;
Ku[49] = 0.428585f;
Ku[50] = 0.423629f;
Ku[51] = 0.418732f;
Ku[52] = 0.413890f;
Ku[53] = 0.409105f;
Ku[54] = 0.404375f;
Ku[55] = 0.399700f;
Ku[56] = 0.395078f;
Ku[57] = 0.390511f;
Ku[58] = 0.385995f;
Ku[59] = 0.381533f;
Ku[60] = 0.377121f;
Ku[61] = 0.372761f;
Ku[62] = 0.368451f;
Ku[63] = 0.364191f;
Ku[64] = 0.359980f;
Ku[65] = 0.355818f;
Ku[66] = 0.351704f;
Ku[67] = 0.347637f;
Ku[68] = 0.343618f;
Ku[69] = 0.339645f;
Ku[70] = 0.335717f;
Ku[71] = 0.331836f;
Ku[72] = 0.327999f;
Ku[73] = 0.324206f;
Ku[74] = 0.320457f;
Ku[75] = 0.316752f;
Ku[76] = 0.313089f;
Ku[77] = 0.309469f;
Ku[78] = 0.305891f;
Ku[79] = 0.302354f;
Ku[80] = 0.298857f;
Ku[81] = 0.295402f;
Ku[82] = 0.291986f;
Ku[83] = 0.288609f;
Ku[84] = 0.285272f;
Ku[85] = 0.281973f;
Ku[86] = 0.278713f;
Ku[87] = 0.275490f;
Ku[88] = 0.272304f;
Ku[89] = 0.269155f;
Ku[90] = 0.266043f;
Ku[91] = 0.262966f;
Ku[92] = 0.259925f;
Ku[93] = 0.256919f;
Ku[94] = 0.253948f;
Ku[95] = 0.251011f;
Ku[96] = 0.248109f;
Ku[97] = 0.245239f;
Ku[98] = 0.242403f;
Ku[99] = 0.239600f;
Ku[100] = 0.236829f;
Ku[101] = 0.234090f;
Ku[102] = 0.231383f;
Ku[103] = 0.228707f;
Ku[104] = 0.226062f;
Ku[105] = 0.223447f;
Ku[106] = 0.220863f;
Ku[107] = 0.218308f;
Ku[108] = 0.215784f;
Ku[109] = 0.213288f;
Ku[110] = 0.210821f;
Ku[111] = 0.208383f;
Ku[112] = 0.205972f;
Ku[113] = 0.203590f;
Ku[114] = 0.201235f;
Ku[115] = 0.198908f;
Ku[116] = 0.196607f;
Ku[117] = 0.194333f;
Ku[118] = 0.192085f;
Ku[119] = 0.189863f;
Ku[120] = 0.187667f;
Ku[121] = 0.185496f;
Ku[122] = 0.183350f;
Ku[123] = 0.181229f;
Ku[124] = 0.179133f;
Ku[125] = 0.177060f;
Ku[126] = 0.175012f;
Ku[127] = 0.172987f;
Ku[128] = 0.170986f;
Ku[129] = 0.169008f;
Ku[130] = 0.167053f;
Ku[131] = 0.165120f;
Ku[132] = 0.163210f;
Ku[133] = 0.161321f;
Ku[134] = 0.159455f;
Ku[135] = 0.157610f;
Ku[136] = 0.155786f;
Ku[137] = 0.153984f;
Ku[138] = 0.152202f;
Ku[139] = 0.150441f;
Ku[140] = 0.148700f;
Ku[141] = 0.146980f;
Ku[142] = 0.145279f;
Ku[143] = 0.143598f;
Ku[144] = 0.141936f;
Ku[145] = 0.140293f;
Ku[146] = 0.138670f;
Ku[147] = 0.137065f;
Ku[148] = 0.135479f;
Ku[149] = 0.133911f;
Ku[150] = 0.132361f;
Ku[151] = 0.130829f;
Ku[152] = 0.129315f;
Ku[153] = 0.127818f;
Ku[154] = 0.126338f;
Ku[155] = 0.124876f;
Ku[156] = 0.123430f;
Ku[157] = 0.122002f;
Ku[158] = 0.120589f;
Ku[159] = 0.119193f;
Ku[160] = 0.117813f;
Ku[161] = 0.116450f;
Ku[162] = 0.115101f;
Ku[163] = 0.113769f;
Ku[164] = 0.112451f;
Ku[165] = 0.111149f;
Ku[166] = 0.109862f;
Ku[167] = 0.108590f;
Ku[168] = 0.107333f;
Ku[169] = 0.106090f;
Ku[170] = 0.104861f;
Ku[171] = 0.103647f;
Ku[172] = 0.102446f;
Ku[173] = 0.101260f;
Ku[174] = 0.100087f;
Ku[175] = 0.098928f;
Ku[176] = 0.097782f;
Ku[177] = 0.096649f;
Ku[178] = 0.095530f;
Ku[179] = 0.094423f;
Ku[180] = 0.093329f;
Ku[181] = 0.092248f;
Ku[182] = 0.091179f;
Ku[183] = 0.090122f;
Ku[184] = 0.089078f;
Ku[185] = 0.088046f;
Ku[186] = 0.087026f;
Ku[187] = 0.086017f;
Ku[188] = 0.085020f;
Ku[189] = 0.084035f;
Ku[190] = 0.083061f;
Ku[191] = 0.082098f;
Ku[192] = 0.081146f;
Ku[193] = 0.080205f;
Ku[194] = 0.079276f;
Ku[195] = 0.078356f;
Ku[196] = 0.077448f;
Ku[197] = 0.076550f;
Ku[198] = 0.075662f;
Ku[199] = 0.074785f;
Ku[200] = 0.073917f;
Ku[201] = 0.073060f;
Ku[202] = 0.072213f;
Ku[203] = 0.071375f;
Ku[204] = 0.070547f;
Ku[205] = 0.069728f;
Ku[206] = 0.068919f;
Ku[207] = 0.068120f;
Ku[208] = 0.067329f;
Ku[209] = 0.066548f;
Ku[210] = 0.065775f;
Ku[211] = 0.065012f;
Ku[212] = 0.064257f;
Ku[213] = 0.063511f;
Ku[214] = 0.062774f;
Ku[215] = 0.062045f;
Ku[216] = 0.061324f;
Ku[217] = 0.060612f;
Ku[218] = 0.059908f;
Ku[219] = 0.059212f;
Ku[220] = 0.058524f;
Ku[221] = 0.057845f;
Ku[222] = 0.057172f;
Ku[223] = 0.056508f;
Ku[224] = 0.055851f;
Ku[225] = 0.055202f;
Ku[226] = 0.054560f;
Ku[227] = 0.053926f;
Ku[228] = 0.053299f;
Ku[229] = 0.052679f;
Ku[230] = 0.052067f;
Ku[231] = 0.051461f;
Ku[232] = 0.050863f;
Ku[233] = 0.050271f;
Ku[234] = 0.049686f;
Ku[235] = 0.049108f;
Ku[236] = 0.048536f;
Ku[237] = 0.047971f;
Ku[238] = 0.047413f;
Ku[239] = 0.046861f;
Ku[240] = 0.046315f;
Ku[241] = 0.045775f;
Ku[242] = 0.045242f;
Ku[243] = 0.044715f;
Ku[244] = 0.044194f;
Ku[245] = 0.043679f;
Ku[246] = 0.043170f;
Ku[247] = 0.042667f;
Ku[248] = 0.042169f;
Ku[249] = 0.041677f;
Ku[250] = 0.041191f;
Ku[251] = 0.040710f;
Ku[252] = 0.040235f;
Ku[253] = 0.039766f;
Ku[254] = 0.039302f;
Ku[255] = 0.038843f;
Ku[256] = 0.038389f;
Ku[257] = 0.037941f;
Ku[258] = 0.037497f;
Ku[259] = 0.037059f;
Ku[260] = 0.036626f;
Ku[261] = 0.036198f;
Ku[262] = 0.035774f;
Ku[263] = 0.035356f;
Ku[264] = 0.034942f;
Ku[265] = 0.034533f;
Ku[266] = 0.034129f;
Ku[267] = 0.033729f;
Ku[268] = 0.033334f;
Ku[269] = 0.032944f;
Ku[270] = 0.032558f;
Ku[271] = 0.032176f;
Ku[272] = 0.031799f;
Ku[273] = 0.031426f;
Ku[274] = 0.031057f;
Ku[275] = 0.030693f;
Ku[276] = 0.030332f;
Ku[277] = 0.029976f;
Ku[278] = 0.029624f;
Ku[279] = 0.029276f;
Ku[280] = 0.028932f;
Ku[281] = 0.028592f;
Ku[282] = 0.028255f;
Ku[283] = 0.027923f;
Ku[284] = 0.027594f;
Ku[285] = 0.027269f;
Ku[286] = 0.026948f;
Ku[287] = 0.026630f;
Ku[288] = 0.026316f;
Ku[289] = 0.026006f;
Ku[290] = 0.025699f;
Ku[291] = 0.025395f;
Ku[292] = 0.025095f;
Ku[293] = 0.024799f;
Ku[294] = 0.024506f;
Ku[295] = 0.024216f;
Ku[296] = 0.023929f;
Ku[297] = 0.023646f;
Ku[298] = 0.023366f;
Ku[299] = 0.023089f;
Ku[300] = 0.022815f;
Ku[301] = 0.022544f;
Ku[302] = 0.022277f;
Ku[303] = 0.022012f;
Ku[304] = 0.021751f;
Ku[305] = 0.021492f;
Ku[306] = 0.021236f;
Ku[307] = 0.020983f;
Ku[308] = 0.020734f;
Ku[309] = 0.020486f;
Ku[310] = 0.020242f;
Ku[311] = 0.020000f;
Ku[312] = 0.019761f;
Ku[313] = 0.019525f;
Ku[314] = 0.019292f;
Ku[315] = 0.019061f;
Ku[316] = 0.018832f;
Ku[317] = 0.018607f;
Ku[318] = 0.018383f;
Ku[319] = 0.018163f;
Ku[320] = 0.017944f;
Ku[321] = 0.017729f;
Ku[322] = 0.017515f;
Ku[323] = 0.017304f;
Ku[324] = 0.017096f;
Ku[325] = 0.016889f;
Ku[326] = 0.016685f;
Ku[327] = 0.016483f;
Ku[328] = 0.016284f;
Ku[329] = 0.016087f;
Ku[330] = 0.015892f;
Ku[331] = 0.015699f;
Ku[332] = 0.015508f;
Ku[333] = 0.015319f;
Ku[334] = 0.015133f;
Ku[335] = 0.014948f;
Ku[336] = 0.014766f;
Ku[337] = 0.014585f;
Ku[338] = 0.014407f;
Ku[339] = 0.014230f;
Ku[340] = 0.014056f;
Ku[341] = 0.013883f;
Ku[342] = 0.013712f;
Ku[343] = 0.013543f;
Ku[344] = 0.013376f;
Ku[345] = 0.013211f;
Ku[346] = 0.013048f;
Ku[347] = 0.012886f;
Ku[348] = 0.012726f;
Ku[349] = 0.012568f;
Ku[350] = 0.012412f;
Ku[351] = 0.012257f;
Ku[352] = 0.012104f;
Ku[353] = 0.011953f;
Ku[354] = 0.011803f;
Ku[355] = 0.011655f;
Ku[356] = 0.011509f;
Ku[357] = 0.011364f;
Ku[358] = 0.011220f;
Ku[359] = 0.011079f;
Ku[360] = 0.010938f;
Ku[361] = 0.010799f;
Ku[362] = 0.010662f;
Ku[363] = 0.010526f;
Ku[364] = 0.010392f;
Ku[365] = 0.010259f;
Ku[366] = 0.010127f;
Ku[367] = 0.009997f;
Ku[368] = 0.009868f;
Ku[369] = 0.009741f;
Ku[370] = 0.009615f;
Ku[371] = 0.009490f;
Ku[372] = 0.009366f;
Ku[373] = 0.009244f;
Ku[374] = 0.009123f;
Ku[375] = 0.009004f;
Ku[376] = 0.008885f;
Ku[377] = 0.008768f;
Ku[378] = 0.008652f;
Ku[379] = 0.008537f;
Ku[380] = 0.008423f;
Ku[381] = 0.008311f;
Ku[382] = 0.008200f;
Ku[383] = 0.008089f;
Ku[384] = 0.007980f;
Ku[385] = 0.007872f;
Ku[386] = 0.007765f;
Ku[387] = 0.007660f;
Ku[388] = 0.007555f;
Ku[389] = 0.007451f;
Ku[390] = 0.007348f;
Ku[391] = 0.007247f;
Ku[392] = 0.007146f;
Ku[393] = 0.007046f;
Ku[394] = 0.006948f;
Ku[395] = 0.006850f;
Ku[396] = 0.006753f;
Ku[397] = 0.006657f;
Ku[398] = 0.006562f;
Ku[399] = 0.006468f;
Ku[400] = 0.006375f;
Ku[401] = 0.006283f;
Ku[402] = 0.006192f;
Ku[403] = 0.006101f;
Ku[404] = 0.006011f;
Ku[405] = 0.005923f;
Ku[406] = 0.005835f;
Ku[407] = 0.005747f;
Ku[408] = 0.005661f;
Ku[409] = 0.005576f;
Ku[410] = 0.005491f;
Ku[411] = 0.005407f;
Ku[412] = 0.005323f;
Ku[413] = 0.005241f;
Ku[414] = 0.005159f;
Ku[415] = 0.005078f;
Ku[416] = 0.004998f;
Ku[417] = 0.004918f;
Ku[418] = 0.004839f;
Ku[419] = 0.004761f;
Ku[420] = 0.004683f;
Ku[421] = 0.004606f;
Ku[422] = 0.004530f;
Ku[423] = 0.004454f;
Ku[424] = 0.004379f;
Ku[425] = 0.004305f;
Ku[426] = 0.004231f;
Ku[427] = 0.004158f;
Ku[428] = 0.004085f;
Ku[429] = 0.004013f;
Ku[430] = 0.003941f;
Ku[431] = 0.003870f;
Ku[432] = 0.003800f;
Ku[433] = 0.003730f;
Ku[434] = 0.003661f;
Ku[435] = 0.003592f;
Ku[436] = 0.003524f;
Ku[437] = 0.003456f;
Ku[438] = 0.003389f;
Ku[439] = 0.003322f;
Ku[440] = 0.003256f;
Ku[441] = 0.003190f;
Ku[442] = 0.003124f;
Ku[443] = 0.003060f;
Ku[444] = 0.002995f;
Ku[445] = 0.002931f;
Ku[446] = 0.002867f;
Ku[447] = 0.002804f;
Ku[448] = 0.002741f;
Ku[449] = 0.002679f;
Ku[450] = 0.002617f;
Ku[451] = 0.002555f;
Ku[452] = 0.002494f;
Ku[453] = 0.002433f;
Ku[454] = 0.002372f;
Ku[455] = 0.002312f;
Ku[456] = 0.002252f;
Ku[457] = 0.002193f;
Ku[458] = 0.002134f;
Ku[459] = 0.002075f;
Ku[460] = 0.002016f;
Ku[461] = 0.001958f;
Ku[462] = 0.001900f;
Ku[463] = 0.001842f;
Ku[464] = 0.001785f;
Ku[465] = 0.001728f;
Ku[466] = 0.001671f;
Ku[467] = 0.001614f;
Ku[468] = 0.001558f;
Ku[469] = 0.001502f;
Ku[470] = 0.001446f;
Ku[471] = 0.001390f;
Ku[472] = 0.001335f;
Ku[473] = 0.001279f;
Ku[474] = 0.001224f;
Ku[475] = 0.001169f;
Ku[476] = 0.001115f;
Ku[477] = 0.001060f;
Ku[478] = 0.001006f;
Ku[479] = 0.000952f;
Ku[480] = 0.000899f;
Ku[481] = 0.000847f;
Ku[482] = 0.000795f;
Ku[483] = 0.000744f;
Ku[484] = 0.000693f;
Ku[485] = 0.000643f;
Ku[486] = 0.000594f;
Ku[487] = 0.000545f;
Ku[488] = 0.000496f;
Ku[489] = 0.000449f;
Ku[490] = 0.000401f;
Ku[491] = 0.000355f;
Ku[492] = 0.000308f;
Ku[493] = 0.000263f;
Ku[494] = 0.000218f;
Ku[495] = 0.000173f;
Ku[496] = 0.000129f;
Ku[497] = 0.000086f;
Ku[498] = 0.000043f;
}
