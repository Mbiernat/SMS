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
uint32_t ts_colors[] = {
	LCD_COLOR_BLUE          ,
	LCD_COLOR_GREEN         ,
	LCD_COLOR_RED           ,
	LCD_COLOR_CYAN          ,
	LCD_COLOR_MAGENTA       ,
	LCD_COLOR_YELLOW        ,
	LCD_COLOR_LIGHTBLUE     ,
	LCD_COLOR_LIGHTGREEN    ,
	LCD_COLOR_LIGHTRED      ,
	LCD_COLOR_LIGHTCYAN     ,
	LCD_COLOR_LIGHTMAGENTA  ,
	LCD_COLOR_LIGHTYELLOW   ,
	LCD_COLOR_DARKBLUE      ,
	LCD_COLOR_DARKGREEN     ,
	LCD_COLOR_DARKRED       ,
	LCD_COLOR_DARKCYAN      ,
	LCD_COLOR_DARKMAGENTA   ,
	LCD_COLOR_DARKYELLOW    ,
	LCD_COLOR_WHITE         ,
	LCD_COLOR_LIGHTGRAY     ,
	LCD_COLOR_GRAY          ,
	LCD_COLOR_DARKGRAY      ,
	LCD_COLOR_BLACK         ,
	LCD_COLOR_BROWN         ,
	LCD_COLOR_ORANGE        
};
char text[100] = {0};

/* Private variables ---------------------------------------------------------*/
uint8_t *resp;
uint16_t resplen;
MB_RESPONSE_STATE respstate;
uint8_t fan_on[] =     {0x00, 0x00, 0x03, 0xE8};    //{0x00, 0x01, 0x03, 0xE8};
uint8_t fan_half[] =   {0x00, 0x00, 0x01, 0xF4};
uint8_t fan_off[] =    {0x00, 0x00, 0x00, 0x00};
uint8_t heater_on[] =  {0x00, 0x04, 0x03, 0xE8};
uint8_t heater_half[] ={0x00, 0x04, 0x01, 0xF4};
uint8_t heater_var[] = {0x00, 0x04, 0x01, 0xF4};
uint8_t heater_off[] = {0x00, 0x04, 0x00, 0x00};
uint8_t get_temp[] =   {0x00, 0x00, 0x00, 0x01};

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
uint16_t plot_x_min;
uint16_t plot_x_max;
uint16_t plot_y_min;
uint16_t plot_y_max;
uint16_t zakres_y_min;
uint16_t zakres_y_max;

uint16_t v_range[2] = {0, 4095};

uint16_t	x;
uint16_t	y_wyj;
uint16_t	y_ster;
uint16_t	y_zad;

uint16_t button_width;
uint16_t button_height;
	
uint16_t left_buttons_x;
uint16_t right_buttons_x;
	
uint16_t up_buttons_y;
uint16_t down_buttons_y;

//=================================
// Dodatkowe zmienne

float Sterowanie;
float Wart_zadana;
float Wyjscie;
float U_set;
float Y_zad;

bool Temp_alarm;

//char * string;

#define REAL_OBJ		0
#define SIMULATION	1
int mode;

#define AUTO		0
#define MANUAL	1
int control_mode;

// Przyciski do manipulacji wartosci
#define STER_PLUS		0
#define STER_MINUS	1
#define ZAD_PLUS		2
#define ZAD_MINUS		3

//=================================

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
	MB_Config(115200);
		
	BSP_LCD_SetFont(&Font8); 
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE); 
	BSP_TS_Init(0,0);
	
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0); 
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
		
	HAL_NVIC_EnableIRQ(TIM2_IRQn); // enabling handling functions of TIM2
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
	
	
	//====================================================================================
	//====================================================================================
	
	
	// text
	uint16_t y_text = BSP_LCD_GetYSize() / 20;
	
	uint16_t left_x_text = (uint16_t)BSP_LCD_GetXSize() / 4;
	uint16_t right_x_text = 3 * (uint16_t)BSP_LCD_GetXSize() / 4;
	
	BSP_LCD_DrawVLine(BSP_LCD_GetXSize() / 2, 0, BSP_LCD_GetYSize());
	
	/* Displaying text */
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_DisplayStringAt(left_x_text, y_text, "PANEL STEROWANIA", LEFT_MODE);
		BSP_LCD_DisplayStringAt(right_x_text, y_text, "PLOT AND DATA", LEFT_MODE);
		
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/32, up_buttons_y   - 20, "Wartosc zadana:", LEFT_MODE);
		BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/32, down_buttons_y - 20, "Sterowanie", LEFT_MODE);
		
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	  BSP_LCD_DrawLine(x, zakres_y_min, x, zakres_y_max);
		
	
	/* Buttons data */
		button_width  = BSP_LCD_GetXSize() * 0.15; // 	/ 15; 
		button_height = BSP_LCD_GetYSize() * 0.10;	// 	/ 20;
		
		left_buttons_x  = (uint16_t)BSP_LCD_GetXSize * 0.05; 	// 	/ 4 - (button_width / 2);
		right_buttons_x = (uint16_t)BSP_LCD_GetXSize * 0.30;		//	/ 4 - (button_width / 2);
		
		up_buttons_y   = BSP_LCD_GetYSize() * 0.40; 	// - button_height;
		down_buttons_y = BSP_LCD_GetYSize() * 0.50;  // - button_height;
	
	
	
	
	
	plot_x_min = BSP_LCD_GetXSize() / 2;
	plot_x_max = BSP_LCD_GetXSize();
	plot_y_min = BSP_LCD_GetYSize() / 2;
	plot_y_max = BSP_LCD_GetYSize() / 2;
	
	x = plot_x_min;
	
	y_wyj = 0;
	y_ster = 0;
	y_zad = 0;
	
	zakres_y_min = 0.15*BSP_LCD_GetYSize();
	zakres_y_max = 0.85*BSP_LCD_GetYSize();
	
	
	
	//=============================
	// Init project vars
	
	mode = REAL_OBJ;
	control_mode = AUTO;
	Sterowanie  = 0.0;
	Y_zad				= 30.0;
	
	Temp_alarm = false;
	
//	int t = 0;
//	//int y = 0;
//	bool flag = 0;
//	bool touched = 0;
//	
//	int prev = 0;
//	int act = 0;
	//=============================
	
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
					BSP_LCD_DrawLine(i, zakres_y_min, i, zakres_y_max);
					x = plot_x_min;
				}
			} 
			else if( Touched_Button(STER_PLUS) && (control_mode == MANUAL) )
			{			
				while(TS_State.touchDetected);
				Sterowanie++;
				if(Sterowanie >   50.0f) Sterowanie =  50.0f;			
			}
			else if( Touched_Button(STER_MINUS) && (control_mode == MANUAL) )
			{
				while(TS_State.touchDetected);
				Sterowanie--;
				if(Sterowanie <  -50.0f) Sterowanie = -50.0f;
			}
			else if( Touched_Button(ZAD_PLUS) && (control_mode == AUTO) )
			{
				while(TS_State.touchDetected);
				Y_zad--;
				if(Y_zad >  80.0f) Y_zad = 80.0f;
			}
			else if( Touched_Button(ZAD_MINUS) && (control_mode == AUTO) )
			{
				while(TS_State.touchDetected);
				Y_zad--;
				if(Y_zad <  24.0f) Y_zad = 24.0f;
			}
		}
		
//		if(tims[5])BSP_LCD_SetTextColor(LCD_COLOR_BLACK); 
//		else BSP_LCD_SetTextColor(LCD_COLOR_WHITE); 
//		BSP_LCD_DisplayStringAt( 10, 50, (uint8_t*)"TIM5", LEFT_MODE);
	
		
		/* PushButton usage */
		if(BSP_PB_GetState(BUTTON_TAMPER))
		{
			while(BSP_PB_GetState(BUTTON_TAMPER));
			if(control_mode == AUTO)
				control_mode = MANUAL;
			else
				control_mode = AUTO;
		}												 
		
		/* Displaying texts */
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	
		sprintf(text,"Wart zadana: %f", Wart_zadana);
		BSP_LCD_DisplayStringAt(100,10, (uint8_t*)text, LEFT_MODE);
		
		sprintf(text,"Wyjscie    : %f", Wyjscie);
		BSP_LCD_DisplayStringAt(110,10, (uint8_t*)text, LEFT_MODE);
		
		sprintf(text,"Sterowanie: %f", Sterowanie);
		BSP_LCD_DisplayStringAt(120,10, (uint8_t*)text, LEFT_MODE);
		
		if( mode == REAL_OBJ)	sprintf(text,"Czujnik sprawny. Sterowanie: obiekt");
		else 									sprintf(text,"Awaria czujnika. Sterowanie: model");
		BSP_LCD_DisplayStringAt(90,10, (uint8_t*)text, LEFT_MODE);
		
		if( control_mode == AUTO)	sprintf(text, "Tryb pracy: AUTO");
		else 											sprintf(text, "Tryb pracy: MANUAL");
		BSP_LCD_DisplayStringAt(200,10, (uint8_t*)text, LEFT_MODE);
		
		/* Displaying buttons	*/
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
		
		
		
		/* Displaying alarm */
		if(Temp_alarm)
		{
			BSP_LCD_SetTextColor(LCD_COLOR_RED); 
			BSP_LCD_FillCircle(200, 20, 10);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			sprintf(text,"ALARM\nTemp. za niska!");
			BSP_LCD_DisplayStringAt(200,40, (uint8_t*)text, LEFT_MODE);
		}
		else
		{
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE); 
			BSP_LCD_FillCircle(200, 20, 10);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK); 
			BSP_LCD_DrawCircle(200, 20, 10);
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			sprintf(text,"ALARM\nTemp. za niska!");
			BSP_LCD_DisplayStringAt(200,40, (uint8_t*)text, LEFT_MODE);
		}
		
	
		/* Touch Screen and LCD usage */
/*		BSP_TS_GetState(&TS_State);		
		for(touch_index=0;touch_index<TS_State.touchDetected;++touch_index)
		{ 
			BSP_LCD_SetTextColor(ts_colors[touch_index]); 
			BSP_LCD_FillCircle( TS_State.touchX[touch_index], TS_State.touchY[touch_index], 2);
		}
*/		
	}
}

bool Touched_Button(int button)
{
	bool ret = false;
	bool x_left, x_right, y_up, y_down;
	
	x_left  = TS_State.touchX[touch_index] > left_buttons_x  && TS_State.touchX[touch_index] < (left_buttons_x  + button_width);
	x_right = TS_State.touchX[touch_index] > right_buttons_x && TS_State.touchX[touch_index] < (right_buttons_x + button_width);
	y_up		= TS_State.touchX[touch_index] > up_buttons_y    && TS_State.touchX[touch_index] < (up_buttons_y 		+ button_height);
	y_down	= TS_State.touchX[touch_index] > down_buttons_y  && TS_State.touchX[touch_index] < (down_buttons_y  + button_height);
	
	switch(button)
	{
		case STER_PLUS:
			if(x_left && y_down)	ret = true;		// w if() zdefiniowac obszar na ekranie w ktorym znajduje sie przycisk
			else ret = false;
			break;
		case STER_MINUS:
			if(x_right && y_down)	ret = true;
			else ret = false;
			break;
		case ZAD_PLUS:
			if(x_left && y_up)	ret = true;
			else ret = false;
			break;
		case ZAD_MINUS:
			if(x_right && y_up)	ret = true;
			else ret = false;
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

//============================
// Zmienne dla regulatorów 

// DMC
float D;
float Ke;
float Ku;
float Ku_mul_dUp[1];
float dUp[1][1];

//PID
float T  = 0.05;
float K  = 10;
float Td = 0;
float Ti = 999999999;
	
float r0;
float r1;
float r2;

float e_past_1;
float e_past_2;

// WSPOLNE
float e;
float u_past;

//===========================

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
			//u = U_set;
			u = Sterowanie;
		}
		else if(control_mode == AUTO)
		{
			if(mode == SIMULATION)
			{
				//===================================================
				// Regulator PID
				
				e_past_2 = e_past_1;
				e_past_1 = e;
				
				u_past = u;
				
				e = Y_zad - y;
				
				r2 = K*Td / T;
				r1 = K*(T / (2*Ti) - (2*(Td / T)) - 1 );
				r0 = K*(1+(T/(2*Ti))+(Td/T));
				
				u = r2 * e_past_2 + r1 * e_past_1 + r0 * e + u_past;
				//===================================================
			}
			
			else if(mode == REAL_OBJ)
			{
				//===================================================
				// Regulator DMC
				
				u_past = u;
				
				e = Y_zad - y;
				
				// przesun w dól wektor dUp i wpisz 
				// na pierwsze miejsce róznice sterowan
				// dUp postaci dUp[D-1][1] zeby mat_move_down dobrze liczylo
				mat_move_down( dUp, D-1, 1, 0, dUp);
				dUp[0][0] = u - u_past;
				
				// int Ku_mul_dUp[1] do przechowania wyniku mnozenia macierzy 
				mat_mul( Ku, 1, D-1, dUp, D-1, 1, Ku_mul_dUp);
				
				u = u_past + Ke * e - Ku_mul_dUp[0];
				
				Ke = 2.8490e-04;
				//===================================================
			}
			
		//	U_set = u;
			Sterowanie = u;
		}
		
		if(y < 24)
			Temp_alarm = true;
		else
			Temp_alarm = false;
		
		Wart_zadana = Y_zad;
		Wyjscie = y;
		
		//============================================================================
		
		/* przyklady tego, jak nalezy interpretowac poszczegolne wartosci sterowania */
//		u = -10.0; // grzanie z moca (-10+50)% =  40%
//		u =   0.0; // grzanie z moca (  0+50)% =  50%
//		u =  50.0; // grzanie z moca ( 50+50)% = 100%
		
			u = 10.0; // grzanie z moca (10+50)% =  60%
				
		/* aplikacja ograniczen na sygnal sterujacy */
		if(u >   50.0f) u =  50.0f;
		if(u <  -50.0f) u = -50.0f;
		
		/* skalowanie z -50..50 do 0..1000 */
		raw_u = (uint16_t)(u+50.0f)*10; // przejscie z -2048 - 2047 do 0 - 4095
		
		// skalowanie: (y - y(0)) / (0.6 * 4095 - 0.4 * 4095)
		
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
		sprintf(txt,"U=%5.2f;Y=%5.2f;", u, y);
	//sprintf(txt,"U=%8.2f;Y=%8.2f;", u, y); // 22 znaki
		
		if(HAL_UART_Transmit_IT(&huart,   (uint8_t*)txt, 14)!= HAL_OK) Error_Handler();
	} 
	if (htim->Instance == TIM3){ // timer odpowiedzialny za aktualizacje MB i odliczanie timeout'u
		MB();
		TimeoutTick();
	}
	if (htim->Instance == TIM4){ // timer odpowiedzialny za odliczanie kwantow 50us
		Timer50usTick();
	}
	if(htim->Instance == TIM5 ) 
	{
		x = (x == plot_x_max)? plot_x_min : x + 1;
		
//		y_wyj = zakres_y_min + (zakres_y_max - zakres_y_min - (adc_value/4095.0)*(zakres_y_max - zakres_y_min));
		y_wyj = (zakres_y_min + zakres_y_max) / 2;				// przykladowa wartosc
		y_ster = (zakres_y_min + zakres_y_max) / 2 + 20;	// przykladowa wartosc
		y_zad = (zakres_y_min + zakres_y_max) / 2 - 20;		// przykladowa wartosc
		
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	  BSP_LCD_DrawLine(x, zakres_y_min, x, zakres_y_max);
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
