#define DHT11_PORT GPIOE
#define DHT11_PIN GPIO_PIN_6

#include "main.h"
#include "stdio.h"

TIM_HandleTypeDef htim6;

void delay (uint16_t time_us){
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	HAL_TIM_Base_Start(&htim6);
	while(__HAL_TIM_GET_COUNTER(&htim6)<=time_us);
	HAL_TIM_Base_Stop(&htim6);
}

void Set_Pin_Output(GPIO_TypeDef* port, uint16_t pin){
	GPIO_InitTypeDef  GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(port, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef* port, uint16_t pin){
	GPIO_InitTypeDef  GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(port, &GPIO_InitStruct);
}

void DHT11_Start (void)
{

	Set_Pin_Output (DHT11_PORT, DHT11_PIN);  // set the pin as output
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
	HAL_Delay(18);  // wait for 18ms
	Set_Pin_Input(DHT11_PORT, DHT11_PIN);    // set as input


}

uint8_t Check_Response (void)
{
	uint8_t Response = 0;
	char str[4];
	delay (60);
	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
	{
		delay (80);
		if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1;
		else Response = -1;
	}
	sprintf(str, "%d", Response);
	LCD_DrawString(10, 10, str);
	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go low
	LCD_DrawString(10, 10, "Fucking IDK!");
	return Response;
}

uint8_t DHT11_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go high
		delay (50);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));  // wait for the pin to go low
	}
	return i;
}
