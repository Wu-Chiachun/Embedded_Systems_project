/*
 * pages.c
 *
 *  Created on: May 4, 2023
 *      Author: wuchiachun
 */
#include "pages.h"


// - 24*32 - numbers for clock
const uint8_t num[][96] = {
{ 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
{ 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00},
{ 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
{ 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
{ 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0},
{ 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
{ 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
{ 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0},
{ 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
{ 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
};

// - 24*24 -
const uint8_t dew[] = { 0x00, 0xc0, 0x07, 0x00, 0x70, 0x02, 0x00, 0x18, 0x01, 0x00, 0x8c, 0x01, 0x00, 0x86, 0x00, 0x00, 0x83, 0x00, 0x80, 0x01, 0x01, 0x80, 0x00, 0x02, 0xc0, 0x60, 0x02, 0x60, 0x60, 0x06, 0x20, 0x00, 0x0c, 0x30, 0x80, 0x19, 0x10, 0x00, 0x1b, 0x18, 0x00, 0x12, 0x08, 0x00, 0x12, 0x08, 0x00, 0x12, 0x18, 0x00, 0x1b, 0x18, 0x00, 0x18, 0x38, 0x00, 0x0c, 0x70, 0x00, 0x0e, 0xf0, 0x00, 0x07, 0xe0, 0x01, 0x03, 0xc0, 0xff, 0x03, 0x80, 0xff, 0x01};
const uint8_t temperature[] = { 0x00, 0x3c, 0x00, 0x00, 0x24, 0x00, 0x00, 0x66, 0x00, 0x00, 0x42, 0x00, 0x00, 0x7a, 0x00, 0x00, 0x4a, 0x00, 0x00, 0x7a, 0x00, 0x00, 0x4a, 0x00, 0x00, 0x7a, 0x00, 0x00, 0x4a, 0x00, 0x00, 0x7a, 0x00, 0x00, 0x4a, 0x00, 0x00, 0x7a, 0x00, 0x00, 0x4a, 0x00, 0x00, 0x7a, 0x00, 0x00, 0x4a, 0x00, 0x00, 0x4a, 0x00, 0x00, 0xcb, 0x00, 0x80, 0x99, 0x01, 0x80, 0x3c, 0x01, 0x80, 0x3c, 0x01, 0x80, 0x98, 0x01, 0x00, 0xc3, 0x00, 0x00, 0x3e, 0x00};
const uint8_t light[] = { 0x00, 0x18, 0x00, 0x00, 0x18, 0x00, 0x0c, 0x18, 0x30, 0x1c, 0x18, 0x38, 0x38, 0x00, 0x1c, 0x70, 0x7e, 0x0e, 0x20, 0xc3, 0x04, 0x80, 0x81, 0x01, 0xc0, 0x00, 0x03, 0x60, 0x00, 0x06, 0x20, 0x00, 0x04, 0x2f, 0x00, 0xf4, 0x2f, 0x00, 0xf4, 0x20, 0x00, 0x04, 0x60, 0x00, 0x06, 0xc0, 0x00, 0x03, 0x80, 0x81, 0x01, 0x20, 0x83, 0x04, 0x70, 0x7e, 0x0e, 0x38, 0x00, 0x1c, 0x1c, 0x18, 0x38, 0x0c, 0x18, 0x30, 0x00, 0x18, 0x00, 0x00, 0x18, 0x00};
const uint8_t plus[] = { 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x3c, 0x00};
const uint8_t minus[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const uint8_t setting[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

void components_display(int temp, int humidity, int luminosity, int water_level, char watering_time[][32]){
	char temperature_value_display [16], humidity_value_display [16];
	char adc1_value_display [8], adc2_value_display [8];
	char watering_time_display[64];

	// clock
	LCD_Draw_Num(48, 60, num[1]);
	LCD_Draw_Num(82, 60, num[2]);
	LCD_Draw_Num(108, 60, num[10]);
	LCD_Draw_Num(134, 60, num[0]);
	LCD_Draw_Num(168, 60, num[8]);

    sprintf(watering_time_display, "Next watering time: %s", watering_time[0]);
	LCD_DrawString(30, 135, watering_time_display);

	//DHT11 Temperature Value and Display
    LCD_Draw_24sqr(20, 166, temperature);
    sprintf(temperature_value_display, "%d", temp);
	LCD_Clear(51, 167, 32, 16, 0xffff);
	LCD_DrawString(51, 167, temperature_value_display);
	LCD_DrawLine(50, 166 ,100, 166, 0x0);
	LCD_DrawLine(50, 186 ,100, 186, 0x0);
	LCD_DrawLine(50, 166 ,50, 186, 0x0);
	LCD_DrawLine(100, 166 ,100, 186, 0x0);

	//DHT11 Humidity Value and Display
	LCD_Draw_24sqr(20, 206, dew);
    sprintf(humidity_value_display, "%d", humidity);
	LCD_Clear(51, 207, 32, 16, 0xffff);
	LCD_DrawString(51, 207, humidity_value_display);
	LCD_DrawLine(50, 206 ,100, 206, 0x0);
	LCD_DrawLine(50, 226 ,100, 226, 0x0);
	LCD_DrawLine(50, 206 ,50, 226, 0x0);
	LCD_DrawLine(100, 206 ,100, 226, 0x0);


	//LDR Sensor Value and Display
	sprintf(adc2_value_display, "%d", luminosity);
	LCD_Clear(51, 247, 32, 16, 0xffff);
	LCD_DrawString(51, 247, adc2_value_display);
	LCD_Draw_24sqr(20, 246, light);
	LCD_DrawLine(50, 246 ,100, 246, 0x0);
	LCD_DrawLine(50, 266 ,100, 266, 0x0);
	LCD_DrawLine(50, 246 ,50, 266, 0x0);
    LCD_DrawLine(100, 246 ,100, 266, 0x0);

    // water level of the tank
    LCD_Clear (145, 210, 70, 50, 0xffff);
	sprintf(adc1_value_display, "%d", water_level);
	LCD_Clear(140, 160, 100, 20, 0xffff);
	water_level *= 0.02;
	LCD_DrawString(140, 160, adc1_value_display);
	LCD_Clear(140, 180, 100, 20, 0xffff);
	LCD_DrawString(140, 180, "Water level");
    LCD_DrawLine(140, 206, 140, 266, 0x0);
    LCD_DrawLine(220, 206, 220, 266, 0x0);
	LCD_DrawLine(140, 266, 220, 266, 0x0);
	LCD_Clear (145, 210, 70, water_level, 0x001f);
}

void HomePage(int temp, int humidity, int luminosity, int water_level, char watering_time[][32]){
	LCD_Draw_24sqr(10, 10, setting);
	components_display(temp, humidity, luminosity, water_level, watering_time);
	LCD_DrawLine(0, 152, 240, 152, 0x0);
}

void SetTimePage(char watering_time[][32]){
//	Setting Watering time
	strType_XPT2046_Coordinate touchpt;
	char touch_x[4], touch_y[4];
	uint8_t hr = 0, min = 0;
	LCD_Clear(0, 0, 240, 320, 0xffff);
	LCD_Draw_24sqr(10, 10, setting);
	LCD_DrawString(40, 50, "Set the watering time:");
	LCD_Draw_Num(48, 80, num[0]);
	LCD_Draw_Num(82, 80, num[0]);
	LCD_Draw_Num(108, 80, num[10]);
	LCD_Draw_Num(134, 80, num[0]);
	LCD_Draw_Num(168, 80, num[0]);

	LCD_DrawLine(0, 135, 240, 135, 0x0);

			while(1){
			  XPT2046_Get_TouchedPoint(&touchpt, &strXPT2046_TouchPara);
			  if(touchpt.x <= 270 && touchpt.x >= 235 && touchpt.y <= 120 && touchpt.y >= 60) hr += 1;
			  else if (touchpt.x <= 270 && touchpt.x >= 235 && touchpt.y <= 190 && touchpt.y >= 135) min += 1;
			  if(hr == 24) hr = 0;
			  if(min == 60) min = 0;
			  LCD_Draw_Num(48, 80, num[hr/10]);
			  LCD_Draw_Num(82, 80, num[hr%10]);
			  LCD_Draw_Num(108, 80, num[10]);
			  LCD_Draw_Num(134, 80, num[min/10]);
			  LCD_Draw_Num(168, 80, num[min%10]);
			  HAL_Delay(200);
	//		  LCD_Clear(48, 60, 192, 92, 0xffff);

	//		  LCD_Clear(20, 200, 220, 20, 0xffff);
			  XPT2046_Get_TouchedPoint(&touchpt, &strXPT2046_TouchPara);
			  if(touchpt.x >= 290 && touchpt.y<=80) break;
			  sprintf(touch_x, "%03d", touchpt.x);
			  sprintf(touch_y, "%03d", touchpt.y);
			  LCD_DrawString(20, 200, touch_x);
			  LCD_DrawString(200, 200, touch_y);
			}
		    LCD_Clear(0, 0, 240, 320, 0xffff);
			sprintf(watering_time, "%d%d:%d%d", hr/10, hr%10, min/10, min%10);
}

void RecordsPage(){
	strType_XPT2046_Coordinate touchpt;
	LCD_Clear(0, 0, 240, 320, 0xffff);
	LCD_Draw_24sqr(10, 10, setting);
	LCD_Draw_24sqr(50, 10, temperature);
	LCD_Draw_24sqr(100, 10, dew);
	LCD_Draw_24sqr(150, 10, light);
	LCD_DrawLine(50, 40 ,200, 40, 0x0);
	LCD_DrawLine(75, 10 ,75, 300, 0x0);
	LCD_DrawLine(125, 10 ,125, 300, 0x0);

	while(1){
	  XPT2046_Get_TouchedPoint(&touchpt, &strXPT2046_TouchPara);
	  if(touchpt.x >= 290 && touchpt.y<=80) break;
	}
	LCD_Clear(0, 0, 240, 320, 0xffff);

}


void ImgPage(){}