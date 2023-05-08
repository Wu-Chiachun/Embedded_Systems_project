/*
 * pages.h
 *
 *  Created on: May 4, 2023
 *      Author: wuchiachun
 */

#ifndef INC_PAGES_H_
#define INC_PAGES_H_

#include "lcd.h"
#include "xpt2046.h"
#include "stdio.h"

void components_display(int temp, int humidity, int luminosity, int water_level, char watering_time[], uint8_t hr, uint8_t min);
// Pages
void TimeInitPage(uint8_t sec, uint8_t min, uint8_t hr, uint8_t weekday, uint8_t date, uint8_t month, uint8_t year, RTC_TimeTypeDef sTime, RTC_DateTypeDef sDate, RTC_HandleTypeDef hrtc);
void HomePage(int temp, int humidity, int luminosity, int water_level, char watering_time[], RTC_TimeTypeDef sTime, RTC_DateTypeDef sDate, RTC_HandleTypeDef hrtc);
void SetTimePage(char watering_time[], char watering_time2[]);
void RecordsPage(char records[10][3][8], uint8_t count);
void ImgPage(uint8_t Ov7725_vsync);

#endif /* INC_PAGES_H_ */
