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

void components_display(int temp, int humidity, int luminosity, int water_level, char watering_time[][32]);
// Pages
void TimeInitPage(uint8_t sec, uint8_t min, uint8_t hr, uint8_t weekday, uint8_t date, uint8_t month, uint8_t year, RTC_TimeTypeDef sTime, RTC_DateTypeDef sDate, RTC_HandleTypeDef hrtc);
void HomePage(int temp, int humidity, int luminosity, int water_level, char watering_time[][32]);
void SetTimePage(char watering_time[][32], RTC_TimeTypeDef sTime);
void RecordsPage();
void ImgPage();

#endif /* INC_PAGES_H_ */
