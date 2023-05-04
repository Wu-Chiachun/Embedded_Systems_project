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
void HomePage(int temp, int humidity, int luminosity, int water_level, char watering_time[][32]);
void SetTimePage(char watering_time[][32]);
void RecordsPage();
void ImgPage();

#endif /* INC_PAGES_H_ */
