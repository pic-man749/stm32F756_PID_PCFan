/*
 * UserMain.h
 *
 *  Created on: Feb 10, 2024
 *      Author: yui
 */

#ifndef INC_USERMAIN_H_
#define INC_USERMAIN_H_

void setup();
void loop();

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef * htim);

#endif /* INC_USERMAIN_H_ */
