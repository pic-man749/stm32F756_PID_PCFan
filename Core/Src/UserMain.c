/*
 * UserMain.c
 *
 *  Created on: Feb 10, 2024
 *      Author: yui
 */
#include <stdlib.h>
#include <stdbool.h>
#include "tim.h"
#include "usart.h"
#include "uart.h"

// define
#define TIMER_CLOCK 100000000.0

// global
uint32_t icBuffer[2];
double nowRpm = 0;
double targetRpm = 3000;
double nowPwm = 0;
double K_p = 0.02;
double K_i = 0.0;
double K_d = 0.0;
bool isParamUpdate = true;
bool isTargetRpmUpdated = true;

typedef enum {
  targetValue_none,
  targetValue_RPM,
  targetValue_Kp,
  targetValue_Ki,
  targetValue_Kd
}targetValue_e;

// PWM値を設定する関数
// num : add/sub value
void setFanSpeed(double num){
  nowPwm += num;

  if(nowPwm < 0){
    nowPwm = 0;
  }else if((1000-1) < nowPwm){
    nowPwm = 1000-1;
  }
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)nowPwm);
}

// 比例制御
double proportionalCtrl(double e){
  return K_p * e;
}
// 積分制御
double integralCtrl(double e, double le, double dt){
  static double sum = 0;
  if(isTargetRpmUpdated){
    sum = 0;
    isTargetRpmUpdated = false;
  }
  sum += (e + le) / 2.0 * dt;
  return K_i * (sum);
}
// 微分制御
double derivativeCtrl(double e, double le, double dt){
  return K_d * (e - le) / dt;
}

// 制御器
// e :誤差[rpm]
// dt:時間[s]
void fanCtrl(double e, double dt){
  // 前回値
  static double last_e = 0;
  // PID
  nowPwm += proportionalCtrl(e) + integralCtrl(e, last_e, dt) + derivativeCtrl(e, last_e, dt);
  // Limiter
  if(nowPwm < 0){
    nowPwm = 0;
  }else if((1000-1) < nowPwm){
    nowPwm = 1000-1;
  }
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)nowPwm);
  // 前回値を更新する
  last_e = e;
}

void setup(){
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

  HAL_TIM_Base_Start(&htim3);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 250-1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  startRecv();
}

void loop(){

  // recv UART 1st char(cmd)
  uint8_t c = sgetc();
  targetValue_e target;

  switch(c){
    case 'r':
      target = targetValue_RPM;
      break;
    case 'p':
      target = targetValue_Kp;
      break;
    case 'i':
      target = targetValue_Ki;
      break;
    case 'd':
      target = targetValue_Kd;
      break;
    default:
      target = targetValue_none;
      break;
  }

  // recv UART value
  static char tmp[32] = {0};
  uint8_t idx = 0;

  while(1){
    uint8_t c = sgetc();

    if(('0' <= c && c <= '9') || c == '.'){
      tmp[idx++] = c;

    }else if('\r' == c){
      tmp[idx++] = '\0';
      isParamUpdate = true;
      switch(target){
        case targetValue_RPM:
          targetRpm = atof(tmp);
          isTargetRpmUpdated = true;
          break;
        case targetValue_Kp:
          K_p = atof(tmp);
          break;
        case targetValue_Ki:
          K_i = atof(tmp);
          break;
        case targetValue_Kd:
          K_d = atof(tmp);
          break;
        default:
          isParamUpdate = false;
          break;
      }
      break;
    }

  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef * htim){
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
    static uint32_t oldCaptureVal = 0;
    static bool skipFlag = true;

    if(skipFlag){
      skipFlag = false;
      return;
    }

    skipFlag = true;

    uint32_t captureVal = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);

    uint32_t subCntr;
    if(oldCaptureVal < captureVal){
      subCntr = captureVal - oldCaptureVal;
    }else{
      subCntr = (captureVal + 0x100000000) - oldCaptureVal;
    }
    double dt = (double)subCntr / TIMER_CLOCK;
    double hz = 1.0 / dt;
    nowRpm = hz * 60;

    oldCaptureVal = captureVal;

    // ctrl
    double e = targetRpm - nowRpm;
    // setFanSpeed(e);
    fanCtrl(e, dt);

    // send result
    static const char formatStr[] = "rpm:%.2f,PWM:%.1f\n";
    static char serialBuffer[128];
    uint16_t size = sprintf(serialBuffer, formatStr, nowRpm, nowPwm*0.1);
    HAL_UART_Transmit(&huart3, (uint8_t *)serialBuffer, size, 100);

    // send param if updated
    if(isParamUpdate){
      static const char formatStrParam[] = "TargetRpm:%d,Kp:%f,Ki:%f,Kd:%f\n";
      uint16_t size = sprintf(serialBuffer, formatStrParam, (int)targetRpm, K_p, K_i, K_d);
      HAL_UART_Transmit(&huart3, (uint8_t *)serialBuffer, size, 100);
      isParamUpdate = false;
    }
  }
}
