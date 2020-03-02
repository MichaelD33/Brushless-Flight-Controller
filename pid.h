#pragma once
#ifndef PID_h
#define PID_h

#include "imu.h"
#include "RX.h"
#include "config.h"

/*  DEFINE PROPORTIONAL CONSTANTS */
#define KpX 0.15
#define KpY 0.15
#define KpZ 0.00


/*  DEFINE INTEGRAL AND DERIVATIVE CONSTANTS ACCORDING TO PID SAMPLE TIME */    
#define KiX 0.000000 * SAMPLETIME_S  //0000005
#define KiY 0.000000 * SAMPLETIME_S  //0000005
#define KiZ 0.0 * SAMPLETIME_S  //0000005

#define KdX 0 / SAMPLETIME_S  //10
#define KdY 0 / SAMPLETIME_S  //10
#define KdZ 0 / SAMPLETIME_S  //7
  



void initPids();
void resetPids();
void computePids();
int_pwmOut motorPwmOut();

#endif
