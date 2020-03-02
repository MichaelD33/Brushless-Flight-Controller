/*
    The PID Controller makes adjustments to the motor speeds in order to adjust orientation in the desired angle/direction
*/
#include "pid.h"
#include "config.h"

float outputX, outputY, outputZ;
float Ix, Iy, Iz;
unsigned long lastTime, currentT;

axis_int16_t desiredAngle;
axis_float_t currentAngle, lastAngle;
axis_float_t error, deltaError, errorSum;

int_pwmOut motorSpeed;

void initPids(){
  //time since last calculation
    
    computePids();
    resetPids();
    lastAngle.x = currentAngle.x;
    lastAngle.y = currentAngle.y;
    lastAngle.z = currentAngle.z;
    lastTime = currentT;
} 
    

void computePids(){

    //read angle from IMU and set it to the current angle
    currentAngle.x = imu_angles().x; 
    currentAngle.y = imu_angles().y;
    currentAngle.z = imu_angles().z;
    //currentAngle.z = 0;
    

    //present error
    error.x = (-1 * chRoll())  - currentAngle.x;                      
    error.y = (-1 * chPitch()) - currentAngle.y;
    error.z = chYaw()          - currentAngle.z;

    //compute integral of error
    Ix += KiX * error.x;
    Iy += KiY * error.y;
    Iz += KiZ * error.z;

    //derivative of error
    deltaError.x = currentAngle.x - lastAngle.x;        
    deltaError.y = currentAngle.y - lastAngle.y; 
    deltaError.z = currentAngle.z - lastAngle.z; 

    
    //clamp the range of integral values
    if(Ix > MAX_INTEGRAL){ 
      Ix = MAX_INTEGRAL; 
    }else if (Ix < (MIN_INTEGRAL)){
      Ix = (MIN_INTEGRAL);
    }
        
    if(Iy > MAX_INTEGRAL){ 
      Iy = MAX_INTEGRAL; 
    }else if (Iy < (MIN_INTEGRAL)){
      Iy = (MIN_INTEGRAL);
    }
        
    if(Iz > MAX_INTEGRAL){ 
      Iz = MAX_INTEGRAL; 
    }else if (Iz < (MIN_INTEGRAL)){
      Iz = (MIN_INTEGRAL);
    }

    outputX = (KpX * error.x + Ix - KdX * deltaError.x);
    outputY = (KpY * error.y + Iy - KdY * deltaError.y);
    outputZ = (KpZ * error.z + Iz - KdZ * deltaError.z);

/*
    //write outputs to corresponding motors at the corresponding speed
     motorSpeed.one = abs(chThrottle() + outputX - outputY - outputZ); 
     motorSpeed.two = abs(chThrottle() - outputX - outputY + outputZ); 
     motorSpeed.three = abs(chThrottle() - outputX + outputY - outputZ);
     motorSpeed.four = abs(chThrottle() + outputX + outputY + outputZ);
*/

     motorSpeed.one = abs(chThrottle() - outputY); 
     motorSpeed.two = abs(chThrottle() - outputY); 
     motorSpeed.three = abs(chThrottle() + outputY);
     motorSpeed.four = abs(chThrottle() + outputY);

    
     //clamp the min and max output from the pid controller (to match the needed 0-255 for pwm)
     if(motorSpeed.one > ESC_MAX){
        motorSpeed.one = ESC_MAX;  
       }else if (motorSpeed.one < ESC_MIN){
        motorSpeed.one = ESC_MIN;
       }else{ }  

     if(motorSpeed.two > ESC_MAX){
        motorSpeed.two = ESC_MAX;  
       }else if (motorSpeed.two < ESC_MIN){
        motorSpeed.two = ESC_MIN;
       }else{ } 

     if(motorSpeed.three > ESC_MAX){
        motorSpeed.three = ESC_MAX;  
       }else if (motorSpeed.three < ESC_MIN){
        motorSpeed.three = ESC_MIN;
       }else{ } 

     if(motorSpeed.four > ESC_MAX){
        motorSpeed.four = ESC_MAX;  
       }else if (motorSpeed.four < ESC_MIN){
        motorSpeed.four = ESC_MIN;
       }else{ } 


}

void resetPids(){

   if(chThrottle() < 10){    
      errorSum.x = 0;
      errorSum.y = 0;
      errorSum.z = 0;   
      
    }else if(armingState() != lastArmingState()){
    //reset the integral term when the quadcopter is armed
      errorSum.x = 0;
      errorSum.y = 0;
      errorSum.z = 0;
    
  }
       
}

int_pwmOut motorPwmOut(){
  return motorSpeed;
}
