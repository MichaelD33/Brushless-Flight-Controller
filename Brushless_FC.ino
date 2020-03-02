#include <Arduino.h>
#include <DShot.h>
#include "imu.h"
#include "RX.h"
#include "pid.h"
#include "SBUS.h"
#include "config.h"

DShot motor1;
DShot motor2;
DShot motor3;
DShot motor4;

bool armState = false;
bool lastArmState = false;

long indexTime = 0;
long lastStart = 0;

long count = 0;

void setup() {

  #ifdef SERIAL_DEBUGGING
    Serial.begin(115200);
  #endif

/*

//  DDRB = DDRB | B11110000; //sets pins D8, D9, D10, D11 as outputs
  DDRC = DDRC | B11000000; //sets pin D5 as output
  DDRD = DDRD | B10000000; //sets pin D6 as output
  DDRE = DDRE | B01000000; //sets pin D7 as output

*/

  // Set pins 8, 9, 10, and 11 as DSHOT ESC outputs
  motor1.attach(8);
  motor2.attach(9);
  motor3.attach(10);
  motor4.attach(11); 

  delay(1000); 

  initSbus();  //connect to the remote reciever (rx.cpp)
  initIMU();   //activate the imu and set gyroscope and accelerometer sensitivity (imu.cpp)

  indexTime = micros();

}

void loop() {

   measureLoopTime(); // currently the sampling is not fixed at a specific time (I just want to see how long it takes)
  
  
//      ** IMU DATA COLLECTION **       
   readIMU(); //read the imu and calculate the quadcopters position relative to gravity (imu.cpp)
              // IMU supports up to 8kHz gyro update rate and 1kHz acc update rate --- when DLPF is activated this is diminished significantly (see MPU6050 register mapping datasheet)

//        ** RX DATA COLLECTION **      
   readRx();  //read the remote and convert data to a rotational rate

   if(failsafeState() == 0){
     switch(chAux1()){
       case 0: //if the arm switch is set to 0, do not enable the quadcopter
         armState = false; break;

       case 1: //if the arm  switch is set to 1, start the PID calculation

/*      ** PROCESS INPUT DATA THROUGH PID CONTROLLER **      */ 
          initPids();

/*      ** SET MOTOR SPEEDS **                  */
          motor1.setThrottle(motorPwmOut().one);
          motor2.setThrottle(motorPwmOut().two);
          motor3.setThrottle(motorPwmOut().three);
          motor4.setThrottle(motorPwmOut().four);
 

/*                                            
          motor1.setThrottle(chThrottle());
          motor2.setThrottle(chThrottle());
          motor3.setThrottle(chThrottle());
          motor4.setThrottle(chThrottle());
   */

          armState = true;
          break;
       
       case 2:
        armState = false; break;

       default:
        armState = false; break;
     }
     
   }else if(failsafeState() != 0){
    
      //turn motors off if failsafe is triggered
      Serial.print("Exception 1: FAILSAFE TRIGGERED ... ");
      Serial.print(failsafeState());
      armState = false;
  
   }else{
    
      //turn motors off if something else happens
      Serial.print("Exception 2: Unknown ... ");
      Serial.print(failsafeState());
      armState = false;
    
   }

  if(armState == false && lastArmState == true){
//    IF DEVICE DISARMS ——> DISABLE MOTORS
/**/
      motor1.setThrottle(200);
      motor2.setThrottle(200);
      motor3.setThrottle(200);
      motor4.setThrottle(200);

  }

  lastArmState = armState;


  #ifdef SERIAL_DEBUGGING
  if(chAux2() == 1){
    Serial.println(imu_angles().y);
  }
  #endif

  if (Serial.available()>0){
    if(Serial.parseInt() == 9){
      
      motor1.setThrottle(300);
      motor2.setThrottle(300);
      motor3.setThrottle(300);
      motor4.setThrottle(300);
      
      Serial.println("Exception 3: User Shutoff ... Please restart device");
      delay(1000);
      exit(0);
    }
  }

}


void measureLoopTime(){


  while((micros() - lastStart) < SAMPLETIME){
    indexTime = micros();
  }

  long lastLoopTime = (indexTime - lastStart);

  #ifdef SERIAL_DEBUGGING
  if(chAux2() == 0){
    Serial.print("Last loop duration (µs): ");
    Serial.println(lastLoopTime);
  }
  #endif
  
  lastStart = indexTime;

  count++;
 
  
}

int armingState(){
  return armState;
}

int lastArmingState(){
  return lastArmState;
}
