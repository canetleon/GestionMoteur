#include <SPI.h>
#include <HighPowerStepperDriver.h>
#include "RP2040_PWM.h"
#include "SerpeMot.h"

#define M1_PWM_pin 25
#define M1_CS_pin 10
#define M1_STALL_pin 19
#define M1_Limit_pin 3

bool test = false;
int i = 0;
SerpeMot *M1;

void stlcallback_test()
{
}

void lscallback_test()
{
  //Serial.println("ls reached");
  test = true;

}

void setup()
{
  Serial.begin(9600);
  M1 = new SerpeMot(M1_CS_pin, M1_PWM_pin, M1_STALL_pin, stlcallback_test, M1_Limit_pin, lscallback_test);
  while(!M1){};
  //  M1->driver.setDirection(false);
   //M1->setFreq(1000,1000);
   //pinMode(25,OUTPUT);
   //RP2040_PWM* motor_pwm = new RP2040_PWM(M1_PWM_pin, 1000, 50);
   //if(motor_pwm) motor_pwm->setPWM(M1_PWM_pin,100,50);
  // delay(1000);
  // M1->setAngularSpeed(-360,500);
  // delay(1500);
  M1->setAngularSpeed(360,1000);
  // Serial.println("ok");
   
   
}

void loop()
{
  // M1->setAngularSpeed(360,5000);
  // M1->setAngularSpeed(-360,5000);

}