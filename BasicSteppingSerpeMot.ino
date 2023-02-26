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
  //in case of motor stall
}

void lscallback_test()
{
  //in case of limit switch reaching
}

void setup()
{
  Serial.begin(9600);
  M1 = new SerpeMot(M1_CS_pin, M1_PWM_pin, M1_STALL_pin, stlcallback_test, M1_Limit_pin, lscallback_test);
  while(!M1){};
  M1->setAngularSpeed(360,1000);
  delay(2000);
  M1->setAngularSpeed(360,1000);
  delay(2000);
  M1->setAngularSpeed(0,1000);
}

void loop()
{
}