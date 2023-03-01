#ifndef SerpeMot_H
#define SerpeMot_H

#include <Arduino.h>
#include <HighPowerStepperDriver.h>
#include "RP2040_PWM.h"
//#include "TimerInterrupt_Generic.h"

#define _TIMERINTERRUPT_LOGLEVEL_     4
#define _PWM_LOGLEVEL_                3

class SerpeMot
{

  private:
    int instanceCount = 0;
    int step_pin;
    int dir_pin;
    int cs_pin;
    int nstall_pin;
    static int nlimit_pin;
    int acceleration = 1000;
    int freqStepSize = 10;
    int actualFreq = 0;
    int currentLimit_ma = 1000;
    uint16_t stepMode = 8;
    float theta_mot = 1.8;
    bool reverseDir = false;
    bool stepState;
    int nbSteps;
    
    //static MBED_RPI_PICO_TimerInterrupt timer1;
    
    static void (*stall_function)();
    static void (*limitSwitch_function)();
    void setDir(bool);

    static void stall_callback(void);
    static void limitSwitch_callback(void);
    static void interruptStep_callback(uint alarm_num);
    
    
  public:
    HighPowerStepperDriver driver;
    RP2040_PWM* motor_pwm;
    SerpeMot(int cs, int step, int dir, int nstall, void (*)(), int nlimit, void (*)());
    //~SerpeMot();

    void setRamp(int hertzPerSec);
    void setFreq(int freq);
    void setFreq(int freq, int hertzPerSec);
    void setAngularSpeed(int deg_s);
    void setAngularSpeed(int deg_s, int hertzPerSec);
    void setSteps(int steps, int freq);
    int getFreq(void);
    //implementer check statut driver
    //renvoyer si le moteur de la pince s'est arreté
    //renvoyer evenements fin de course
};
#endif