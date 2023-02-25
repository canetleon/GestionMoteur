#include "SerpeMot.h"

//Static objects definition
void (*SerpeMot::stall_function)();

void (*SerpeMot::limitSwitch_function)();
int SerpeMot::step_pin = 2;
int SerpeMot::nlimit_pin = 0;
bool SerpeMot::stepState = false;
RP2040_PWM* SerpeMot::motor_pwm = nullptr;
//MBED_RPI_PICO_TimerInterrupt timer1 = MBED_RPI_PICO_TimerInterrupt(1);


/*
*SerpeMot Class functions definition
*/

//Constructor
SerpeMot::SerpeMot(int cs, int step, int nstall, void (*stall_fx)(), int nlimit, void (*limitSwitch_fx)())
{
  cs_pin = cs;
  step_pin = step;
  nstall_pin = nstall;
  nlimit_pin = nlimit;
  stall_function = stall_fx;
  limitSwitch_function = limitSwitch_fx;
  instanceCount++;

  SPI.begin();
  driver.setChipSelectPin(cs_pin);
  delay(10); // Give the driver some time to power up.
  driver.resetSettings();
  driver.clearStatus();
  driver.setDecayMode(HPSDDecayMode::AutoMixed); //Recommended mode : AutoMixed
  driver.setCurrentMilliamps36v4(currentLimit_ma);
  driver.setStepMode(stepMode);
  pinMode(nstall_pin, INPUT_PULLUP);
  pinMode(nlimit_pin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(nstall_pin), stall_callback, FALLING);
  //attachInterrupt(digitalPinToInterrupt(nlimit_pin), limitSwitch_callback, FALLING);
  SerpeMot::motor_pwm = new RP2040_PWM(step_pin, 0, 0);
  driver.setDirection(reverseDir);
  driver.enableDriver();
  Serial.begin(2000000);

}

//Function that executes the desired subfunction in case of motor stall
void SerpeMot::stall_callback(void)
{
  (*SerpeMot::stall_function)();
}

void SerpeMot::setRamp(int hertzPerSec)
{
  acceleration = hertzPerSec;
}

void SerpeMot::limitSwitch_callback(void)
{
  //detachInterrupt(digitalPinToInterrupt(nlimit_pin));
  //motor_pwm->setPWM(step_pin, 500, 0);
  (*SerpeMot::limitSwitch_function)();
  //attachInterrupt(nlimit_pin, limitSwitch_callback, FALLING);
}

// void SerpeMot::interruptStep_callback(uint alarm_num)
// {
//   ///////////////////////////////////////////////////////////
//   // Always call this for MBED RP2040 before processing ISR
//   TIMER_ISR_START(alarm_num);
//   ///////////////////////////////////////////////////////////
//   SerpeMot::stepState = !SerpeMot::stepState;
//   digitalWrite(step_pin,stepState);
//   if(stepState) nbSteps++;

//   ////////////////////////////////////////////////////////////
//   // Always call this for MBED RP2040 after processing ISR
//   TIMER_ISR_END(alarm_num);
//   ////////////////////////////////////////////////////////////
// }

//Sets the step frequency of the motor converting positive/negative speed value in step/dir signals
// and using default acceleration profile
void SerpeMot::setFreq(int freq)
{
  if (freq == 0)
  {
    SerpeMot::motor_pwm->setPWM(SerpeMot::step_pin, 500, 0); // Use DC = 0 to stop stepper
  }
  else
  {
    int delta_freq = freq-actualFreq;
    int sign = (delta_freq > 0) - (delta_freq < 0);
    int lastFreq;

    if(delta_freq != 0)
    {
      for(int i = 0; i<abs((int)((delta_freq)/freqStepSize)) ; i ++)
      {
        lastFreq = actualFreq;
        actualFreq += freqStepSize*sign;
        // if(((actualFreq > 0) - (actualFreq < 0))!= ((lastFreq > 0) - (lastFreq < 0))) //if the sign of the next frequency to apply changed....
        // {
        //   driver.setDirection((reverseDir && !(actualFreq > 0)) || ( !reverseDir && (actualFreq > 0))); //either positive speed corresponds to DIR bit == 0 or 1
        // }

        if(actualFreq>0)
        {
          driver.setDirection(0);
        }
        else if(actualFreq<0)
        {
          driver.setDirection(1);
        }

        motor_pwm->setPWM(step_pin, abs(actualFreq), 50);
        delayMicroseconds((int)(1.033*(1000000*freqStepSize)/acceleration));

      }
    }
  }    
}

//Sets the step frequency of the motor converting positive/negative speed value in step/dir signals 
//and using coustom acceleration profile
void SerpeMot::setFreq(int freq, int hertzPerSec)
{
  if (freq == 0)
  {
    motor_pwm->setPWM(step_pin, 500, 0); // Use DC = 0 to stop stepper
  }
  else
  {
    int delta_freq = freq-actualFreq;
    int sign = (delta_freq > 0) - (delta_freq < 0);
    int lastFreq;

    if(delta_freq != 0)
    {
      for(int i = 0; i<abs((int)((delta_freq)/freqStepSize)) ; i ++)
      {
        lastFreq = actualFreq;
        actualFreq += freqStepSize*sign;
        if(((actualFreq > 0) - (actualFreq < 0))!= ((lastFreq > 0) - (lastFreq < 0))) //if the sign of the next frequency to apply changed....
        {
          driver.setDirection((reverseDir && !(actualFreq > 0)) || ( !reverseDir && (actualFreq > 0))); //either positive speed corresponds to DIR bit == 0 or 1
        }
        motor_pwm->setPWM(step_pin, abs(actualFreq), 50);
        delayMicroseconds((int)(1.033*(1000000*freqStepSize)/hertzPerSec));

      }
    }
  } 
}


//Sets the step angular speed of the motor converting positive/negative speed value in step/dir signals 
//and using default acceleration profile
void SerpeMot::setAngularSpeed(int deg_s)
{
  int freq = (deg_s*stepMode)/theta_mot;
  if (freq == 0)
  {
    motor_pwm->setPWM(step_pin, 500, 0); // Use DC = 0 to stop stepper
  }
  else
  {
    int delta_freq = freq-actualFreq;
    int sign = (delta_freq > 0) - (delta_freq < 0);
    int lastFreq;

    if(delta_freq != 0)
    {
      for(int i = 0; i<abs((int)((delta_freq)/freqStepSize)) ; i ++)
      {
        lastFreq = actualFreq;
        actualFreq += freqStepSize*sign;
        if(((actualFreq > 0) - (actualFreq < 0))!= ((lastFreq > 0) - (lastFreq < 0))) //if the sign of the next frequency to apply changed....
        {
          driver.setDirection((reverseDir && !(actualFreq > 0)) || ( !reverseDir && (actualFreq > 0))); //either positive speed corresponds to DIR bit == 0 or 1
        }
        motor_pwm->setPWM(step_pin, abs(actualFreq), 50);
        delayMicroseconds((int)(1.033*(1000000*freqStepSize)/acceleration));

      }
    }
  }

}

//Sets the step angular speed of the motor converting positive/negative speed value in step/dir signals 
//and using coustom acceleration profile
void SerpeMot::setAngularSpeed(int deg_s, int hertzPerSec)
{
  int freq = (deg_s*stepMode)/theta_mot;
  if (freq == 0)
  {
    motor_pwm->setPWM(step_pin, 500, 0); // Use DC = 0 to stop stepper
  }
  else
  {
    int delta_freq = freq-actualFreq;
    int sign = (delta_freq > 0) - (delta_freq < 0);
    int lastFreq;

    if(delta_freq != 0)
    {
      for(int i = 0; i<abs((int)((delta_freq)/freqStepSize)) ; i ++)
      {
        lastFreq = actualFreq;
        // Serial.print(driver.getDirection());
        // Serial.print(" ");
        // Serial.println(actualFreq);
        actualFreq += freqStepSize*sign;
        
        // if(((actualFreq > 0) - (actualFreq < 0))!= ((lastFreq > 0) - (lastFreq < 0))) //if the sign of the next frequency to apply changed....
        // {
        //   driver.setDirection((reverseDir && !(actualFreq > 0)) || ( !reverseDir && (actualFreq > 0))); //either positive speed corresponds to DIR bit == 0 or 1
        // }
        if(actualFreq>=0)
        {
          driver.setDirection(false);
        }
        else if(actualFreq<0)
        {
          driver.setDirection(true);
        }
        motor_pwm->setPWM(step_pin, abs(actualFreq), 50);
        delayMicroseconds((int)(1.033*(1000000*freqStepSize)/(hertzPerSec)));

      }
    }
  }
}


void SerpeMot::setSteps(int steps, int freq)
{
  // int period = (int)((1/freq)*1000000.0f);
  // motor_pwm->setPWM(step_pin,0,0);
  // pinMode(step_pin, OUTPUT);
  // digitalWrite(step_pin,LOW);

  // timer1.attachInterruptInterval(freq, interruptStep_callback);

}

int SerpeMot::getFreq(void)
{
  return SerpeMot::actualFreq;
}



