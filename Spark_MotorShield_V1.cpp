/*
 * ADAFRUIT MOTOR SHIELD V1 LIBRARY FOR SPARK CORE
 * =======================================================
 * Copy this into a new application at:
 * https://www.spark.io/build and go nuts!
 * !! Pinouts on line 15 below !!
 * -------------------------------------------------------
 *  Author: BDub
 * Website: http://technobly.com
 *    Date: Feb 6th 2014
 * =======================================================
 * https://github.com/technobly/SparkCore-MotorShield-V1
 */

/* ========================= PINOUTS ============================== */

/*
 * SPARK CORE   FUNCTION    ARDUINO MOTOR SHIELD
 * A5           PWM1        11
 * D0           PWM2        3
 * A7           PWM3        6
 * D1           PWM4        5
 * A4           MOTORLATCH  12
 * D3           MOTORCLK    4
 * D4           MOTORENABLE 7
 * D5           MOTORDATA   8
 * GND          GROUND      GND
 * VIN(#)       VCC         5V
 * 5V(##)       VCC         5V
 * VIN(##)      V+          VIN
 * (# only connect if powering Core with regulated 5V supply)
 * (## denotes Spark Core Shield Shield only.  If connecting 
 *  directly to a Spark Core, connect an external power 
 *  supply to the EXT PWR connector on the motor shield and 
 *  remove the PWR jumper... or make sure you have a 5V 1A 
 *  power supply connected to the micro USB input or VIN.
 */

/* ======================= Spark_MotorShield_V1.cpp ================ */
  
#include "Spark_MotorShield_V1.h"

// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!
// -------------------------------------------
// Ported to Spark Core by BDub (http://technobly.com)

static uint8_t latch_state;

#if (MICROSTEPS == 8)
  uint8_t microstepcurve[] = {0, 50, 98, 142, 180, 212, 236, 250, 255};
#elif (MICROSTEPS == 16)
  uint8_t microstepcurve[] = {0, 25, 50, 74, 98, 120, 141, 162, 180, 197, 212, 225, 236, 244, 250, 253, 255};
#endif

AFMotorController::AFMotorController(void) {
    _TimerInitialized = false;
}

void AFMotorController::enable(void) {
  // Spark Core pin names to interface with 74HCT595 latch
  pinMode(MOTORLATCH, OUTPUT);  // Spark Core pin A4, Arduino pin 12 (RCK)
  pinMode(MOTORENABLE, OUTPUT); // Spark Core pin D4, Arduino pin 7  (~OE)
  pinMode(MOTORDATA, OUTPUT);   // Spark Core pin D5, Arduino pin 8  (SER)
  pinMode(MOTORCLK, OUTPUT);    // Spark Core pin D3, Arduino pin 4  (SRCLK)

  latch_state = 0;

  latch_tx();  // "reset"

  // enable the chip outputs!
  digitalWrite(MOTORENABLE, LOW); // Spark Core pin D4, Arduino pin 7  (~OE)
}

void AFMotorController::latch_tx(void) {
  uint8_t i;

  digitalWrite(MOTORLATCH, LOW);     // Spark Core pin A4, Arduino pin 12 (RCK)
  digitalWrite(MOTORDATA, LOW);      // Spark Core pin D5, Arduino pin 8  (SER)

  for (i=0; i<8; i++) {
    digitalWrite(MOTORCLK, LOW);     // Spark Core pin D3, Arduino pin 4  (SRCLK)
    if (latch_state & _BV(7-i)) {    // walks down mask from bit 7 to bit 0
      digitalWrite(MOTORDATA, HIGH); // Spark Core pin D5, Arduino pin 8  (SER)
    } else {
      digitalWrite(MOTORDATA, LOW);  // Spark Core pin D5, Arduino pin 8  (SER)
    }
    digitalWrite(MOTORCLK, HIGH);    // Spark Core pin D3, Arduino pin 4  (SRCLK)
  }
  digitalWrite(MOTORLATCH, HIGH);    // Spark Core pin A4, Arduino pin 12 (RCK)
}

static AFMotorController MC;

/******************************************
               MOTORS
******************************************/
inline void initPWM1(uint16_t freq) {
  // use PWM from Arduino pin 11, Spark Core pin A5
  initPWM(A5, freq);
}

inline void setPWM1(uint8_t duty) {
  // use PWM from Arduino pin 11, Spark Core pin A5
  setPWM(A5,duty);
}

inline void initPWM2(uint16_t freq) {
  // use PWM from Arduino pin 3, Spark Core pin D0
  initPWM(D0, freq);
}

inline void setPWM2(uint8_t duty) {
  // use PWM from Arduino pin 3, Spark Core pin D0
  setPWM(D0,duty);
}

inline void initPWM3(uint16_t freq) {
  // use PWM from Arduino pin 6, Spark Core pin A7
  initPWM(A7, freq);
}

inline void setPWM3(uint8_t duty) {
  // use PWM from Arduino pin 6, Spark Core pin A7
  setPWM(A7,duty);
}

inline void initPWM4(uint16_t freq) {
  // use PWM from Arduino pin 5, Spark Core pin D1
  initPWM(D1, freq);
}

inline void setPWM4(uint8_t duty) {
  // use PWM from Arduino pin 5, Spark Core pin D1
  setPWM(D1,duty);
}

AF_DCMotor::AF_DCMotor(uint8_t num, uint16_t freq) {
  _motornum = num;
  _pwmfreq = freq;
}

void AF_DCMotor::init(void) {
#ifdef MOTORDEBUG
  Serial.print("MOTOR:");
  Serial.print(_motornum);
  Serial.print(" FREQ:");
  Serial.println(_pwmfreq);
#endif

  MC.enable();

  switch (_motornum) {
  case 1:
    latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B); // set both motor pins to 0
    MC.latch_tx();
    initPWM1(_pwmfreq);
    break;
  case 2:
    latch_state &= ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // set both motor pins to 0
    MC.latch_tx();
    initPWM2(_pwmfreq);
    break;
  case 3:
    latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B); // set both motor pins to 0
    MC.latch_tx();
    initPWM3(_pwmfreq);
    break;
  case 4:
    latch_state &= ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // set both motor pins to 0
    MC.latch_tx();
    initPWM4(_pwmfreq);
    break;
  }
}

void AF_DCMotor::run(uint8_t cmd) {
  uint8_t a, b;
  switch (_motornum) {
  case 1:
    a = MOTOR1_A; b = MOTOR1_B; break;
  case 2:
    a = MOTOR2_A; b = MOTOR2_B; break;
  case 3:
    a = MOTOR3_A; b = MOTOR3_B; break;
  case 4:
    a = MOTOR4_A; b = MOTOR4_B; break;
  default:
    return;
  }
  
  switch (cmd) {
  case FORWARD:
    latch_state |= _BV(a);
    latch_state &= ~_BV(b); 
    MC.latch_tx();
    break;
  case BACKWARD:
    latch_state &= ~_BV(a);
    latch_state |= _BV(b); 
    MC.latch_tx();
    break;
  case RELEASE:
    latch_state &= ~_BV(a);     // A and B both low
    latch_state &= ~_BV(b); 
    MC.latch_tx();
    break;
  }
}

void AF_DCMotor::setSpeed(uint8_t speed) {
  switch (_motornum) {
  case 1:
    setPWM1(speed); break;
  case 2:
    setPWM2(speed); break;
  case 3:
    setPWM3(speed); break;
  case 4:
    setPWM4(speed); break;
  }
}

/******************************************
               STEPPERS
******************************************/

AF_Stepper::AF_Stepper(uint16_t steps, uint16_t num) {
  MC.enable();

  _revsteps = steps;
  _steppernum = num;
  _currentstep = 0;

  if (_steppernum == 1) {
    latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B) &
      ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // all motor pins to 0
    MC.latch_tx();
    
    // enable both H bridges
    pinMode(A5, OUTPUT);    // Arduino pin 11
    pinMode(D0, OUTPUT);    // Arduino pin 3
    digitalWrite(A5, HIGH); // Arduino pin 11
    digitalWrite(D0, HIGH); // Arduino pin 3

    // use PWM for microstepping support
    initPWM1(STEPPER1_PWM_RATE);
    initPWM2(STEPPER1_PWM_RATE);
    setPWM1(255);
    setPWM2(255);

  } else if (_steppernum == 2) {
    latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B) &
      ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // all motor pins to 0
    MC.latch_tx();

    // enable both H bridges
    pinMode(D1, OUTPUT);    // Arduino pin 5
    pinMode(A7, OUTPUT);    // Arduino pin 6
    digitalWrite(D1, HIGH); // Arduino pin 5
    digitalWrite(A7, HIGH); // Arduino pin 6

    // use PWM for microstepping support
    // use PWM for microstepping support
    initPWM3(STEPPER2_PWM_RATE);
    initPWM4(STEPPER2_PWM_RATE);
    setPWM3(255);
    setPWM4(255);
  }
}

void AF_Stepper::setSpeed(uint16_t rpm) {
  _usperstep = 60000000 / ((uint32_t)_revsteps * (uint32_t)rpm);
  _steppingcounter = 0;
}

void AF_Stepper::release(void) {
  if (_steppernum == 1) {
    latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B) &
      ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // all motor pins to 0
    MC.latch_tx();
  } else if (_steppernum == 2) {
    latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B) &
      ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // all motor pins to 0
    MC.latch_tx();
  }
}

void AF_Stepper::step(uint16_t steps, uint8_t dir,  uint8_t style) {
  uint32_t uspers = _usperstep;
  uint8_t ret = 0;

  if (style == INTERLEAVE) {
    uspers /= 2;
  }
 else if (style == MICROSTEP) {
    uspers /= MICROSTEPS;
    steps *= MICROSTEPS;
#ifdef MOTORDEBUG
    Serial.print("steps = "); Serial.println(steps, DEC);
#endif
  }

  while (steps--) {
    ret = onestep(dir, style);
    delay(uspers/1000); // in ms
    _steppingcounter += (uspers % 1000);
    if (_steppingcounter >= 1000) {
      delay(1);
      _steppingcounter -= 1000;
    }
  }
  if (style == MICROSTEP) {
    while ((ret != 0) && (ret != MICROSTEPS)) {
      ret = onestep(dir, style);
      delay(uspers/1000); // in ms
      _steppingcounter += (uspers % 1000);
      if (_steppingcounter >= 1000) {
        delay(1);
        _steppingcounter -= 1000;
      } 
    }
  }
}

uint8_t AF_Stepper::onestep(uint8_t dir, uint8_t style) {
  uint8_t a, b, c, d;
  uint8_t ocrb, ocra;

  ocra = ocrb = 255;

  if (_steppernum == 1) {
    a = _BV(MOTOR1_A);
    b = _BV(MOTOR2_A);
    c = _BV(MOTOR1_B);
    d = _BV(MOTOR2_B);
  } else if (_steppernum == 2) {
    a = _BV(MOTOR3_A);
    b = _BV(MOTOR4_A);
    c = _BV(MOTOR3_B);
    d = _BV(MOTOR4_B);
  } else {
    return 0;
  }

  // next determine what sort of stepping procedure we're up to
  if (style == SINGLE) {
    if ((_currentstep/(MICROSTEPS/2)) % 2) { // we're at an odd step, weird
      if (dir == FORWARD) {
        _currentstep += MICROSTEPS/2;
      }
      else {
        _currentstep -= MICROSTEPS/2;
      }
    } else {           // go to the next even step
      if (dir == FORWARD) {
        _currentstep += MICROSTEPS;
      }
      else {
        _currentstep -= MICROSTEPS;
      }
    }
  } else if (style == DOUBLE) {
    if (! (_currentstep/(MICROSTEPS/2) % 2)) { // we're at an even step, weird
      if (dir == FORWARD) {
        _currentstep += MICROSTEPS/2;
      } else {
        _currentstep -= MICROSTEPS/2;
      }
    } else {           // go to the next odd step
      if (dir == FORWARD) {
        _currentstep += MICROSTEPS;
      } else {
        _currentstep -= MICROSTEPS;
      }
    }
  } else if (style == INTERLEAVE) {
    if (dir == FORWARD) {
       _currentstep += MICROSTEPS/2;
    } else {
       _currentstep -= MICROSTEPS/2;
    }
  } 

  if (style == MICROSTEP) {
    if (dir == FORWARD) {
      _currentstep++;
    } else {
      // BACKWARDS
      _currentstep--;
    }

    _currentstep += MICROSTEPS*4;
    _currentstep %= MICROSTEPS*4;

    ocra = ocrb = 0;
    if ( (_currentstep >= 0) && (_currentstep < MICROSTEPS)) {
      ocra = microstepcurve[MICROSTEPS - _currentstep];
      ocrb = microstepcurve[_currentstep];
    } else if  ( (_currentstep >= MICROSTEPS) && (_currentstep < MICROSTEPS*2)) {
      ocra = microstepcurve[_currentstep - MICROSTEPS];
      ocrb = microstepcurve[MICROSTEPS*2 - _currentstep];
    } else if  ( (_currentstep >= MICROSTEPS*2) && (_currentstep < MICROSTEPS*3)) {
      ocra = microstepcurve[MICROSTEPS*3 - _currentstep];
      ocrb = microstepcurve[_currentstep - MICROSTEPS*2];
    } else if  ( (_currentstep >= MICROSTEPS*3) && (_currentstep < MICROSTEPS*4)) {
      ocra = microstepcurve[_currentstep - MICROSTEPS*3];
      ocrb = microstepcurve[MICROSTEPS*4 - _currentstep];
    }
  }

  _currentstep += MICROSTEPS*4;
  _currentstep %= MICROSTEPS*4;

#ifdef MOTORDEBUG
  Serial.print("current step: "); Serial.println(_currentstep, DEC);
  Serial.print(" pwmA = "); Serial.print(ocra, DEC); 
  Serial.print(" pwmB = "); Serial.println(ocrb, DEC); 
#endif

  if (_steppernum == 1) {
    setPWM1(ocra);
    setPWM2(ocrb);
  } else if (_steppernum == 2) {
    setPWM3(ocra);
    setPWM4(ocrb);
  }

  // release all
  latch_state &= ~a & ~b & ~c & ~d; // all motor pins to 0

  //Serial.println(step, DEC);
  if (style == MICROSTEP) {
    if ((_currentstep >= 0) && (_currentstep < MICROSTEPS))
      latch_state |= a | b;
    if ((_currentstep >= MICROSTEPS) && (_currentstep < MICROSTEPS*2))
      latch_state |= b | c;
    if ((_currentstep >= MICROSTEPS*2) && (_currentstep < MICROSTEPS*3))
      latch_state |= c | d;
    if ((_currentstep >= MICROSTEPS*3) && (_currentstep < MICROSTEPS*4))
      latch_state |= d | a;
  } else {
    switch (_currentstep/(MICROSTEPS/2)) {
    case 0:
      latch_state |= a; // energize coil 1 only
      break;
    case 1:
      latch_state |= a | b; // energize coil 1+2
      break;
    case 2:
      latch_state |= b; // energize coil 2 only
      break;
    case 3:
      latch_state |= b | c; // energize coil 2+3
      break;
    case 4:
      latch_state |= c; // energize coil 3 only
      break; 
    case 5:
      latch_state |= c | d; // energize coil 3+4
      break;
    case 6:
      latch_state |= d; // energize coil 4 only
      break;
    case 7:
      latch_state |= d | a; // energize coil 1+4
      break;
    }
  }

  MC.latch_tx();
  return _currentstep;
}

/* ======================= PWM.cpp ==================================== */

// User defined analogWrite() to gain control of PWM frequency
void initPWM(uint16_t pin, uint16_t pwm_freq) {

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  
  //PWM Frequency : PWM_FREQ (Hz)
  uint16_t TIM_Prescaler = (uint16_t)(SystemCoreClock / 24000000) - 1; //TIM Counter clock = 24MHz

  // TIM_ARR Calculates period
  //uint16_t TIM_ARR = (uint16_t)(24000000 / pwm_freq) - 1;
  TIM_ARR = (uint16_t)(24000000 / pwm_freq) - 1;
  
  // TIM Channel Duty Cycle(%) = (TIM_CCR / TIM_ARR + 1) * 100
  //uint16_t TIM_CCR = (uint16_t)(value * (TIM_ARR + 1) / 255);
  uint16_t TIM_CCR = 0; // init duty cycle to zero, might want to change this to use private "value"

  // AFIO clock enable
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  pinMode(pin, AF_OUTPUT_PUSHPULL);

  // TIM clock enable
  if (PIN_MAP[pin].timer_peripheral == TIM2)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  else if (PIN_MAP[pin].timer_peripheral == TIM3)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  else if (PIN_MAP[pin].timer_peripheral == TIM4)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  // Time base configuration
  TIM_TimeBaseStructure.TIM_Period = TIM_ARR;
  TIM_TimeBaseStructure.TIM_Prescaler = TIM_Prescaler;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(PIN_MAP[pin].timer_peripheral, & TIM_TimeBaseStructure);

  // PWM1 Mode configuration
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_Pulse = TIM_CCR;

  // PIN pin channel       PWM
  //---------------------------
  // D1  1   TIM_Channel_1 PWM4
  // A5  15  TIM_Channel_2 PWM1
  // D0  0   TIM_Channel_2 PWM2
  // A7  17  TIM_Channel_4 PWM3

  if (PIN_MAP[pin].timer_ch == TIM_Channel_1) {
    // PWM1 Mode configuration: Channel1
    TIM1_ARR = TIM_ARR;
    TIM_OC1Init(PIN_MAP[pin].timer_peripheral, & TIM_OCInitStructure);
    TIM_OC1PreloadConfig(PIN_MAP[pin].timer_peripheral, TIM_OCPreload_Enable);
  } else if (PIN_MAP[pin].timer_ch == TIM_Channel_2) {
    // PWM1 Mode configuration: Channel2
    TIM2_ARR = TIM_ARR;
    TIM_OC2Init(PIN_MAP[pin].timer_peripheral, & TIM_OCInitStructure);
    TIM_OC2PreloadConfig(PIN_MAP[pin].timer_peripheral, TIM_OCPreload_Enable);
  } else if (PIN_MAP[pin].timer_ch == TIM_Channel_3) {
    // PWM1 Mode configuration: Channel3
    TIM3_ARR = TIM_ARR;
    TIM_OC3Init(PIN_MAP[pin].timer_peripheral, & TIM_OCInitStructure);
    TIM_OC3PreloadConfig(PIN_MAP[pin].timer_peripheral, TIM_OCPreload_Enable);
  } else if (PIN_MAP[pin].timer_ch == TIM_Channel_4) {
    // PWM1 Mode configuration: Channel4
    TIM4_ARR = TIM_ARR;
    TIM_OC4Init(PIN_MAP[pin].timer_peripheral, & TIM_OCInitStructure);
    TIM_OC4PreloadConfig(PIN_MAP[pin].timer_peripheral, TIM_OCPreload_Enable);
  }

  TIM_ARRPreloadConfig(PIN_MAP[pin].timer_peripheral, ENABLE);

  // TIM enable counter
  TIM_Cmd(PIN_MAP[pin].timer_peripheral, ENABLE);
}

// User defined analogWrite() to gain control of PWM duty cycle update
void setPWM(uint16_t pin, uint8_t value) {
  if (pin >= TOTAL_PINS || PIN_MAP[pin].timer_peripheral == NULL) {
    return;
  }
  // SPI safety check
  if (SPI.isEnabled() == true && (pin == SCK || pin == MOSI || pin == MISO)) {
    return;
  }
  // I2C safety check
  if (Wire.isEnabled() == true && (pin == SCL || pin == SDA)) {
    return;
  }
  // Serial1 safety check
  if (Serial1.isEnabled() == true && (pin == RX || pin == TX)) {
    return;
  }
  if (PIN_MAP[pin].pin_mode != OUTPUT && PIN_MAP[pin].pin_mode != AF_OUTPUT_PUSHPULL) {
    return;
  }
  // Don't re-init PWM and cause a glitch if already setup, just update duty cycle and return.
  if (PIN_MAP[pin].pin_mode == AF_OUTPUT_PUSHPULL) {
    if (PIN_MAP[pin].timer_ch == TIM_Channel_1) {
      //Serial.print(TIM1_ARR); Serial.println(" TIM_Channel_1");
      //PIN_MAP[pin].timer_peripheral-> ARR = TIM1_ARR;
      //TIM_OCInitStructure.TIM_Pulse = (uint16_t)(value * (TIM1_ARR + 1) / 255);
      //PIN_MAP[pin].timer_peripheral-> CCR1 = TIM_OCInitStructure.TIM_Pulse;
      PIN_MAP[pin].timer_peripheral-> CCR1 = (uint16_t)(value * (TIM1_ARR + 1) / 255);
    } else if (PIN_MAP[pin].timer_ch == TIM_Channel_2) {
      //Serial.print(TIM2_ARR); Serial.println(" TIM_Channel_2");
      //PIN_MAP[pin].timer_peripheral-> ARR = TIM2_ARR;
      //TIM_OCInitStructure.TIM_Pulse = (uint16_t)(value * (TIM2_ARR + 1) / 255);
      //PIN_MAP[pin].timer_peripheral-> CCR2 = TIM_OCInitStructure.TIM_Pulse;
      PIN_MAP[pin].timer_peripheral-> CCR2 = (uint16_t)(value * (TIM2_ARR + 1) / 255);
    } else if (PIN_MAP[pin].timer_ch == TIM_Channel_3) {
      //Serial.print(TIM3_ARR); Serial.println(" TIM_Channel_3");
      //PIN_MAP[pin].timer_peripheral-> ARR = TIM3_ARR;
      //TIM_OCInitStructure.TIM_Pulse = (uint16_t)(value * (TIM3_ARR + 1) / 255);
      //PIN_MAP[pin].timer_peripheral-> CCR3 = TIM_OCInitStructure.TIM_Pulse;
      PIN_MAP[pin].timer_peripheral-> CCR3 = (uint16_t)(value * (TIM3_ARR + 1) / 255);
    } else if (PIN_MAP[pin].timer_ch == TIM_Channel_4) {
      //Serial.print(TIM4_ARR); Serial.println(" TIM_Channel_4");
      //PIN_MAP[pin].timer_peripheral-> ARR = TIM4_ARR;
      //TIM_OCInitStructure.TIM_Pulse = (uint16_t)(value * (TIM4_ARR + 1) / 255);
      //PIN_MAP[pin].timer_peripheral-> CCR4 = TIM_OCInitStructure.TIM_Pulse;
      PIN_MAP[pin].timer_peripheral-> CCR4 = (uint16_t)(value * (TIM4_ARR + 1) / 255);
    }
    return;
  }
}