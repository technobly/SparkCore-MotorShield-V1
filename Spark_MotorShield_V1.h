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

/* ======================= Spark_MotorShield_V1.h ================== */

#ifndef SPARK_MOTORSHIELD_V1_H
#define SPARK_MOTORSHIELD_V1_H

/* ========================= AFMotor.h ============================ */

// Adafruit Motor v1 shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!
// -------------------------------------------
// Ported to Spark Core by BDub (http://technobly.com)

// Uncomment the follwing line for Serial Debug info
//#define MOTORDEBUG 1

#define MICROSTEPS 16 // set to 8 or 16

// Motor Frequencies - These are just some common values,
// techically you can specify any number from 500 to 64000.
#define MOTOR1234_64KHZ 64000
#define MOTOR1234_20KHZ 20000
#define MOTOR1234_15KHZ 15000
#define MOTOR1234_8KHZ 8000
#define MOTOR1234_2KHZ 2000
#define MOTOR1234_1KHZ 1000
#define MOTOR1234_500HZ 500
    
#define DC_MOTOR_PWM_RATE   500   // Default PWM rate for DC motors

#define STEPPER1_PWM_RATE   MOTOR1234_64KHZ   // PWM rate for stepper 1
#define STEPPER2_PWM_RATE   MOTOR1234_64KHZ   // PWM rate for stepper 2

// Bit positions in the 74HCT595 shift register output
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR4_A 0 // should be 5 per schematic, but PCB silk M3 and M4 swapped
#define MOTOR4_B 6 // should be 7 per schematic, but PCB silk M3 and M4 swapped
#define MOTOR3_A 5 // should be 0 per schematic, but PCB silk M3 and M4 swapped
#define MOTOR3_B 7 // should be 6 per schematic, but PCB silk M3 and M4 swapped
// if PCB silk is ever fixed, also need to swap initPWM3/4 and setPWM3/4.

// Constants that the user passes in to the motor calls
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

// Constants that the user passes in to the stepper calls
#define SINGLE 1
#define DOUBLE 2
#define INTERLEAVE 3
#define MICROSTEP 4

// Spark Core pin names to interface with 74HCT595 latch
#define MOTORLATCH A4  // Arduino pin 12 (RCK)
#define MOTORCLK D3    // Arduino pin 4  (SRCLK)
#define MOTORENABLE D4 // Arduino pin 7  (~OE)
#define MOTORDATA D5   // Arduino pin 8  (SER)

class AFMotorController
{
  public:
    AFMotorController(void);
    void enable(void);
    friend class AF_DCMotor;
    void latch_tx(void);
    uint8_t _TimerInitialized;
};

class AF_DCMotor
{
 public:
  AF_DCMotor(uint8_t motornum, uint16_t freq = DC_MOTOR_PWM_RATE);
  void init(void);
  void run(uint8_t);
  void setSpeed(uint8_t);

 private:
  uint8_t _motornum;
  uint16_t _pwmfreq;
};

class AF_Stepper {
 public:
  AF_Stepper(uint16_t, uint16_t);
  void step(uint16_t steps, uint8_t dir,  uint8_t style = SINGLE);
  void setSpeed(uint16_t);
  uint8_t onestep(uint8_t dir, uint8_t style);
  void release(void);
  uint16_t _revsteps; // # steps per revolution
  uint8_t _steppernum;
  uint32_t _usperstep, _steppingcounter;
 private:
  uint8_t _currentstep;

};

uint8_t getlatchstate(void);

#endif // SPARK_MOTORSHIELD_V1_H