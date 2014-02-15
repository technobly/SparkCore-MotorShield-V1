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
 
/* ======================= Spark-AF_DCMotor.cpp ======================= */

/* ========================= INCLUDES ================================= */

#include <application.h>

/* ========================= PROTOTYPE DEFS =========================== */

void initPWM(uint16_t pin, uint16_t pwm_freq);
void setPWM(uint16_t pin, uint8_t value);

/* ========================= APPLICATION.cpp ========================== */

// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!
// -------------------------------------------
// Ported to Spark Core by BDub (http://technobly.com)

bool direction = true; // default to fwd, first time through loop

// Setup your motors AF_DCMotor motorx(Motor Output Number, PWM Frequency)
// Motor 1, 2, 3, 4 all have to be the same custom frequency (for now),
// but you can set them to pretty much anything between 500 and 64000

AF_DCMotor motor1(1, MOTOR1234_8KHZ);
AF_DCMotor motor2(2, MOTOR1234_8KHZ);
AF_DCMotor motor3(3, MOTOR1234_8KHZ);
AF_DCMotor motor4(4, MOTOR1234_8KHZ);

void setup() {
  pinMode(D7,OUTPUT);
  digitalWrite(D7,HIGH);
  #ifdef MOTORDEBUG    
    Serial.begin(115200);           // Close terminal, start spark core...
    while(!Serial.available());     // open terminal, and press ENTER
    Serial.println("Motor test!");
  #endif
  motor1.init();
  motor2.init();
  motor3.init();
  motor4.init();
  
  digitalWrite(D7,LOW);
}

void loop() {
  uint8_t i;
  
  if(direction) {
    #ifdef MOTORDEBUG 
      Serial.print(" FWD ");
    #endif
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
  }
  else {
    #ifdef MOTORDEBUG 
      Serial.print(" RVS ");
    #endif
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
  }
  for (i=0; i<255; i++) {
    motor1.setSpeed(i);
    motor2.setSpeed(i);
    motor3.setSpeed(i);
    motor4.setSpeed(i);
    delay(5);
  }
  for (i=255; i!=0; i--) {
    motor1.setSpeed(i);
    motor2.setSpeed(i);
    motor3.setSpeed(i);
    motor4.setSpeed(i);
    delay(5);
  }

  //switch direction every time through loop
  direction = !direction;
}