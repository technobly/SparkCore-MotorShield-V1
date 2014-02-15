SparkCore-MotorShield-V1
========================

An Implementation of Adafruit's Motor Shield V1 Library for the Spark Core

###Currently supports:###
* AF_DCMotor (all motors use the same custom frequency)
* Works best when used with the Shield Shield for level converting, but also works directly hooked to the Spark Core.
* NOTE: Make sure you have the power connections fully understood before trying to use this!

###Not Tested:###
* AF_StepperMotor (might work)
* AF_Servo (no PWM routed to these pins, will require HW modification to shield)

###How To Use:###
* Grab the RAW version of each file and place into your web IDE as follows:
![image](http://i.imgur.com/C5rBeMq.png)