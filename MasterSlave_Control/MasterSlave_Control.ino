/******************************************************************
 *   Code to control a (modified) 
 *   LynxMotion AL5D robot arm using a master arm.
 *
 *   Requires Arduino Servo library:
 *       arduino.cc/en/Reference/Servo
 *
 *   Eric Goldsmith
 *   www.ericgoldsmith.com
 *
 *   Current Version:
 *       https://github.com/EricGoldsmith/AL5D-BotBoarduino-MasterSlave
 *   Version history
 *       0.1 Initial version
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * <http://www.gnu.org/licenses/>
 * 
 ******************************************************************/

#include <Servo.h>

int dummy;                  // Defining this dummy variable to work around a bug in the
                            // IDE (1.0.3) pre-processor that messes up #ifdefs
                            // More info: http://code.google.com/p/arduino/issues/detail?id=906
                            //            http://code.google.com/p/arduino/issues/detail?id=987
                            //            http://arduino.cc/forum/index.php/topic,125769.0.html

//#define DEBUG             // Uncomment to turn on debugging output
                            
//#define WRIST_ROTATE      // Uncomment if wrist rotate hardware is installed

// Arduino digital pin numbers for servo connections
#define BAS_SERVO_PIN 2     // Base servo HS-485HB
#define SHL_SERVO_PIN 3     // Shoulder Servo HS-805BB
#define ELB_SERVO_PIN 4     // Elbow Servo HS-755HB
#define WRI_SERVO_PIN 10    // Wrist servo HS-645MG
#define GRI_SERVO_PIN 11    // Gripper servo HS-422
#ifdef WRIST_ROTATE
 #define WRO_SERVO_PIN 12   // Wrist rotate servo HS-485HB
#endif

// Arduino analog pin numbers for potentiometer connections
#define BAS_POT_PIN 0
#define SHL_POT_PIN 1
#define ELB_POT_PIN 2
#define WRI_POT_PIN 3
#define GRI_POT_PIN 4
#ifdef WRIST_ROTATE
 #define WRO_POT_PIN 5
#endif

// Arduino pin number of on-board speaker
#define SPK_PIN 5

// Define range limits for analog input
#define ANALOG_MIN 0
#define ANALOG_MAX 1023

// Define generic range limits for servos, in microseconds (us) and degrees (deg)
// Used to map range of 180 deg to 1800 us (native servo units).
// Specific per-servo/joint limits are defined below
#define SERVO_MIN_US 600
#define SERVO_MID_US 1500
#define SERVO_MAX_US 2400
#define SERVO_MIN_DEG 0.0
#define SERVO_MID_DEG 90.0
#define SERVO_MAX_DEG 180.0

// Set physical limits (in degrees) per servo/joint.
// Will vary for each servo/joint, depending on mechanical range of motion.
// The MID setting is the required servo input needed to achieve a 
// 90 degree joint angle, to allow compensation for horn misalignment
#define BAS_MIN 0.0         // Fully CCW
#define BAS_MID 90.0
#define BAS_MAX 180.0       // Fully CW

#define SHL_MIN 20.0        // Max forward motion
#define SHL_MID 81.0
#define SHL_MAX 140.0       // Max rearward motion

#define ELB_MIN 20.0        // Max upward motion
#define ELB_MID 88.0
#define ELB_MAX 165.0       // Max downward motion

#define WRI_MIN 0.0         // Max downward motion
#define WRI_MID 93.0
#define WRI_MAX 180.0       // Max upward motion

#define GRI_MIN 25.0        // Fully open
#define GRI_MID 90.0
#define GRI_MAX 165.0       // Fully closed

#ifdef WRIST_ROTATE
 #define WRO_MIN 0.0
 #define WRO_MID 90.0
 #define WRO_MAX 180.0
#endif

// Practical navigation limit.
// Enforced on controller input, and used for CLV calculation 
// for base rotation in 2D mode. 
#define Y_MIN 100.0         // mm

// Audible feedback sounds
#define TONE_READY 1000     // Hz
#define TONE_RANGE_ERROR 200   // Hz
#define TONE_DURATION 100   // ms
 
// Servo objects 
Servo   Bas_Servo;
Servo   Shl_Servo;
Servo   Elb_Servo;
Servo   Wri_Servo;
Servo   Gri_Servo;
#ifdef WRIST_ROTATE
 Servo   Wro_Servo;
#endif

/*
 * Setup function - runs once when Arduino is powered up or reset
 */
void setup()
{
#ifdef DEBUG
    Serial.begin(115200);
#endif

    // Attach to the servos and specify range limits
    Bas_Servo.attach(BAS_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
    Shl_Servo.attach(SHL_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
    Elb_Servo.attach(ELB_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
    Wri_Servo.attach(WRI_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
    Gri_Servo.attach(GRI_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
#ifdef WRIST_ROTATE
    Wro_Servo.attach(WRO_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
#endif

#ifdef DEBUG
    Serial.println("Start");
#endif

    delay(500);
    // Sound tone to indicate it's safe to turn on servo power
    tone(SPK_PIN, TONE_READY, TONE_DURATION);
    delay(TONE_DURATION * 2);
    tone(SPK_PIN, TONE_READY, TONE_DURATION);

} // end setup()

/*
 * Loop function - runs forever, after setup() function
 */
void loop()
{
    int bas, shl, elb, wri, gri;
#ifdef WRIST_ROTATE
    int wro;
#endif

    // Read potentiometer positions
    bas = analogRead(BAS_POT_PIN);
    shl = analogRead(SHL_POT_PIN);
    elb = analogRead(ELB_POT_PIN);
    wri = analogRead(WRI_POT_PIN);
    gri = analogRead(gri_POT_PIN);
#ifdef WRIST_ROTATE
    wro = analogRead(WRO_POT_PIN);
#endif

#ifdef DEBUG
    Serial.print("Bas Pot: ");
    Serial.print(bas);
    Serial.print("  Shl Pot: ");
    Serial.print(shl);
    Serial.print("  Elb Pot: ");
    Serial.print(elb);
    Serial.print("  Wri Pot: ");
    Serial.print(wri);
    Serial.print("  Gri Pot: ");
    Serial.print(gri);
    Serial.println();
 #endif

    // Scale to servo output
    bas = map(bas, ANALOG_MIN, ANALOG_MAX, SERVO_MIN_US, SERVO_MAX_US);
    shl = map(shl, ANALOG_MIN, ANALOG_MAX, SERVO_MIN_US, SERVO_MAX_US);
    elb = map(elb, ANALOG_MIN, ANALOG_MAX, SERVO_MIN_US, SERVO_MAX_US);
    wri = map(wri, ANALOG_MIN, ANALOG_MAX, SERVO_MIN_US, SERVO_MAX_US);
    gri = map(gri, ANALOG_MIN, ANALOG_MAX, SERVO_MIN_US, SERVO_MAX_US);
#ifdef WRIST_ROTATE
    wro = map(wro, ANALOG_MIN, ANALOG_MAX, SERVO_MIN_US, SERVO_MAX_US);
#endif

#ifdef DEBUG
    Serial.print("Bas Servo: ");
    Serial.print(bas);
    Serial.print("  Shl Servo: ");
    Serial.print(shl);
    Serial.print("  Elb Servo: ");
    Serial.print(elb);
    Serial.print("  Wri Servo: ");
    Serial.print(wri);
    Serial.print("  Gri Servo: ");
    Serial.print(gri);
    Serial.println();
#endif

    // Write output to servos
    Bas_Servo.writeMicroseconds(bas);
    Shl_Servo.writeMicroseconds(shl);
    Elb_Servo.writeMicroseconds(elb);
    wri_Servo.writeMicroseconds(wri);
    Gri_Servo.writeMicroseconds(gri);
#ifdef WRIST_ROTATE
    Wro_Servo.writeMicroseconds(wro);
#endif

    // Give the servos time to move
    delay(15);
 } // end loop()
 