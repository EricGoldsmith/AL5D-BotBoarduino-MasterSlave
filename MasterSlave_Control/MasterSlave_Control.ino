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
#define BAS_SRV_PIN 2     // Base servo HS-485HB
#define SHL_SRV_PIN 3     // Shoulder Servo HS-805BB
#define ELB_SRV_PIN 4     // Elbow Servo HS-755HB
#define WRI_SRV_PIN 10    // Wrist servo HS-645MG
#define GRI_SRV_PIN 11    // Gripper servo HS-422
#ifdef WRIST_ROTATE
 #define WRO_SRV_PIN 12   // Wrist rotate servo HS-485HB
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

// Pot range limits for +/- 90 degree rotation from midpoints
// Used to scale pot input to servo position
#define BAS_POT_MIN 183
#define BAS_POT_MAX 950

#define SHL_POT_MIN 114
#define SHL_POT_MAX 935

#define ELB_POT_MIN 67
#define ELB_POT_MAX 905

#define WRI_POT_MIN 87
#define WRI_POT_MAX 932

#define GRI_POT_MIN 0
#define GRI_POT_MAX 1023

#ifdef WRIST_ROTATE
 #define WRO_POT_MIN 45
 #define WRO_POT_MAX 990
#endif

// Define generic range limits for servos, in microseconds (us)
// Specific per-servo/joint limits are defined below
#define SERVO_MIN 600
#define SERVO_MID 1500
#define SERVO_MAX 2400

// Servo offsets (in microseconds) for centered position
#define BAS_SRV_OFFSET (0)
#define SHL_SRV_OFFSET (-50)
#define ELB_SRV_OFFSET (-20)
#define WRI_SRV_OFFSET (+30)
#define GRI_SRV_OFFSET (0)
#ifdef WRIST_ROTATE
 #define WRO_SRV_OFFSET (0)
#endif

// Servo range limits
// Used to scale pot input to servo position
#define BAS_SRV_MIN (SERVO_MIN + BAS_SRV_OFFSET)
#define BAS_SRV_MAX (SERVO_MAX + BAS_SRV_OFFSET)

#define SHL_SRV_MIN (SERVO_MIN + SHL_SRV_OFFSET)
#define SHL_SRV_MAX (SERVO_MAX + SHL_SRV_OFFSET)

#define ELB_SRV_MIN (SERVO_MIN + ELB_SRV_OFFSET)
#define ELB_SRV_MAX (SERVO_MAX + ELB_SRV_OFFSET)

#define WRI_SRV_MIN (SERVO_MIN + WRI_SRV_OFFSET)
#define WRI_SRV_MAX (SERVO_MAX + WRI_SRV_OFFSET)

#define GRI_SRV_MIN (SERVO_MIN + GRI_SRV_OFFSET)
#define GRI_SRV_MAX (SERVO_MAX + GRI_SRV_OFFSET)

#ifdef WRIST_ROTATE
 #define WRO_SRV_MIN (SERVO_MIN + WRO_SRV_OFFSET)
 #define WRO_SRV_MAX (SERVO_MAX + WRO_SRV_OFFSET)
#endif


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
    // Note that the scaling limits (e.g. BAS_SRV_MIN, etc.) are not used here
    // The limits here are just the physical limits of the servos, to prevent damage
    Bas_Servo.attach(BAS_SRV_PIN, SERVO_MIN, SERVO_MAX);
    Shl_Servo.attach(SHL_SRV_PIN, SERVO_MIN, SERVO_MAX);
    Elb_Servo.attach(ELB_SRV_PIN, SERVO_MIN, SERVO_MAX);
    Wri_Servo.attach(WRI_SRV_PIN, SERVO_MIN, SERVO_MAX);
    Gri_Servo.attach(GRI_SRV_PIN, SERVO_MIN, SERVO_MAX);
#ifdef WRIST_ROTATE
    Wro_Servo.attach(WRO_SERVO_PIN, SERVO_MIN, SERVO_MAX);
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
    gri = analogRead(GRI_POT_PIN);
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
    bas = map(bas, BAS_POT_MIN, BAS_POT_MAX, BAS_SRV_MIN, BAS_SRV_MAX);
    shl = map(shl, SHL_POT_MIN, SHL_POT_MAX, SHL_SRV_MIN, SHL_SRV_MAX);
    elb = map(elb, ELB_POT_MIN, ELB_POT_MAX, ELB_SRV_MIN, ELB_SRV_MAX);
    wri = map(wri, WRI_POT_MIN, WRI_POT_MAX, WRI_SRV_MIN, WRI_SRV_MAX);
    gri = map(gri, GRI_POT_MIN, GRI_POT_MAX, GRI_SRV_MIN, GRI_SRV_MAX);
#ifdef WRIST_ROTATE
    wro = map(wro, WRO_POT_MIN, WRO_POT_MAX, WRO_SRV_MIN, WRO_SRV_MAX);
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
    Wri_Servo.writeMicroseconds(wri);
    Gri_Servo.writeMicroseconds(gri);
#ifdef WRIST_ROTATE
    Wro_Servo.writeMicroseconds(wro);
#endif

} // end loop()
