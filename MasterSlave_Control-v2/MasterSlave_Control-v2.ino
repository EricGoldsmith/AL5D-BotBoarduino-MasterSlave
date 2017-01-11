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

int dummy;                // Defining this dummy variable to work around a bug in the
                          // IDE (1.0.3) pre-processor that messes up #ifdefs
                          // More info: http://code.google.com/p/arduino/issues/detail?id=906
                          //            http://code.google.com/p/arduino/issues/detail?id=987
                          //            http://arduino.cc/forum/index.php/topic,125769.0.html


//#define DEBUG             // Uncomment to turn on debugging output

#define WRO_SWITCH        // The wrist rotation input is a toggle switch, not a potentiometer
                          // Requires special handling to prevent servo whiplash
                            
// Arduino digital pin numbers for servo connections
#define BAS_SRV_PIN 2     // Base servo HS-485HB
#define SHL_SRV_PIN 3     // Shoulder Servo HS-805BB
#define ELB_SRV_PIN 4     // Elbow Servo HS-755HB
#define WRI_SRV_PIN 10    // Wrist servo HS-645MG
#define GRI_SRV_PIN 11    // Gripper servo HS-422
#define WRO_SRV_PIN 12    // Wrist rotate servo HS-225MG

// Arduino analog pin numbers for potentiometer connections
#define BAS_POT_PIN 0
#define SHL_POT_PIN 1
#define ELB_POT_PIN 2
#define WRI_POT_PIN 3
#define GRI_POT_PIN 4
#define WRO_POT_PIN 5

// Arduino pin number of on-board speaker
#define SPK_PIN 5

// Pot range limits for +/- 90 degree rotation from midpoints
// Used to scale pot input to servo position
#define BAS_POT_MIN 82
#define BAS_POT_MAX 920

#define SHL_POT_MIN 121
#define SHL_POT_MAX 959

#define ELB_POT_MIN 94
#define ELB_POT_MAX 920

#define WRI_POT_MIN 76
#define WRI_POT_MAX 911

#define GRI_POT_MIN 0
#define GRI_POT_MAX 1023

#define WRO_POT_MIN 0
#define WRO_POT_MAX 1023

// Define generic range limits for servos, in microseconds (us)
// Specific per-servo/joint limits are defined below
#define SERVO_MIN 600
#define SERVO_MID 1500
#define SERVO_MAX 2400

// Servo offsets (in microseconds) for centered position
#define BAS_SRV_OFFSET (0)
#define SHL_SRV_OFFSET (-40)
#define ELB_SRV_OFFSET (-80)
#define WRI_SRV_OFFSET (+30)
#define GRI_SRV_OFFSET (0)
#define WRO_SRV_OFFSET (0)

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

#define WRO_SRV_MIN (SERVO_MIN + WRO_SRV_OFFSET)
#define WRO_SRV_MAX (SERVO_MAX + WRO_SRV_OFFSET)


// Audible feedback sounds
#define TONE_READY 1000       // Hz
#define TONE_RANGE_ERROR 200  // Hz
#define TONE_DURATION 100     // ms
 
// Servo objects 
Servo   Bas_Servo;
Servo   Shl_Servo;
Servo   Elb_Servo;
Servo   Wri_Servo;
Servo   Gri_Servo;
Servo   Wro_Servo;

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
    Wro_Servo.attach(WRO_SER_PIN, SERVO_MIN, SERVO_MAX);

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
    int bas_pot, shl_pot, elb_pot, wri_pot, gri_pot, wro_pot;
    int bas_srv, shl_srv, elb_srv, wri_srv, gri_srv, wro_srv;

    // Read potentiometer positions
    bas_pot = analogRead(BAS_POT_PIN);
    shl_pot = analogRead(SHL_POT_PIN);
    elb_pot = analogRead(ELB_POT_PIN);
    wri_pot = analogRead(WRI_POT_PIN);
    gri_pot = analogRead(GRI_POT_PIN);
    wro_pot = analogRead(WRO_POT_PIN);

#ifdef DEBUG
    Serial.print("Bas Pot: ");
    Serial.print(bas_pot);
    Serial.print("  Shl Pot: ");
    Serial.print(shl_pot);
    Serial.print("  Elb Pot: ");
    Serial.print(elb_pot);
    Serial.print("  Wri Pot: ");
    Serial.print(wri_pot);
    Serial.print("  Gri Pot: ");
    Serial.print(gri_pot);
    Serial.print("  Wro Pot: ");
    Serial.print(wro_pot);
    Serial.println();
 #endif

    // Scale to servo output
    bas_srv = map(bas_pot, BAS_POT_MIN, BAS_POT_MAX, BAS_SRV_MIN, BAS_SRV_MAX);
    shl_srv = map(shl_pot, SHL_POT_MIN, SHL_POT_MAX, SHL_SRV_MIN, SHL_SRV_MAX);
    elb_srv = map(elb_pot, ELB_POT_MIN, ELB_POT_MAX, ELB_SRV_MIN, ELB_SRV_MAX);
    wri_srv = map(wri_pot, WRI_POT_MIN, WRI_POT_MAX, WRI_SRV_MIN, WRI_SRV_MAX);
    gri_srv = map(gri_pot, GRI_POT_MIN, GRI_POT_MAX, GRI_SRV_MIN, GRI_SRV_MAX);
    wro_srv = map(wro_pot, WRO_POT_MIN, WRO_POT_MAX, WRO_SRV_MIN, WRO_SRV_MAX);

#ifdef DEBUG
    Serial.print("Bas Servo: ");
    Serial.print(bas_srv);
    Serial.print("  Shl Servo: ");
    Serial.print(shl_srv);
    Serial.print("  Elb Servo: ");
    Serial.print(elb_srv);
    Serial.print("  Wri Servo: ");
    Serial.print(wri_srv);
    Serial.print("  Gri Servo: ");
    Serial.print(gri_srv);
    Serial.print("  Wro Servo: ");
    Serial.print(wro_srv);
    Serial.println();
#endif

    // Write output to servos
    Bas_Servo.writeMicroseconds(bas_srv);
    Shl_Servo.writeMicroseconds(shl_srv);
    Elb_Servo.writeMicroseconds(elb_srv);
    Wri_Servo.writeMicroseconds(wri_srv);
    Gri_Servo.writeMicroseconds(gri_srv);
#ifdef WRO_SWITCH
	// If using a switch to control the wrist rotation, gradually step
	// from old to new position, to avoid servo whiplash
	
	// Max step, in milliseconds (ms), per iteration
	max_step = 50;

	// Get current position
	wro_srv_last = Wro_Servo.readMicroseconds();
	
	delta = wro_srv - wro_srv_last;
	
	while (delta != 0) {
		// If we need to rotate, determine increment and direction
		if (abs(delta) >= max_step) {
			step = delta > 0 ? max_step : -max_step;
		} else {
			step = delta;
		}
	
		// Move into position
		wro_srv_last += step;
		Wro_Servo.writeMicroseconds(wro_srv_last);
		
		delta = wro_srv - wro_srv_last;
	} 
#else
    Wro_Servo.writeMicroseconds(wro_srv);
#endif

} // end loop()
