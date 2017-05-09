/*
 * data.h
 *
 *  Created on: 2016. dec. 21.
 *      Author: Sanyi
 */

#ifndef DATA_H_
#define DATA_H_

#include "Arduino.h"
#include "EEPROM.h"
#include "ttloco.h"

#define NUM_OF_SERVOS 6
#define NUM_OF_TRACKS 1
#define NUM_OF_SWITCHES 6
#define TRACK_CHANGE_SPEED 0.04 // around 4 secs to speed to max (from 0 to ~200 PWM)
#define PWM_LIMIT_LOW 50 // trains start from this PWM value
#define PWM_LIMIT_HIGH 13.0/16.5*255  // max PWM value at 16.5V (-> 13V, cheating...)
#define MIN_SPEED  3 // -min - +min value on the slider the train is stopped!
#define MAX_SPEED 100 // maximum value on the slider
#define EEPROM_SERVO_OFFSET 4
#define EEPROM_SWITCH_OFFSET EEPROM_SERVO_OFFSET+NUM_OF_SERVOS
#define SENSEPIN_TRACK1 A6
#define SENSEPIN_TRACK2 A7
#define SENSEPIN_LIMIT 200

SwitchData switchData[NUM_OF_SWITCHES] = {
        { 7,  0, 0, &switchData[1] }, // turnout#1 frog
        { 5,  0, 1, NULL },            // turnout#1 led
        { A0, 0, 2, NULL },         // turnout#2 frog
        { A1, 0, 3, NULL },         // turnout#3 frog
        { A2, 0, 4, NULL },         // turnout#4 frog
        { A3, 0, 5, NULL }          // turnout#5 frog

};

Servo servos[NUM_OF_SERVOS];
ServoData servoData[NUM_OF_SERVOS] = {
        (ServoData ) { { 180, 120 }, 0.02, 1, 6, NULL }, // Barrier
        (ServoData ) { { 10, 35 }, 0.03, 0,  8, &switchData[0] }, // Turnout #1, servopin:#8, frogPin:#7
        (ServoData ) { { 10, 35 }, 0.03, 0,  4, &switchData[2] }, // Turnout #2, servopin:#4, frogPin:#A0
        (ServoData ) { { 10, 35 }, 0.03, 0,  3, &switchData[3] }, // Turnout #3, servopin:#3, frogPin:#A1
        (ServoData ) { { 10, 35 }, 0.03, 0,  2, &switchData[4] }, // Turnout #4, servopin:#2, frogPin:#A2
        (ServoData ) { { 10, 35 }, 0.03, 0, 12, &switchData[5] }, // Turnout #5, servopin:#12, frogPin:#A3

};

TrackData trackData[NUM_OF_TRACKS] = { (TrackData ) { 11, 10, 9, 0 } }; // PWM: pin#11, forward: PIN#10, backward: PIN#9, initial speed: 0

volatile int voltage_track1 = 0;
volatile int voltage_track2 = 0;
volatile int max_voltage_track1 = 0;
volatile int max_voltage_track2 = 0;


#endif /* DATA_H_ */
