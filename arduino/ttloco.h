/*
 * loco.h
 *
 *  Created on: 2016. dec. 18.
 *      Author: Sanyi
 */

#ifndef TTLOCO_H_
#define TTLOCO_H_

typedef struct {
    byte index;
    float value;
    float change_speed; // changes in 1 ms (the bigger the speed the faster the value changes)
    unsigned long timeStamp;
    bool finished;
} ChangeTask;

typedef struct st_switchData {
    byte pin;
    byte state;
    byte index;
    struct st_switchData* next;
} SwitchData;

typedef struct st_servoData {
    byte limits[2];
    float switching_speed;
    byte servo_state;
    byte servoPin;
//    void (*checkServoState)(*st_servoData sd);
    SwitchData *extraSwitch; // turnout or LED's pin, NULL if no extra switch is controlled,
} ServoData;

typedef struct {
    byte pinPWM;
    byte pinBack;
    byte pinForward;
    signed short speed;
} TrackData;

#endif /* TTLOCO_H_ */
