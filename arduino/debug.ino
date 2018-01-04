/*
 * debug.c
 *
 *  Created on: 2016. dec. 21.
 *      Author: Sanyi
 */

#include <Arduino.h>
#include "debug.h"
#include "data.h"

void debugTrackSpeedRequestArrived(int index, int value) {
#ifdef DBG
    Serial.print(F("DBG:Track speed request arrived. Track:"));
    Serial.print(index);
    Serial.print(F(", value: "));
    Serial.println(value);
#endif
}

void debugSwitchChangeRequestArrived(int index) {
#ifdef DBG
    Serial.print(F("DBG:Switch change request arrived. Switch:"));
    Serial.println(index);
#endif
}

void debugServoChangeRequestArrived(int index) {
#ifdef DBG
    Serial.print(F("DBG:Servo change request arrived. Servo:"));
    Serial.println(index);
#endif
}

void debugParseDone() {
#ifdef DBG
    Serial.println(F("DBG: Parse done."));
#endif
}

void logSwitchChanged(SwitchData *swD) {
#ifdef DBG
    Serial.print(F("LOG: Switch on pin #"));
    Serial.print(swD->pin);
    Serial.print(F(" changed to "));
    Serial.println(swD->state);
#endif
}

void logSwitchNotChanged(SwitchData *swD) {
#ifdef DBG
    Serial.print(F("LOG: Switch on pin #"));
    Serial.print(swD->pin);
    Serial.print(F(" *NOT* changed. Its current state is: "));
    Serial.println(swD->state);
#endif
}

void debugSettingPWMTo(byte track, short speed) {
#ifdef DBG
    Serial.print(F("DBG: setting speed PWM on track# "));
    Serial.print(track);
    Serial.print(F(" to "));
    Serial.println(speed);
#endif
}

void logServoChangeFinished(byte index, byte angle) {
#ifdef DBG
        Serial.print(F("LOG:Servo #"));
        Serial.print(index);
        Serial.print(F(" reached its destination angle: "));
        Serial.println(angle);
#endif
}

void logTrackChangeFinished(byte index, short value) {
#ifdef DBG
    Serial.print(F("LOG:Track #"));
    Serial.print(index);
    Serial.print(F(" reached its speed: "));
    Serial.println(value);
#endif
}

void logServosInitialized() {
#ifdef DBG
    Serial.println(F("DBG:Servos initialized."));
#endif
}

void logSwitchesInitialized() {
#ifdef DBG
    Serial.println(F("DBG:Switches initialized."));
#endif
}

void logTrackInitialized() {
#ifdef DBG
    Serial.println(F("DBG: Track/speed initialized."));
#endif
}

void printTrack() {
    Serial.println(F("- Track"));
    Serial.print(F("  number of tracks: "));
    Serial.println(NUM_OF_TRACKS);
    for (int i = 0; i < NUM_OF_TRACKS; i++) {
        Serial.print(F("  Track#: "));
        Serial.println(i);
        TrackData *td = &trackData[i];
        Serial.print(F("    PWM pin#: "));
        Serial.println(td->pinPWM);
        Serial.print(F("    forward pin#: "));
        Serial.println(td->pinForward);
        Serial.print(F("    backward pin#: "));
        Serial.println(td->pinBack);
        Serial.print(F("    current speed: "));
        Serial.println(td->speed);
    }
}

void printServo() {
    Serial.println(F("- Servo"));
    Serial.print(F("  number of servos: "));
    Serial.println(NUM_OF_SERVOS);
    for (int i = 0; i < NUM_OF_SERVOS; i++) {
        Serial.print(F("  Servo#: "));
        Serial.println(i);
        ServoData *sd = &servoData[i];
        Serial.print(F("    servo pin#: "));
        Serial.println(sd->servoPin);
        Serial.print(F("    limits: {"));
        Serial.print(sd->limits[0]);
        Serial.print(',');
        Serial.print(sd->limits[1]);
        Serial.println('}');
        Serial.print(F("    extra pin#:"));
        SwitchData *swData = sd->extraSwitch;
        while (swData != NULL) {
            Serial.print(' ');
            Serial.print(swData->pin);
            swData = swData->next;
        }
        Serial.println();
        Serial.print(F("    switching speed: "));
        Serial.println(sd->switching_speed);
        Serial.print(F("    current state: "));
        Serial.println(sd->servo_state);
    }
}

void printSwitch() {
    Serial.println(F("- Switch"));
    Serial.print(F("  number of switches: "));
    Serial.println(NUM_OF_SWITCHES);
    for (int i = 0; i < NUM_OF_SWITCHES; i++) {
        Serial.print(F("  Switch#: "));
        Serial.println(i);
        SwitchData *sd = &switchData[i];
        Serial.print(F("    switch pin#: "));
        Serial.println(sd->pin);
        Serial.print(F("    current state: "));
        Serial.println(sd->state);
    }
}

void printInitScreen() {
    Serial.print(F("tr:<#track> <speed:[-"));
    Serial.print(MAX_SPEED);
    Serial.print(':');
    Serial.print(MAX_SPEED);
    Serial.println(F("]>LF  - Setting speed of a track"));
    Serial.println(F("sw:<#switch>LF  - Change state of a switch"));
    Serial.println(F("se:<#servo>LF  - Switch endposition of a servo"));
    Serial.println(
            F("li:LF - lists all configured tracks, switches and servos"));
    Serial.println(
            F("                   Have fun!           (sanyi - 2016-12-18 - 2018-01-04)"));
}
