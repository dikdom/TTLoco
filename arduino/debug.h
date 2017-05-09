/*
 * debug.h
 *
 *  Created on: 2016. dec. 21.
 *      Author: Sanyi
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#define DBG


void debugTrackSpeedRequestArrived(int index, int value);
void debugSwitchChangeRequestArrived(int index);
void debugServoChangeRequestArrived(int index);
void debugParseDone();
void logSwitchChanged(SwitchData *swD);
void logSwitchNotChanged(SwitchData *swD);
void debugSettingPWMTo(byte track, short speed);
void logServoChangeFinished(byte index, byte angle);
void logTrackChangeFinished(byte index, short value);
void logServosInitialized();
void logSwitchesInitialized();
void logTrackInitialized();
void printSwitch();
void printServo();
void printTrack();

#endif /* DEBUG_H_ */
