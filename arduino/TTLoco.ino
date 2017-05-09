#include "Arduino.h"
#include <Servo.h>
#include <math.h>
#include "ttloco.h"
#include "data.h"
#include "debug.h"
#include "EEPROM.h"


/**
 * Első kör:
 *   - csak az alsó kör vezérelhető, a váltó (hozzá a fémgyök), sorompó
 *   - 2x 2db LED, mely megmutatja, hogy a váltó melyik ágra engedi a mozdonyt
 *   Mozdony:>  PWM: pin 11, irány: pin 10-9
 *   Váltó:>  servo: pin 8, gyök: pin 7
 *   Sorompó: pin 6
 *   LED: pin 5
 *
 * Második kör:
 *   - a középső kör bekerül a 4 váltóval, gyökökkel, mellékág lekapcsolható, piros LED jelzi, ha nincs áram alatt
 *
 * Harmadik kör:
 *   - fölső ág (2. PWM), 2 váltó, fönti (ez már Nano boarddal nem megy)
 */


ChangeTask pending_track_tasks[NUM_OF_TRACKS];
ChangeTask pending_servo_tasks[NUM_OF_SERVOS];
unsigned int blinkDelay = 1000;

/**
 * commands:
 *   tr: <track_num> <speed>
 * Change track speed on track_num to speed (-MAX_SPEED - MAX_SPEED)
 *   sw: <switch_num>
 * Toggle switch
 *   se: <servo_num>
 * Toggle servo
 *   li:
 * Lists all configured items
 * All commands must be finished by a \n character (no whitespaces are allowed after number or :)!
 */

int scaleTrackSpeed(int slider_value) {
    return map(slider_value, -MAX_SPEED, MAX_SPEED, -PWM_LIMIT_HIGH, PWM_LIMIT_HIGH);
}

void initiateServoChange(int index) {
    ServoData* sd = &servoData[index];
    float speed = sd->switching_speed;
    ChangeTask* task = &pending_servo_tasks[index];
    if (speed == 0.0) {
        task->change_speed = 180.0 / 500.0; // half round was done around 500ms without any load
    } else {
        task->change_speed = speed;
    }
    task->timeStamp = millis();
    task->finished = false;
    sd->servo_state = 1 - sd->servo_state;
    //        TIMSK2 = 0;
    servos[index].attach(sd->servoPin);
}

void parseInput() {
    String command = Serial.readStringUntil(':');
    int index = Serial.parseInt();
    if (command == "tr") {
        int value = Serial.parseInt();
        debugTrackSpeedRequestArrived(index, value);
        if (index >= NUM_OF_TRACKS || index < 0) {
            Serial.println(F("Err:Track index is out of bounds"));
            return;
        }
        if (abs(value) > MAX_SPEED) {
            Serial.println(F("Err:Track speed is out of bounds"));
            return;
        }

        ChangeTask *task = &pending_track_tasks[index];
        TrackData *td = &trackData[index];
        td->speed = scaleTrackSpeed(value);
        task->change_speed = TRACK_CHANGE_SPEED;
        task->timeStamp = millis();
        task->finished = false;
    } else if (command == "sw") {
        debugSwitchChangeRequestArrived(index);
        if (index >= NUM_OF_SWITCHES || index < 0) {
            Serial.println(F("Err:Switch index out of bounds"));
            return;
        }
        changeSwitch(&switchData[index]);
    } else if (command == "se") {
        debugServoChangeRequestArrived(index);
        if (index >= NUM_OF_SERVOS || index < 0) {
            Serial.println(F("Err:Servo index out of bounds"));
            return;
        }

        initiateServoChange(index);
    } else if (command == "li") {
        printList();
    } else {
        Serial.print(F("ERR:Not recognized command: "));
        Serial.println(command);
    }
    Serial.readStringUntil('\n');
    debugParseDone();
}

void changeSwitch(SwitchData *swD) {
    swD->state = 1 - swD->state;
    digitalWrite(swD->pin, swD->state);
    EEPROM.update(swD->index + EEPROM_SWITCH_OFFSET, swD->state);
    logSwitchChanged(swD);
}

void changeSwitchTo(SwitchData *swD, byte state) {
    if(swD->state != state) {
        changeSwitch(swD);
    } else {
        logSwitchNotChanged(swD);
    }
}

boolean lastZeroSpeedLogged = false;
void setTrackSpeed(byte track, short speed) {
    TrackData *td = &trackData[track];
    digitalWrite(td->pinForward, speed > PWM_LIMIT_LOW ? HIGH : LOW);
    digitalWrite(td->pinBack, speed < -PWM_LIMIT_LOW ? HIGH : LOW);
    short pwmValue;
    if (abs(speed) < PWM_LIMIT_LOW) {
        pwmValue = 0;
    } else {
        pwmValue = speed;
        lastZeroSpeedLogged = false;
    }
    analogWrite(td->pinPWM, abs(pwmValue));
    if(!lastZeroSpeedLogged) {
        lastZeroSpeedLogged = pwmValue==0;
        debugSettingPWMTo(track, pwmValue);
    }
}

void initServos() {
    for (byte b = 0; b < NUM_OF_SERVOS; b++) {
        ServoData *sd = &servoData[b];
        servos[b].write(sd->limits[sd->servo_state]);
    }
    logServosInitialized();
}

void initSwitches() {
    for (byte b = 0; b < NUM_OF_SWITCHES; b++) {
        SwitchData *swD = &switchData[b];
        pinMode(swD->pin, OUTPUT);
        digitalWrite(swD->pin, swD->state);
        swD->index = b;
    }
    delay(500);
    logSwitchesInitialized();
}

void initSpeed() {
    for (byte b = 0; b < NUM_OF_TRACKS; b++) {
        TrackData *td = &trackData[b];
        pinMode(td->pinPWM, OUTPUT);
        pinMode(td->pinBack, OUTPUT);
        pinMode(td->pinForward, OUTPUT);
        setTrackSpeed(b, td->speed);
    }
    logTrackInitialized();
}

void initTrackTasks() {
    for (byte b = 0; b < NUM_OF_TRACKS; b++) {
        ChangeTask *ct = &pending_track_tasks[b];
        ct->index = b;
        ct->finished = true;
        ct->value = 0;
    }
}

void initServoTasks() {
    for (byte b = 0; b < NUM_OF_SERVOS; b++) {
        ChangeTask *ct = &pending_servo_tasks[b];
        ServoData *sd = &servoData[b];
        ct->index = b;
        ct->finished = true;
        ct->value = sd->limits[sd->servo_state];
    }
}

void initTasks() {
    initTrackTasks();
    initServoTasks();
#ifdef DBG
    Serial.println(F("DBG: Tasks initialized."));
#endif
}

void processOngoingServoChange(ChangeTask &task, unsigned long currentTime) {
    long delay = currentTime - task.timeStamp;
    if (delay == 0) {
        return;
    }
    ServoData *sd = &servoData[task.index];
    task.timeStamp = currentTime;
    float change = task.change_speed * delay;
    float oldValue = task.value;
    int destValue = sd->limits[sd->servo_state];
    if (task.value < destValue) {
        task.value += change;
        task.finished = task.value >= destValue;
    } else {
        task.value -= change;
        task.finished = task.value <= destValue;
    }
    if (int(oldValue) != int(task.value)) {
        servos[task.index].write(int(task.value));
    }
    if (task.finished) {
        task.value = destValue;
        EEPROM.update(task.index + EEPROM_SERVO_OFFSET, sd->servo_state);
        SwitchData *switchData = sd->extraSwitch;
        while (switchData != NULL) {
            changeSwitchTo(switchData, sd->servo_state);
            switchData = switchData->next;
        }
        logServoChangeFinished(task.index, task.value);
    }
}

void processOngoingTrackSpeedChange(ChangeTask &task, unsigned long currentTime) {
    long delay = currentTime - task.timeStamp;
    if (delay == 0) {
        return;
    }
    float change = task.change_speed * delay;
    int oldValue = lround(task.value);
    task.timeStamp = currentTime;
    int destValue = trackData[task.index].speed;
    if (task.value < destValue) {
        task.value += change;
        task.finished = task.value >= destValue;
    } else {
        task.value -= change;
        task.finished = task.value <= destValue;
    }
    if (task.finished) {
        task.value = destValue;
        logTrackChangeFinished(task.index, task.value);
    }
    if (oldValue != lround(task.value)) {
        setTrackSpeed(task.index, lround(task.value));
    }
    if(abs(task.value)<PWM_LIMIT_LOW) {
        task.timeStamp--; // let's cheat, so the stopped train will reverse almost immediately (after 2*PWM_LIMIT_LOW processXX calls ~ few us)
    }
}

void processOngoingTasks() {
    unsigned long currentTime = 0;
    currentTime = millis();
    boolean processing = false;
    for (byte b = 0; b < NUM_OF_TRACKS; b++) {
        if (!pending_track_tasks[b].finished) {
            processOngoingTrackSpeedChange(pending_track_tasks[b], currentTime);
            processing = true;
        }
    }

    for (byte b = 0; b < NUM_OF_SERVOS; b++) {
        if (!pending_servo_tasks[b].finished) {
            processOngoingServoChange(pending_servo_tasks[b], currentTime);
            processing = true;
        } else if(pending_servo_tasks[b].change_speed!=0.0 && currentTime - pending_servo_tasks[b].timeStamp > 200) {
            servos[b].detach();
            pending_servo_tasks[b].change_speed = 0.0;
            processing = true;
//            TIMSK2 = _BV(TOIE2);
        }
    }
    blinkDelay = processing ? 200 : 1000;
}

volatile byte isrcounter = 0;
volatile long isrMicros = 0;

ISR(TIMER2_OVF_vect) {
    interrupts();
    voltage_track1 = analogRead(SENSEPIN_TRACK1);
    voltage_track1 = analogRead(SENSEPIN_TRACK1);
    voltage_track2 = analogRead(SENSEPIN_TRACK2);
    voltage_track2 = analogRead(SENSEPIN_TRACK2);
#ifdef DBG
    if(voltage_track1 > max_voltage_track1)
        max_voltage_track1 = voltage_track1;
    if(voltage_track2 > max_voltage_track2)
        max_voltage_track2 = voltage_track2;
#endif
}

void setupPWM() {
    // pin 3, 11 freq set to 122.55Hz, http://playground.arduino.cc/Main/TimerPWMCheatsheet
    TCCR2B = (TCCR2B & 0b11111000) | 0x06;

    // setting up interrupt on overflow, for current sensing at the middle of PWM high stage
    noInterrupts();
    TIMSK2 = _BV(TOIE2);
    interrupts();
}

void printList() {
    Serial.println(F("Printing current settings."));
    printTrack();
    printServo();
    printSwitch();
}

void initStatesInEeprom(uint32_t checkSum) {
    EEPROM.put(0, checkSum);
    for (int i = 0; i < NUM_OF_SERVOS; i++) {
        EEPROM.write(EEPROM_SERVO_OFFSET + i, servoData[i].servo_state);
    }
    for (int i = 0; i < NUM_OF_SWITCHES; i++) {
        EEPROM.write(EEPROM_SWITCH_OFFSET + i, switchData[i].state);
    }
}

void loadStatesFromEeprom() {
    for (int i = 0; i < NUM_OF_SERVOS; i++) {
        byte servoState = EEPROM.read(EEPROM_SERVO_OFFSET + i);
        servoData[i].servo_state = servoState;
    }
    for (int i = 0; i < NUM_OF_SWITCHES; i++) {
        byte switchState = EEPROM.read(EEPROM_SWITCH_OFFSET + i);
        switchData[i].state = switchState;
    }
}

void loadDataFromEEPROM() {
    unsigned long eepromChecksum;
    EEPROM.get(0, eepromChecksum);
    unsigned long calcChecksum = ((long)NUM_OF_TRACKS << 16) + ((long)NUM_OF_SERVOS << 8) + (NUM_OF_SWITCHES);
    Serial.print("calculated checkSum: 0x");
    Serial.print(calcChecksum, HEX);
    Serial.print(", stored checkSum: 0x");
    Serial.println(eepromChecksum, HEX);
    if(eepromChecksum != calcChecksum) {
        initStatesInEeprom(calcChecksum);
    } else {
        loadStatesFromEeprom();
    }
}

void initAnalogPrescaler() {
    // must be set before setting up PWM (and its interrupt!)
    // change analog prescaler from x128 to x64, faster, yet precise enough
    ADCSRA = _BV(ADEN) + _BV(ADPS2) + _BV(ADPS1);
    analogRead(SENSEPIN_TRACK1);
    analogRead(SENSEPIN_TRACK2);
}

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    digitalWrite(13, HIGH);
    Serial.println("Starting loco...");
    initAnalogPrescaler();
    setupPWM();
    loadDataFromEEPROM();
    initSwitches();
    initServos();
    initSpeed();
    initTasks();
    digitalWrite(13, LOW);
    printInitScreen();
}

long blinkTimeStamp;
byte pin13state = 0;

void blinkLED13() {
    if (millis() - blinkTimeStamp >= blinkDelay) {
        digitalWrite(13, pin13state);
        pin13state = 1 - pin13state;
        blinkTimeStamp = millis();
    }
}

long shortCircuitTimeStamp = 0;

void checkForShortCircuit() {
    int track1 = 0;
    int track2 = 0;
    int maxTrack1 = 0;
    int maxTrack2 = 0;
    noInterrupts();
    track1 = voltage_track1;
    track2 = voltage_track2;
    interrupts();
    if((track1 > SENSEPIN_LIMIT) || (track2 > SENSEPIN_LIMIT)) {
        byte tid = (track1 > SENSEPIN_LIMIT) ? 1 : 2;
        int val = tid == 1?track1:track2;
        Serial.print(F("SHORT CIRCUIT PROTECTION ACTIVATED FOR TRACK: "));
        Serial.print(tid);
        Serial.print(F(" at current: "));
        Serial.print((5.0 * val / 1024.0) / 0.47);
        Serial.print(F("A ("));
        Serial.print(val);
        Serial.println(')');
        for(int i=0; i<NUM_OF_TRACKS; i++) {
            analogWrite(trackData[i].pinPWM, 0);
            digitalWrite(trackData[i].pinBack, LOW);
            digitalWrite(trackData[i].pinForward, LOW);
        }
        for(int i=0; i<NUM_OF_SERVOS; i++) {
            servos[i].detach();
        }
        while(true) {
            digitalWrite(13, HIGH);
            delay(50);
            digitalWrite(13, LOW);
            delay(100);
        }
    } else {
#ifdef DBG
        long ts = millis();
        if(ts - shortCircuitTimeStamp > 5000) {
            long micros = 0;
            noInterrupts();
            maxTrack1 = max_voltage_track1;
            max_voltage_track1 = 0;
            maxTrack2 = max_voltage_track2;
            max_voltage_track2 = 0;
            micros = isrMicros;
            interrupts();

            Serial.print(F("Sensing summary ("));
            Serial.print(micros);
            Serial.println(F("):"));
            Serial.print(F("Voltage1, maxVoltage1: "));
            Serial.print(5.0 * track1 / 1024.0);
            Serial.print(F("V, "));
//            Serial.print(voltage_track1);
            Serial.print(5.0 * maxTrack1 / 1024.0);
            Serial.println('V');
            Serial.print(F("Voltage2, maxVoltage2: "));
            Serial.print(5.0 * track2 / 1024.0);
//            Serial.print(track2);
            Serial.print(F("V, "));
            Serial.print(5.0 * maxTrack2 / 1024.0);
            Serial.println('V');
            shortCircuitTimeStamp = ts;
        }
#endif
    }
}

void loop() {
    // put your main code here, to run repeatedly:
    if (Serial.available() > 3) {
        parseInput();
    }

    processOngoingTasks();

    checkForShortCircuit();

    blinkLED13();
}

