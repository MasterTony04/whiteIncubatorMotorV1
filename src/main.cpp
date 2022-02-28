#include <Arduino.h>
#include<EEPROM.h>
#include <SimpleTimer.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

SimpleTimer positionTwoTimer(36000);
SimpleTimer demo(180000);

SoftwareSerial WSerial(2, 3);

uint8_t bSwitch = 12, fSwitch = 11, fanOut = 10;
uint8_t motA = A0, motB = A1;
bool forwardMotor = true, startMotor = false, timer2Active = false, timer4Active = false, motorState = false, startTimerTwo = false, startTimerOne = false, startFan = false;
uint8_t restingPosition = 1;

void setup() {

    pinMode(bSwitch, INPUT);
    pinMode(fSwitch, INPUT);
    pinMode(motA, OUTPUT);
    pinMode(motB, OUTPUT);
    pinMode(fanOut, OUTPUT);

    Serial.begin(9600);
    WSerial.begin(9600);

    digitalWrite(motA, LOW);
    digitalWrite(motB, LOW);
    restingPosition = EEPROM.read(0);
}

void loop() {
    StaticJsonDocument<1024> doc;
    if (WSerial.available() > 0) {

        DeserializationError error = deserializeJson(doc, WSerial);
        if (error) {
            Serial.println("Invalid JSON object");

            return;
        }
        Serial.println("JSON Object Received");
        JsonArray array = doc.as<JsonArray>();
        startFan = array[0].as<bool>();
        motorState = array[1].as<bool>();
        serializeJson(doc, Serial);
        Serial.println();
    }

    if (startFan) {
        digitalWrite(fanOut, HIGH);

    }
    else {

        digitalWrite(fanOut, LOW);
    }
////TODO: for demo purposes only
//if(demo.isReady()){
//    startMotor = true;
//    demo.reset();
//}

    if (motorState) {
        startMotor = true;
    }

    if (startMotor == false) {
        digitalWrite(motA, LOW);
        digitalWrite(motB, LOW);
    }

    if (digitalRead(fSwitch) == 0) {
        forwardMotor = false;
        Serial.println("Forward pressed");

        if (restingPosition == 1) {
            startMotor = false;
            restingPosition = 2;
            EEPROM.write(0, restingPosition);
        }

        if (restingPosition == 4 && startTimerTwo == false) {
            startTimerTwo = true;
            positionTwoTimer.reset();

        }
    }

    if (digitalRead(bSwitch) == 0) {
        forwardMotor = true;
        Serial.println("Backwards pressed");

        if (restingPosition == 3) {
            startMotor = false;
            restingPosition = 4;
            EEPROM.write(0, restingPosition);
        }

        if (restingPosition == 2 && startTimerTwo == false) {
            startTimerTwo = true;
            positionTwoTimer.reset();

        }
    }

    if (positionTwoTimer.isReady() && startTimerTwo == true) {
        startMotor = false;
        startTimerTwo = false;
        // Serial.println("Hapa kazi tuu.")
        if (restingPosition == 4) {
            restingPosition = 1;
            EEPROM.write(0, restingPosition);
        }
        if (restingPosition == 2) {

            restingPosition = 3;
            EEPROM.write(0, restingPosition);
        }
        positionTwoTimer.reset();
    }

    if (forwardMotor == true && startMotor == true) {
        digitalWrite(motA, HIGH);
        digitalWrite(motB, LOW);
        Serial.println("F===========");
    }

    if (forwardMotor == false && startMotor == true) {
        digitalWrite(motA, LOW);
        digitalWrite(motB, HIGH);
        Serial.println("B===========");
    }
    Serial.println("****************************");
    Serial.print("Resting Position: ");
    Serial.println(restingPosition);
    Serial.print("start Motor: ");
    Serial.println(startMotor);
    Serial.print("forward Motor: ");
    Serial.println(forwardMotor);
    Serial.println("****************************");
}