#include "rc_receiver.h"
#include "conf.h"
#include "controller.h"
#include "imu.h"
#include "radio_controller.h"
//#include "gps_controller.h"

RcReceiver* receiver;
OperationMode mode;
RcCommands commands;

Controller* controller;

RadioController* radio_controller;
//GpsController* gps_controller;

Imu* imu;

const int voltagePin = A2;
const int ledPin = 13;

int ledTimeOn;  //in us
int ledTimeOff; //in us

bool successful_setup;

#ifdef DEBUG_DT
int current_time;
#endif

uint16_t i = 0;

void setup() {

#ifdef DEBUG
  Serial.begin(9600);
#endif

  receiver = RcReceiver::Create();
  imu = new Imu(successful_setup);
  controller = new Controller(imu);
  radio_controller = new RadioController(controller, receiver);
  //gps_controller = GpsController::Create(controller, imu);
  pinMode(13, OUTPUT);
  current_time = millis();
}

void loop() {
  if (i%100 == 0) {
    int voltage = map(analogRead(voltagePin), 0, 1024, 0, 12600); //in mV
    if (voltage < 9000) { // low voltage
      ledTimeOn = 500;
      ledTimeOff = 1000;
    } else if (!successful_setup) { // error 
      ledTimeOn = 100;
      ledTimeOff = 100;
    } else { // normal operation
      ledTimeOn = 1000;
      ledTimeOff = 0;
    }
  }

  if (i%10 == 0) {
    receiver->Update();
    receiver->GetMode(mode);
    radio_controller->Update();
  }

  imu->UpdateOrientation(i%10 == 0);

  controller->Update();

#ifdef DEBUG_VOLTAGE
  Serial.print(voltage);
  Serial.print("\t");
#endif

#ifdef DEBUG_DT
  if (i%100 == 0) {
    int new_time = millis();
    int dt = new_time - current_time;
    current_time = new_time;
    Serial.print(dt);
    Serial.print("\t");
  }
#endif

  i++;

#ifdef DEBUG
  Serial.println();
#endif
}

void updateLed() {
  if (millis() % (ledTimeOn + ledTimeOff) < ledTimeOn) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}
