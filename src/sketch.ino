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
uint16_t i = 0;

#ifdef DEBUG
bool printed = false;
#endif

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
  {
    static int i = 0;
    i++;
    if (i == 50) {
      printed = true;
      Serial.print(voltage);
      Serial.print("\t");
      i = 0;
    }
  }
#endif

#ifdef DEBUG_DT
  {
    static int i = 0;
    static uint32_t current_time = 0;
    i++;
    if (i == 50) {
      printed = true;
      int new_time = millis();
      int dt = new_time - current_time;
      current_time = new_time;
      Serial.print(dt);
      Serial.print(" (ms/100 loops)");
      Serial.print("\t");
      i = 0;
    }
  }
#endif

  i++;

#ifdef DEBUG
  if (printed) {
    Serial.println();
    printed = false;
  }
#endif
}

void updateLed() {
  if (millis() % (ledTimeOn + ledTimeOff) < ledTimeOn) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}
