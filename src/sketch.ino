#include "rc_receiver.h"

RcReceiver* receiver;
RcCommands commands;
OperationMode mode;

void setup() {
	Serial.begin(9600);
	receiver = RcReceiver::Create();
}

void loop() {
	receiver->Update();
	receiver->GetMode(mode);
	receiver->GetCommands(commands);
	Serial.print("Mode: ");
	Serial.println(mode);

	Serial.print("Elevation: ");
	Serial.println(commands.pitch);
	Serial.print("Bank: ");
	Serial.println(commands.roll);
	Serial.print("Yaw: ");
	Serial.println(commands.yaw);
	Serial.print("Throttle: ");
	Serial.println(commands.throttle);
  delay(1000);
}
