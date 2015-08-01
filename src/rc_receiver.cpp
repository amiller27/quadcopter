#include "rc_receiver.h"

// tells EnableInterrupt to only register interrupts for pins D0 to D13
// this is needed to be compatible with SoftwareSerial
#define EI_NOTPORTC

// we have to do this here, not in rc_receiver.h,
// because the writer of EnableInterrupt.h defined a bunch
// of global variables which are defined each time the header is included
#include <EnableInterrupt.h>

RcReceiver* g_interrupt_receiver = 0;

RcReceiver* RcReceiver::Create() {
  RcReceiver* result = new RcReceiver();

  if (!g_interrupt_receiver) {
    g_interrupt_receiver = result;
    result->SetInterrupts();
  }

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  TCCR1B |= _BV(CS11);

  return result;
}

void RcReceiver::Update() {
  if (update_flags_shared_) {
    noInterrupts();
    
    update_flags_ = update_flags_shared_;
    
    for (int i = 0; i < 8; i++) {
      inputs_[i] = inputs_shared_[i]; 
    }
    
    update_flags_shared_ = 0;
    
    interrupts();
  }
}

void RcReceiver::GetMode(OperationMode& out) {
  if (inputs_[mode] <= kModeCutoff) {
    out = rc;
  } else {
    out = gps;
  }
}

void RcReceiver::GetCommands(RcCommands& out) {
  out.attitude = constrain(map(inputs_[attitude], kMinPulseLength, kMaxPulseLength, 0, 1), 0, 1);
  out.bank = constrain(map(inputs_[bank], kMinPulseLength, kMaxPulseLength, 0, 1), 0, 1);
  out.yaw = constrain(map(inputs_[yaw], kMinPulseLength, kMaxPulseLength, 0, 1), 0, 1);
  out.throttle = constrain(map(inputs_[throttle], kMinPulseLength, kMaxPulseLength, 0, 1), 0, 1);
  out.aggressiveness = constrain(map(inputs_[aggressiveness], kMinPulseLength, kMaxPulseLength, 0, 1), 0, 1);
}

void RcReceiver::Interrupt(int channel) {
  if (digitalRead(kPins[channel]) == HIGH) {
    input_start_times_[channel] = TCNT1;
  } else {
    inputs_shared_[channel] = (TCNT1 - input_start_times_[channel]) >> 1;
    update_flags_shared_ |= _BV(channel);
  }
}

void RcReceiver::SetInterrupts() {
  enableInterrupt(kPins[0] | PINCHANGEINTERRUPT, Interrupt0, CHANGE);
  enableInterrupt(kPins[1] | PINCHANGEINTERRUPT, Interrupt1, CHANGE);
  enableInterrupt(kPins[2] | PINCHANGEINTERRUPT, Interrupt2, CHANGE);
  enableInterrupt(kPins[3] | PINCHANGEINTERRUPT, Interrupt3, CHANGE);
  enableInterrupt(kPins[4] | PINCHANGEINTERRUPT, Interrupt4, CHANGE);
  enableInterrupt(kPins[5] | PINCHANGEINTERRUPT, Interrupt5, CHANGE);
  enableInterrupt(kPins[6] | PINCHANGEINTERRUPT, Interrupt6, CHANGE);
  enableInterrupt(kPins[7] | PINCHANGEINTERRUPT, Interrupt7, CHANGE);
}

void RcReceiver::Interrupt0() {
  g_interrupt_receiver->Interrupt(0);
}

void RcReceiver::Interrupt1() {
  g_interrupt_receiver->Interrupt(1);
}

void RcReceiver::Interrupt2() {
  g_interrupt_receiver->Interrupt(2);
}

void RcReceiver::Interrupt3() {
  g_interrupt_receiver->Interrupt(3);
}

void RcReceiver::Interrupt4() {
  g_interrupt_receiver->Interrupt(4);
}

void RcReceiver::Interrupt5() {
  g_interrupt_receiver->Interrupt(5);
}

void RcReceiver::Interrupt6() {
  g_interrupt_receiver->Interrupt(6);
}

void RcReceiver::Interrupt7() {
  g_interrupt_receiver->Interrupt(7);
}
