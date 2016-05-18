#ifndef QUADCOPTER_RC_RECEIVER_H_
#define QUADCOPTER_RC_RECEIVER_H_

#include <stdint.h>

#include "CustomServo.h"

enum OperationMode {
  gps,
  rc
};

struct RcCommands {
  //in % of maximum
  float yaw = 0;
  float attitude = 0;
  float bank = 0;
  float throttle = 0;
  float kp_adj = 0;
  float ki_adj = 0;
};

class RcReceiver {
 public:
  static RcReceiver* Create();
  ~RcReceiver();

  void Update();
  
  void GetMode(OperationMode& out);
  void GetCommands(RcCommands& out);

 private:
  RcReceiver() {};
  enum {
    bank = 0,
    attitude = 1,
    throttle = 2,
    yaw = 3,
    kp_adj  = 4,
    ki_adj  = 5,
    mode = 6,
    unmapped3 = 7
  };

  static const uint16_t kModeCutoff = 2952;
  static const uint16_t kMinPulseLength = 2112;
  static const uint16_t kMaxPulseLength = 3792;

  // indexed by channel, 0-7
  const uint8_t kPins[8] {2, 3, 4, 5, 6, 7, 0, 0};
  
  uint8_t update_flags_ = 0;
  uint16_t inputs_[8] = {};
  
  // shared variables
  volatile uint8_t update_flags_shared_ = 0;
  volatile uint16_t inputs_shared_[8];

  volatile uint16_t periods_[8];
  
  // interrupt variables
  uint16_t input_start_times_[8];
  
  void Interrupt(int channel);
  void SetInterrupts();

  static void Interrupt0();
  static void Interrupt1();
  static void Interrupt2();
  static void Interrupt3();
  static void Interrupt4();
  static void Interrupt5();
  static void Interrupt6();
  static void Interrupt7();
};

#endif
