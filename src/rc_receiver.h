#ifndef QUADCOPTER_RC_RECEIVER_H_
#define QUADCOPTER_RC_RECEIVER_H_

#include <stdint.h>

enum OperationMode {
  gps,
  rc
};

struct RcCommands {
  float pitch = 0;
  float roll = 0;
  float yaw = 0;
  float throttle = 0;
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
    roll = 0,
    pitch = 1,
    throttle = 2,
    yaw = 3,
    mode = 4,
    unmapped1 = 5,
    unmapped2 = 6,
    unmapped3 = 7
  };

  const float kModeCutoff = 1400;

  // indexed by channel, 0-7
  const int kPins[8] {0, 0, 0, 0, 0, 0, 0, 0};
  
  uint8_t update_flags_ = 0;
  uint16_t inputs_[8];
  
  // shared variables
  volatile uint8_t update_flags_shared_ = 0;
  volatile uint16_t inputs_shared_[8];
  
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
