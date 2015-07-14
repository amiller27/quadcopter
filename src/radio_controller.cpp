#include "radio_controller.h"

RadioController::RadioController(Controller* controller,
                                 RcReceiver* receiver)
    : controller_(controller),
      receiver_(receiver) {}

void RadioController::Update() {
  
}
