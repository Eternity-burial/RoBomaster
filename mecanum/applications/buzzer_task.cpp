#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

io::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);

extern "C" {
void buzzer_task()
{
  buzzer.start();
  buzzer.set(2000, 0.1);
  vTaskDelay(100);
  buzzer.set(5000, 0.1);
  vTaskDelay(200);
  buzzer.stop();

  while (1) {
    vTaskDelay(1000);
  }
}
}