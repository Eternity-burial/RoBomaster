#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

io::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);

extern "C" {
void buzzer_task()
{
  buzzer.set(8192, 0);
  buzzer.start();
  vTaskDelay(100);
  buzzer.set(9000, 0.2);
  vTaskDelay(100);
  buzzer.stop();
  vTaskDelay(1000);

  while (1) {
    vTaskDelay(1000);
  }
}
}