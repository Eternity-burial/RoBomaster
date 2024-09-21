#include "cmsis_os.h"
#include "io/led/led.hpp"
io::LED led(&htim5);

extern "C" {
void led_task()
{
  led.start();
  // HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  // HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  // HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  while (1) {
    // __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 1000);
    led.set(65535, 0, 0);
    vTaskDelay(200);
    led.set(0, 0, 0);
    vTaskDelay(200);
    led.set(0, 65535, 0);
    vTaskDelay(200);
    led.set(0, 0, 0);
    vTaskDelay(200);
    led.set(0, 0, 65535);
    vTaskDelay(200);
    led.set(0, 0, 0);
    vTaskDelay(200);
  }
}
}
