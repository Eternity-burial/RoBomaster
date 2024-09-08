// #include "cmsis_os.h"
// #include "io/dbus/dbus.hpp"
// #include "io/plotter/plotter.hpp"

// io::DBus remote(&huart3);
// io::Plotter plotter(&huart1);

// extern "C" void uart_task()
// {
//   remote.start();

//   while (!remote.is_open())
//     ;

//   while (true) {
//     plotter.plot(remote.stick_lv, remote.stick_lh, remote.stick_rh, remote.stick_rv);
//     osDelay(100);
//   }
// }

// extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
// {
//   if (huart == &huart3) {
//     remote.update(osKernelSysTick());
//   }
// }

// extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
// {
//   if (huart == &huart3) {
//     remote.start();
//   }
// }