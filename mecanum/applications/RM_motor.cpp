#include "motor/rm_motor/rm_motor.hpp"

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "tools/pid/pid.hpp"

motor::GM6020 g6020_1(6);
io::CAN can1(&hcan1);
io::DBus remote(&huart3);
tools::PID pid_1(0.001, 1.0, 0.0, 0.0, 13.0, 2.0, 1);
io::Plotter plotter(&huart1);

extern "C" {
void rm_motor_Task()
{
  while (1) {
    remote.restart();
    while (!remote.is_open())
      ;
    // DBusSwitchMode::DOWN
    if (remote.switch_r == io::DBusSwitchMode::DOWN) {
      g6020_1.read(can1.rx_data_, osKernelSysTick());
      g6020_1.cmd(0);
      g6020_1.write(can1.tx_data_);
      can1.send(g6020_1.tx_id());

      vTaskDelay(1);
    }
    else if (remote.switch_r == io::DBusSwitchMode::MID) {
      g6020_1.read(can1.rx_data_, osKernelSysTick());
      pid_1.calc(10, g6020_1.speed());
      g6020_1.cmd(pid_1.out);
      g6020_1.write(can1.tx_data_);
      can1.send(g6020_1.tx_id());

      vTaskDelay(1);
    }
  }
}

extern "C" void uart_task()
{
  remote.restart();

  while (!remote.is_open())
    ;

  while (true) {
    plotter.plot(remote.stick_lv, remote.stick_lh, remote.stick_rh, remote.stick_rv);
    osDelay(100);
  }
}
// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
// {
//   if (hcan == &hcan1) {
//     can1.recv();
//     if (can1.rx_header_.StdId == g6020_1.rx_id()) {
//       {
//         g6020_1.read(can1.rx_data_, osKernelSysTick());
//       }
//     }
//     return;
//   }
// }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart == &huart3) {
    remote.update(osKernelSysTick());
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.restart();
  }
}
