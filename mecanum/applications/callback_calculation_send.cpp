#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "para_init.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/mecanum/mecanum.hpp"
#include "tools/pid/pid.hpp"
motor::M3508 chassis_lf(chassis_left_front_motor);
motor::M3508 chassis_lr(chassis_left_rear_motor);
motor::M3508 chassis_rf(chassis_right_front_motor);
motor::M3508 chassis_rr(chassis_right_rear_motor);

io::CAN can_1(&hcan1);
io::CAN can_2(&hcan2);
io::DBus remote_mecanum(&huart3);
io::Plotter chassis_plot(&huart1);

tools::PID chassis_lf_pid(
  chassis_lf_pid_dt, chassis_lf_pid_kp, chassis_lf_pid_ki, chassis_lf_pid_kd, chassis_lf_maxout,
  chassis_lf_maxiout, chassis_lf_alpha);

tools::PID chassis_lr_pid(
  chassis_lr_pid_dt, chassis_lr_pid_kp, chassis_lr_pid_ki, chassis_lr_pid_kd, chassis_lr_maxout,
  chassis_lr_maxiout, chassis_lr_alpha);

tools::PID chassis_rf_pid(
  chassis_rf_pid_dt, chassis_rf_pid_kp, chassis_rf_pid_ki, chassis_rf_pid_kd, chassis_rf_maxout,
  chassis_rf_maxiout, chassis_rf_alpha);

tools::PID chassis_rr_pid(
  chassis_rr_pid_dt, chassis_rr_pid_kp, chassis_rr_pid_ki, chassis_rr_pid_kd, chassis_rr_maxout,
  chassis_rr_maxiout, chassis_rr_alpha);

tools::Mecanum chassis(
  mecanum_radius, chassis_half_length, chassis_half_width, chassis_reverse_lf, chassis_reverse_lr,
  chassis_reverse_rf, chassis_reverse_rr);

void chassis_date_transmit(void)
{
  can_1.send(chassis_lf.tx_id());
  // flag = 1;
  can_2.send(chassis_lf.tx_id());
};

// 底盘数据发送

void can_filter_init(void)
{
  can_1.config();
  can_1.start();
  can_2.config();
  can_2.start();
}

extern "C" {
void transmit_task()
{
  while (1) {
    chassis_date_transmit();
    vTaskDelay(3);
  }
}
}
//数据发送

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan == &hcan1) {
    can_1.recv();
    if (can_1.rx_id == chassis_lf.rx_id()) {
      {
        chassis_lf.read(can_1.rx_data, osKernelSysTick());
      }
    }
    return;
  }
  if (hcan == &hcan2) {
    can_2.recv();
    // switch (can_2.rx_header_.StdId)
    // {
    // case chassis_lf.rx_id():
    //   chassis_lf.read(can_2.rx_data_, osKernelSysTick());
    //   break;
    // case chassis_lr.rx_id():
    //   chassis_lf.read(can_2.rx_data_, osKernelSysTick());
    //   break;
    // case chassis_rf.rx_id():
    //   chassis_rf.read(can_2.rx_data_, osKernelSysTick());
    //   break;
    // case chassis_rr.rx_id():
    //   chassis_rr.read(can_2.rx_data_, osKernelSysTick());
    //   break;
    // default:
    //   break;
    // }
    if (can_2.rx_id == chassis_lf.rx_id()) {
      {
        chassis_lf.read(can_2.rx_data, osKernelSysTick());
      }
    }
    if (can_2.rx_id == chassis_lr.rx_id()) {
      {
        chassis_lr.read(can_2.rx_data, osKernelSysTick());
      }
    }
    if (can_2.rx_id == chassis_rf.rx_id()) {
      {
        chassis_rf.read(can_2.rx_data, osKernelSysTick());
      }
    }
    if (can_2.rx_id == chassis_rr.rx_id()) {
      {
        chassis_rr.read(can_2.rx_data, osKernelSysTick());
      }
    }

    return;
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart == &huart3) {
    remote_mecanum.update(osKernelSysTick());
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote_mecanum.restart();
  }
}