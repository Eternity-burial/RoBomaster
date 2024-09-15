#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "para_init.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/mecanum/mecanum.hpp"
#include "tools/pid/pid.hpp"

motor::M3508 chassis_lf(1);
motor::M3508 chassis_lr(2);
motor::M3508 chassis_rf(3);
motor::M3508 chassis_rr(4);

io::CAN can_1(&hcan1);
io::DBus remote_mecanum(&huart3);
io::Plotter chassis_plot(&huart1);

tools::PID chassis_lf_pid(
  chassis_lf_pid_dt, chassis_lf_pid_kp, chassis_lf_pid_ki, chassis_lf_pid_kd, chassis_lf_maxout,
  chassis_lf_maxiout, chassis_lf_alpha);

tools::PID chassis_lr_pid(
  chassis_lr_pid_dt, chassis_lr_pid_kp, chassis_lr_pid_ki, chassis_lr_pid_kd, chassis_lr_maxout,
  chassis_lr_maxiout, chassis_lr_alpha);

tools::PID chassis_rf_pid(
  chassis_rf_pid_dt, chassis_lf_pid_kp, chassis_lf_pid_ki, chassis_lf_pid_kd, chassis_rf_maxout,
  chassis_rf_maxiout, chassis_rf_alpha);

tools::PID chassis_rr_pid(
  chassis_rr_pid_dt, chassis_rr_pid_kp, chassis_rr_pid_ki, chassis_rr_pid_kd, chassis_rr_maxout,
  chassis_rr_maxiout, chassis_rr_alpha);

tools::Mecanum chassis(
  chassis_wheel_radius, chassis_half_length, chassis_half_width, chassis_reverse_lf,
  chassis_reverse_lr, chassis_reverse_rf, chassis_reverse_rr);

void chassis_date_calculation(void)
{
  chassis.calc(2 * remote_mecanum.stick_lh, remote_mecanum.stick_lv, 6 * remote_mecanum.stick_rv);
  chassis_lf_pid.calc(chassis.speed_lf, chassis_lf.speed());
  chassis_lr_pid.calc(chassis.speed_lr, chassis_lr.speed());
  chassis_rf_pid.calc(chassis.speed_rf, chassis_rf.speed());
  chassis_rr_pid.calc(chassis.speed_rr, chassis_rr.speed());
}  //底盘运算

void chassis_date_transmit(void) { can_1.send(chassis_lf.tx_id()); };

// 底盘数据发送

extern "C" {
void transmit_task()
{
  while (1) {
    void chassis_date_transmit();
    vTaskDelay(1);
  }
}
}
//数据发送

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan == &hcan1) {
    can_1.recv();
    if (can_1.rx_header_.StdId == chassis_lf.rx_id()) {
      {
        chassis_lf.read(can_1.rx_data_, osKernelSysTick());
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