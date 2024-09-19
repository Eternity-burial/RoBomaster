#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "para_init.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/mecanum/mecanum.hpp"
#include "tools/pid/pid.hpp"

enum class Mode
{
  zero_force_mode,
  rc_control_mode,
};

extern motor::M3508 chassis_lf;
extern motor::M3508 chassis_lr;
extern motor::M3508 chassis_rf;
extern motor::M3508 chassis_rr;

extern io::CAN can_1;
extern io::CAN can_2;
extern io::DBus remote_mecanum;
extern io::Plotter chassis_plot;

extern tools::PID chassis_lf_pid;
extern tools::PID chassis_lr_pid;
extern tools::PID chassis_rf_pid;
extern tools::PID chassis_rr_pid;
extern tools::Mecanum chassis;

Mode mode = Mode::zero_force_mode;

float chassis_lf_real_speed;
float chassis_lr_real_speed;
float chassis_rf_real_speed;
float chassis_rr_real_speed;

void mode_receive(void)
{
  switch (remote_mecanum.switch_r) {
    case io::DBusSwitchMode::DOWN:
      mode = Mode::zero_force_mode;
      break;
    case io::DBusSwitchMode::MID:
      mode = Mode::rc_control_mode;
      break;
    case io::DBusSwitchMode::UP:
      mode = Mode::rc_control_mode;
      break;
    default:
      break;
  }
}  //控制模式读取

// void mode_setting(void)
// {
//   switch (mode) {
//     case Mode::zero_force_mode:

//       break;
//     case Mode::rc_control_mode:
//       break;
//     default:
//       break;
//   }
// }  //模式、死区设定

void chassis_date_reveive(void)
{
  chassis_lf.read(can_2.rx_data_, osKernelSysTick());
  chassis_lr.read(can_2.rx_data_, osKernelSysTick());
  chassis_rf.read(can_2.rx_data_, osKernelSysTick());
  chassis_rr.read(can_2.rx_data_, osKernelSysTick());

}  //底盘数据读取

void chassis_real_speed(void)
{
  chassis_lf_real_speed = chassis_lf.speed() * mecanum_radius;
  chassis_lr_real_speed = chassis_lr.speed() * mecanum_radius;
  chassis_rf_real_speed = chassis_rf.speed() * mecanum_radius;
  chassis_rr_real_speed = chassis_rr.speed() * mecanum_radius;
}  //底盘实际速度解算

void chassis_date_plot(void)
{
  chassis_plot.plot(chassis_lf_real_speed
                    // , chassis_lr_real_speed, chassis_rf_real_speed, chassis_rr_real_speed
  );
}  //底盘数据打印

void chassis_date_calculation(void)
{
  chassis.calc(2 * remote_mecanum.stick_lv, remote_mecanum.stick_lh, 6 * remote_mecanum.stick_rh);
  chassis_lf_pid.calc(chassis.speed_lf, chassis_lf.speed());
  chassis_lr_pid.calc(chassis.speed_lr, chassis_lr.speed());
  chassis_rf_pid.calc(chassis.speed_rf, chassis_rf.speed());
  chassis_rr_pid.calc(chassis.speed_rr, chassis_rr.speed());
}  //底盘运算

void chassis_date_write(void)
{
  if (mode == Mode::zero_force_mode) {
    chassis_lf.cmd(0);
    chassis_lf.write(can_2.tx_data_);
    chassis_lr.cmd(0);
    chassis_lr.write(can_2.tx_data_);
    chassis_rf.cmd(0);
    chassis_rf.write(can_2.tx_data_);
    chassis_rr.cmd(0);
    chassis_rr.write(can_2.tx_data_);
    // can_2.send(chassis_lf.tx_id());
  }
  else if (mode == Mode::rc_control_mode) {
    // chassis_lf.cmd(chassis_lf_pid.out);
    chassis_lf.cmd(chassis_lf_pid.out);
    chassis_lf.write(can_2.tx_data_);
    chassis_lr.cmd(chassis_lr_pid.out);
    chassis_lr.write(can_2.tx_data_);
    chassis_rf.cmd(chassis_rf_pid.out);
    chassis_rf.write(can_2.tx_data_);
    chassis_rr.cmd(chassis_rr_pid.out);
    chassis_rr.write(can_2.tx_data_);
    // can_2.send(chassis_lf.tx_id());
  }
};
//底盘数据写入

extern void chassis_date_transmit(void);

extern "C" {
void chassis_task()
{
  remote_mecanum.restart();
  while (!remote_mecanum.is_open())
    ;
  while (1) {
    mode_receive();
    chassis_date_reveive();
    chassis_real_speed();
    chassis_date_plot();
    chassis_date_calculation();
    chassis_date_write();
    chassis_date_transmit();
    vTaskDelay(20);
  }
}
}
