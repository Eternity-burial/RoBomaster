#ifndef _PARA_INIT_HPP_
#define _PARA_INIT_HPP_

// 遥控器拨杆值
constexpr uint8_t RC_SW_UP = 1;
constexpr uint8_t RC_SW_MID = 3;
constexpr uint8_t RC_SW_DOWN = 2;

// 底盘数据

static const float chassis_wheel_radius = 77.0f;
static const float chassis_half_length = 165.0f;
static const float chassis_half_width = 185.0f;
static const bool chassis_reverse_lf = false;
static const bool chassis_reverse_lr = false;
static const bool chassis_reverse_rf = true;
static const bool chassis_reverse_rr = true;

// 底盘电机

// 左前
//电机编号
constexpr uint8_t chassis_left_front_motor = 4;
//PID参数
static const float chassis_lf_pid_dt = 0.001;
static const float chassis_lf_pid_kp = 1.0f;
static const float chassis_lf_pid_ki = 0.0f;
static const float chassis_lf_pid_kd = 0.0f;
static const float chassis_lf_maxout = 500.0f;
static const float chassis_lf_maxiout = 2.0f;
static const float chassis_lf_alpha = 0.1f;

// 左后
constexpr uint8_t chassis_left_rear_motor = 1;
//PID参数
static const float chassis_lr_pid_dt = 0.001;
static const float chassis_lr_pid_kp = 1.0;
static const float chassis_lr_pid_ki = 0.0;
static const float chassis_lr_pid_kd = 0.0;
static const float chassis_lr_maxout = 13.0f;
static const float chassis_lr_maxiout = 2.0f;
static const float chassis_lr_alpha = 0.1f;

//右前
constexpr uint8_t chassis_right_front_motor = 3;
//PID参数
static const float chassis_rf_pid_dt = 0.001;
static const float chassis_rf_pid_kp = 1.0;
static const float chassis_rf_pid_ki = 0.0;
static const float chassis_rf_pid_kd = 0.0;
static const float chassis_rf_maxout = 13.0f;
static const float chassis_rf_maxiout = 2.0f;
static const float chassis_rf_alpha = 0.1f;

//右后
constexpr uint8_t chassis_right_rear_motor = 2;
//PID参数
static const float chassis_rr_pid_dt = 0.001;
static const float chassis_rr_pid_kp = 1.0;
static const float chassis_rr_pid_ki = 0.0;
static const float chassis_rr_pid_kd = 0.0;
static const float chassis_rr_maxout = 13.0f;
static const float chassis_rr_maxiout = 2.0f;
static const float chassis_rr_alpha = 0.1f;

#endif  // _PARA_INIT_HPP