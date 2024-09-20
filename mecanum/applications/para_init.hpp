#ifndef _PARA_INIT_HPP_
#define _PARA_INIT_HPP_

// 遥控器拨杆值
constexpr uint8_t RC_SW_UP = 1;
constexpr uint8_t RC_SW_MID = 3;
constexpr uint8_t RC_SW_DOWN = 2;

// 底盘数据

static const float chassis_half_length = 0.165f;
static const float chassis_half_width = 0.185f;
static const float mecanum_radius = 0.077f;
static const float vx = -2;
static const float vy = 1;
static const float wz = 6;

// extern float chassis_lf_real_speed;
// extern float chassis_lr_real_speed;
// extern float chassis_rf_real_speed;
// extern float chassis_rr_real_speed;

static const bool chassis_reverse_lf = true;
static const bool chassis_reverse_lr = true;
static const bool chassis_reverse_rf = false;
static const bool chassis_reverse_rr = false;

// 底盘电机

// 左前
//电机编号
constexpr uint8_t chassis_left_front_motor = 4;
//PID参数
static const float chassis_lf_pid_dt = 0.003;
static const float chassis_lf_pid_kp = 0.7f;
static const float chassis_lf_pid_ki = 0.0001f;
static const float chassis_lf_pid_kd = -0.001f;
static const float chassis_lf_maxout = 3.0f;
static const float chassis_lf_maxiout = 1.2f;
static const float chassis_lf_alpha = 0.1f;

// 左后
constexpr uint8_t chassis_left_rear_motor = 1;
//PID参数
static const float chassis_lr_pid_dt = 0.003;
static const float chassis_lr_pid_kp = 0.9f;
static const float chassis_lr_pid_ki = 0.0008f;
static const float chassis_lr_pid_kd = -0.001f;
static const float chassis_lr_maxout = 3.0f;
static const float chassis_lr_maxiout = 1.2f;
static const float chassis_lr_alpha = 0.1f;

//右前
constexpr uint8_t chassis_right_front_motor = 3;
//PID参数
static const float chassis_rf_pid_dt = 0.003;
static const float chassis_rf_pid_kp = 0.9f;
static const float chassis_rf_pid_ki = 0.0008f;
static const float chassis_rf_pid_kd = -0.001f;
static const float chassis_rf_maxout = 2.5f;
static const float chassis_rf_maxiout = 0.5f;
static const float chassis_rf_alpha = 0.1f;

//右后
constexpr uint8_t chassis_right_rear_motor = 2;
//PID参数
static const float chassis_rr_pid_dt = 0.003;
static const float chassis_rr_pid_kp = 0.8f;
static const float chassis_rr_pid_ki = 0.0008f;
static const float chassis_rr_pid_kd = -0.001f;
static const float chassis_rr_maxout = 2.5f;
static const float chassis_rr_maxiout = 0.5f;
static const float chassis_rr_alpha = 0.1f;

#endif  // _PARA_INIT_HPP