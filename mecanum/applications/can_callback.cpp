#include "motor/rm_motor/rm_motor.hpp"
// #include ""

extern motor::M3508 chassis_lf(1);
extern motor::M3508 chassis_lr(2);
extern motor::M3508 chassis_rf(3);
extern motor::M3508 chassis_rr(4);

// void get_upcommand(uint8_t * date)

extern "C" {
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  switch (rx_header.StdId) {
    case chassis_left_id:
      motor.decode_motor_measure(chassis_left_motor, rx_data);
      break;
    case chassis_right_id:
      motor.decode_motor_measure(chassis_right_motor, rx_data);
      break;
    case lift_id:
      motor.decode_motor_measure(lift_motor, rx_data);
      break;
    case y_id:
      motor.decode_motor_measure(y_axis_motor, rx_data);
      break;
    case 0X100:
      get_upcommand(rx_data);
      break;
    default:
      break;
  }
}
}