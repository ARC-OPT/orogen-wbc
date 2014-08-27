
# Test WBC

dynamic_transform "aila_right_setpoint_proxy.output",
  "setpoint_hand_r" => "Rover_base"

dynamic_transform "aila_left_setpoint_proxy.output",
  "setpoint_hand_l" => "Rover_base"

dynamic_transform "aila_body_setpoint_proxy.output",
  "setpoint_body" => "Rover_base"
  
dynamic_transform "aila_keep_hands_parallel_proxy",
  "setpoint_keep_parallel" => "Hand_r"

#dynamic_transform "aila_board_detector.pose",
#  "iss_panel" => "LeftCamera"

dynamic_transform "servoing_generator.LeftCamera2RoverBase",
    "LeftCamera" => "Rover_base"

dynamic_transform "aila_wbc.pose_p0_cart_cart_pos_ctrl_right_arm",
  "Hand_r" => "Rover_base"

dynamic_transform "aila_wbc.pose_p1_cart_cart_pos_ctrl_left_arm",
  "Hand_l" => "Rover_base"

dynamic_transform "aila_wbc.pose_p1_cart_keep_hands_parallel_ctrl",
  "Hand_r" => "Hand_l"
  
dynamic_transform "aila_wbc.pose_p5_cart_body_posture_ctrl",
  "Chest" => "Rover_base"


# Test FT Control

dynamic_transform "aila_chain_publisher.Hand_rToForce_Sensor_r",
  "Hand_r" => "Force_Sensor_r"

dynamic_transform "aila_chain_publisher.Hand_lToForce_Sensor_l",
  "Hand_l" => "Force_Sensor_l"

# Test FT Processing

dynamic_transform "aila_chain_publisher.Force_Sensor_rToRover_base",
  "Force_Sensor_r" => "Rover_base"

dynamic_transform "aila_chain_publisher.Force_Sensor_lToRover_base",
  "Force_Sensor_l" => "Rover_base"
