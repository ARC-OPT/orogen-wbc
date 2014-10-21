
# Test WBC

dynamic_transform "cart_pos_ctrl_right_proxy.output",
  "setpoint_hand_r" => "base_link"

dynamic_transform "wbc.pose_cart_pos_ctrl_right_arm",
  "Hand_r" => "base_link"
