require 'orocos'
require 'readline'

Orocos.initialize
Orocos.conf.load_dir('config')

Orocos.run "wbc::WbcVelocityTask" => "wbc", 
           "ctrl_lib::CartesianPositionController" => "cart_pos_ctrl_right_arm", 
           "ctrl_lib::JointRadialPotentialFields" => "upper_limit_avoidance", 
           "ctrl_lib::JointPositionController" => "joint_position_ctrl", 
           "joint_control::FakeJointDriverTask" => "joint_driver" do

   wbc = Orocos::TaskContext.get "wbc" 
   cart_pos_ctrl_right_arm = Orocos::TaskContext.get "cart_pos_ctrl_right_arm"
   upper_limit_avoidance = Orocos::TaskContext.get "upper_limit_avoidance"
   joint_position_ctrl = Orocos::TaskContext.get "joint_position_ctrl"
   joint_driver = Orocos::TaskContext.get "joint_driver"

   Orocos.conf.apply(wbc, ["default"], true)   
   Orocos.conf.apply(cart_pos_ctrl_right_arm, ["default"], true)
   Orocos.conf.apply(upper_limit_avoidance, ["default", "upper_limits"], true)
   Orocos.conf.apply(joint_position_ctrl, ["default"], true)
   Orocos.conf.apply(joint_driver, ["default", "velocity"], true)

   # Configure wbc already here to create ports: Depending on the configured constraints, in- and output ports
   # will be created automatically 
   wbc.configure

   # Connect wbc control output and joint state (wbc <--> joint driver)
   wbc.port("joint_state").connect_to joint_driver.port("joint_state")
   wbc.port("ctrl_out").connect_to joint_driver.port("command")

   # Connect constraints to controllers (Priority 0)
   wbc.port("ref_cart_pos_ctrl_right_arm").connect_to cart_pos_ctrl_right_arm.port("control_output")
   wbc.port("pose_cart_pos_ctrl_right_arm").connect_to cart_pos_ctrl_right_arm.port("feedback")

   # Connect constraints to controllers (Priority 1)
   wbc.port("ref_upper_joint_limits").connect_to upper_limit_avoidance.port("control_output")
   joint_driver.port("joint_state").connect_to upper_limit_avoidance.port("position")

   # Connect constraints to controllers (Priority 2)
   wbc.port("ref_joint_position_ctrl").connect_to joint_position_ctrl.port("control_output")
   joint_driver.port("joint_state").connect_to joint_position_ctrl.port("feedback")

   cart_pos_ctrl_right_arm.configure
   upper_limit_avoidance.configure
   joint_position_ctrl.configure
   joint_driver.configure

   cart_pos_ctrl_right_arm.start
   upper_limit_avoidance.start
   joint_position_ctrl.start
   joint_driver.start
   wbc.start

   Readline::readline("Press Enter to exit...")
end
