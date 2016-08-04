require 'orocos'

#
# For this example to run, you will need to install orogen/ctrl_lib and orogen/joint_control!
#
# If you want to have a visualization of the robot, checkout the kuka_lbr bundle, install rock-gui/gui-robot_model
# and type
#
#     rock-roboviz <kuka_bundle_path>/data/urdf/kuka_lbr.urdf -s driver:joint_state
#
# If you want to use the wbc GUIs, checkout and install gui/wbc_ctrl_gui and gui/wbc_status_gui
# and type
#
#     wbc_status_gui or wbc_ctrl_gui
#
# to start the respective GUIs
#

Orocos.initialize
Orocos.conf.load_dir('config')

Orocos.run "wbc::WbcVelocityTask" => "wbc",
           "ctrl_lib::CartesianPositionController" => "cartesian_controller",
           "ctrl_lib::JointLimitAvoidance"         => "joint_limit_avoidance",
           "joint_control::FakeJointDriverTask"    => "driver" do

    cartesian_controller  = Orocos::TaskContext.get "cartesian_controller"
    joint_limit_avoidance = Orocos::TaskContext.get "joint_limit_avoidance"
    wbc                   = Orocos::TaskContext.get "wbc"
    driver                = Orocos::TaskContext.get "driver"

    Orocos.conf.apply(wbc,    ["default"], true)
    Orocos.conf.apply(driver, ["default"], true)
    Orocos.conf.apply(cartesian_controller,  ["default"], true)
    Orocos.conf.apply(joint_limit_avoidance, ["default"], true)

    # Configure already here to create dynamic ports for the constraints
    wbc.configure

    # Connect ports

    # WBC with joint driver
    driver.port("joint_state").connect_to wbc.port("joint_state")
    wbc.port("ctrl_out").connect_to driver.port("command")

    # Priority 0: Cartesian Position control
    constraint_name = "cart_position_ctrl_right"
    cartesian_controller.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    wbc.port("state_" + constraint_name).connect_to cartesian_controller.port("feedback")

    # Priority 1: Joint Limit avoidance
    constraint_name = "joint_limit_avoidance"
    joint_limit_avoidance.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    joint_limit_avoidance.port("activation").connect_to wbc.port("weight_" + constraint_name)
    wbc.port("state_" + constraint_name).connect_to joint_limit_avoidance.port("feedback")

    cartesian_controller.configure
    joint_limit_avoidance.configure
    driver.configure

    # Run

    driver.start
    cartesian_controller.start
    joint_limit_avoidance.start
    wbc.start

    Readline.readline("Press Enter to move to a target pose")

    # Set target pose for Cartesian Controller
    target_pose = Types::Base::Samples::RigidBodyState.new
    target_pose.position = Types::Base::Vector3d.new(0.22, -1.44, 0.97)
    target_pose.orientation = Types::Base::Quaterniond.from_euler(Types::Base::Vector3d.new(1.71, -1.57, -0.14), 2,1,0)
    pose_writer = cartesian_controller.port("setpoint").writer
    pose_writer.write(target_pose)

    # Activate Cartesian Constraint
    activation_writer = wbc.port("activation_cart_position_ctrl_right")
    activation_writer.write(1.0)

    # Check if current pose == target pose in a loop
    reached = false
    reader = cartesian_controller.port("current_feedback").reader
    feedback = Types::Base::Samples::RigidBodyState.new
    while not reached
       feedback = reader.read
       if feedback
          print "Target position:  "
          target_pose.position.data.each do |v| print "#{'%.04f' % v} " end
          print "\nCurrent position: "
          feedback.position.data.each do |v| print "#{'%.04f' % v} " end
          print "\n\n"

          if (target_pose.position - feedback.position).norm < 1e-3
             reached = true
             puts "Reached Target Position!"
          end
       end
       sleep 0.1
    end

    Readline.readline("Press Enter to exit")
end
