require 'orocos'
require 'readline'

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
           "wbc::HierarchicalLSSolverTask" => "solver",
           "ctrl_lib::CartesianPositionController" => "cartesian_controller",
           "ctrl_lib::CartesianForceController"    => "force_controller",
           "joint_control::FakeJointDriverTask"    => "joint_driver",
           "joint_control::FakeForceDriverTask"    => "force_sensor" do

    cartesian_controller = Orocos::TaskContext.get "cartesian_controller"
    force_controller     = Orocos::TaskContext.get "force_controller"
    wbc                  = Orocos::TaskContext.get "wbc"
    solver                = Orocos::TaskContext.get "solver"
    joint_driver         = Orocos::TaskContext.get "joint_driver"
    force_sensor         = Orocos::TaskContext.get "force_sensor"

    Orocos.conf.apply(wbc,    ["default", "position_force_control"], true)
    Orocos.conf.apply(solver, ["default"], true)
    Orocos.conf.apply(joint_driver, ["default"], true)
    Orocos.conf.apply(force_sensor, ["default"], true)
    Orocos.conf.apply(cartesian_controller,  ["default"], true)
    Orocos.conf.apply(force_controller, ["default"], true)

    # Note: WBC will create dynamic ports for the constraints on configuration, so configure already here
    wbc.configure
    solver.configure
    cartesian_controller.configure
    force_controller.configure
    joint_driver.configure
    force_sensor.configure

    # Connect ports

    # Connect WBC with your robot's joint interface. In this case the joint driver
    joint_driver.port("joint_state").connect_to wbc.port("joint_state")
    solver.port("solver_output").connect_to joint_driver.port("command")
    wbc.port("constraints_prio").connect_to solver.port("constraints_prio")
    solver.port("solver_output").connect_to wbc.port("solver_output")

    # Priority 0:

    # Cartesian Position control
    constraint_name = "cart_position_ctrl_right"
    cartesian_controller.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    wbc.port("pose_" + constraint_name).connect_to cartesian_controller.port("feedback")

    # Cartesian Force Control
    constraint_name = "cart_force_ctrl_right"
    force_controller.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    force_controller.port("activation").connect_to wbc.port("weight_" + constraint_name)
    force_sensor.port("wrench").connect_to force_controller.port("feedback")

    # Run

    joint_driver.start
    cartesian_controller.start
    force_controller.start
    force_sensor.start
    wbc.start
    solver.start

    Readline.readline("Press Enter to start motion")

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
