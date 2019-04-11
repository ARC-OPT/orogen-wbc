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

Orocos.run "wbc::WbcVelocityTask"                  => "wbc",
           "wbc_solvers::HierarchicalLSSolverTask" => "solver",
           "ctrl_lib::CartesianPositionController" => "cartesian_controller",
           "ctrl_lib::JointLimitAvoidance"         => "joint_limit_avoidance",
           "joint_control::FakeJointDriverTask"    => "joint_driver" do

    cartesian_controller  = Orocos::TaskContext.get "cartesian_controller"
    joint_limit_avoidance = Orocos::TaskContext.get "joint_limit_avoidance"
    wbc                   = Orocos::TaskContext.get "wbc"
    solver                = Orocos::TaskContext.get "solver"
    joint_driver          = Orocos::TaskContext.get "joint_driver"

    Orocos.conf.apply(wbc,    ["default", "cart_control_with_joint_limits"], true)
    Orocos.conf.apply(solver, ["default"], true)
    Orocos.conf.apply(joint_driver, ["default"], true)
    Orocos.conf.apply(cartesian_controller,  ["default"], true)
    Orocos.conf.apply(joint_limit_avoidance, ["default"], true)

    # Note: WBC will create dynamic ports for the constraints on configuration, so configure already here
    wbc.configure
    solver.configure
    cartesian_controller.configure
    joint_limit_avoidance.configure
    joint_driver.configure

    # Connect ports

    # Connect WBC with your robot's joint interface. In this case the joint driver
    joint_driver.port("joint_state").connect_to wbc.port("joint_state")
    solver.port("solver_output").connect_to joint_driver.port("command")
    wbc.port("hierarchical_qp").connect_to solver.port("hierarchical_qp")
    solver.port("solver_output").connect_to wbc.port("solver_output")

    # Priority 0:

    # Cartesian Position control
    constraint_name = "cart_position_ctrl_right"
    cartesian_controller.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    wbc.port("status_" + constraint_name).connect_to cartesian_controller.port("feedback")

    # Priority 1:

    # Joint Limit avoidance
    constraint_name = "joint_limit_avoidance"
    joint_limit_avoidance.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    joint_limit_avoidance.port("activation").connect_to wbc.port("weight_" + constraint_name)
    wbc.port("status_" + constraint_name).connect_to joint_limit_avoidance.port("feedback")

    # Run

    joint_driver.start
    cartesian_controller.start
    joint_limit_avoidance.start
    wbc.start
    solver.start

    Readline.readline("Press Enter to start motion")

    # Set target pose for Cartesian Controller
    target_pose = Types.wbc.CartesianState.new
    target_pose.pose.position = Types::Base::Vector3d.new(0.22, -1.44, 0.97)
    target_pose.pose.orientation = Types::Base::Quaterniond.from_euler(Types::Base::Vector3d.new(1.71, -1.57, -0.14), 2,1,0)
    pose_writer = cartesian_controller.port("setpoint").writer
    pose_writer.write(target_pose)

    # Check if current pose == target pose in a loop
    reached = false
    reader = cartesian_controller.port("current_feedback").reader
    reader_ctrl_out = solver.solver_output.reader
    feedback = Types.wbc.CartesianState.new
    while not reached
       feedback = reader.read
       solver_output = reader_ctrl_out.read
       if feedback && solver_output
          print "Target position:  "
          target_pose.pose.position.data.each do |v| print "#{'%.04f' % v} " end
          print "\nCurrent position: "
          feedback.pose.position.data.each do |v| print "#{'%.04f' % v} " end
          print "\nSolver output: "
          solver_output.elements.each do |v| print "#{'%.04f' % v.speed} " end
          print "\n\n"

          if (target_pose.pose.position - feedback.pose.position).norm < 1e-3
             reached = true
             puts "Reached Target Position!"
          end
       end
       sleep 0.1
    end

    Readline.readline("Press Enter to exit")
end
