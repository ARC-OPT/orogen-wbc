#
# Velocity-based example for a simple task hierarchy: Cartesian position control of the end effector on the lower priority
# and joint position control of joint 5 on the highest priority.
#
require 'orocos'
require 'readline'

Orocos.initialize
Orocos.conf.load_dir('config')

Orocos.run "wbc::WbcVelocityTask"                  => "kuka_iiwa_wbc",
           "ctrl_lib::CartesianPositionController" => "kuka_iiwa_cart_ctrl",
           "ctrl_lib::JointPositionController"     => "kuka_iiwa_jnt_ctrl" do

    cart_ctrl    = Orocos::TaskContext.get "kuka_iiwa_cart_ctrl"
    wbc          = Orocos::TaskContext.get "kuka_iiwa_wbc"
    jnt_ctrl     = Orocos::TaskContext.get "kuka_iiwa_jnt_ctrl"

    Orocos.conf.apply(wbc,        ["default", "kuka_iiwa", "kuka_iiwa_cart_pos_ctrl_hierarchies"])
    Orocos.conf.apply(cart_ctrl,  ["kuka_iiwa_cart_controller"])
    Orocos.conf.apply(jnt_ctrl,   ["kuka_iiwa_jnt_pos_ctrl"])

    # Note: WBC will create dynamic ports for the constraints at configuration time, so configure already here
    wbc.configure
    cart_ctrl.configure
    jnt_ctrl.configure

    # Priority 0: Cartesian Position control
    constraint_name = wbc.wbc_config[0].name
    jnt_ctrl.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    wbc.port("status_" + constraint_name).connect_to jnt_ctrl.port("feedback")

    # Priority 1: Cartesian Position control
    constraint_name = wbc.wbc_config[1].name
    cart_ctrl.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    wbc.port("status_" + constraint_name).connect_to cart_ctrl.port("feedback")

    # Run
    jnt_ctrl.start
    cart_ctrl.start
    wbc.start

    # Write initial joint state
    joint_state = Types.base.samples.Joints.new
    joint_state.names = ["kuka_lbr_l_joint_1", "kuka_lbr_l_joint_2", "kuka_lbr_l_joint_3", "kuka_lbr_l_joint_4", "kuka_lbr_l_joint_5", "kuka_lbr_l_joint_6", "kuka_lbr_l_joint_7"]
    joint_state.names.each do
       js = Types.base.JointState.new
       js.position = 0.1
       joint_state.elements << js
    end
    joint_state.time = Types.base.Time.now
    writer_joint_state = wbc.joint_state.writer
    writer_joint_state.write joint_state

    puts "\n----------------------------- Press Enter to start motion ---------------------------------"
    puts "NOTE: To enable visualization, type "
    puts "        'rock-roboviz models/kuka_iiwa/urdf/kuka_iiwa.urdf -s kuka_iiwa_wbc:solver_output'"
    puts "This requires that you install 'roboviz' by typing 'aup/amake gui/robot_model'."
    Readline.readline("------------------------------------------------------------------------------------------")

    # Set activation for constraint
    wbc.activateConstraint(constraint_name,1)

    # Set target pose for Joint Controller
    target_jnt_pos = Types.base.samples.Joints.new
    js = Types.base.JointState.new
    js.position  = 0.1
    target_jnt_pos.elements = [js]
    target_jnt_pos.names = ["kuka_lbr_l_joint_5"]
    jnt_pos_writer = jnt_ctrl.port("setpoint").writer
    jnt_pos_writer.write(target_jnt_pos)

    # Set target pose for Cartesian Controller
    target_pose = Types.base.samples.RigidBodyStateSE3.new
    target_pose.pose.position = Types.base.Vector3d.new(0,0,0.8)
    target_pose.pose.orientation = Types.base.Quaterniond.from_euler(Types.base.Vector3d.new(0,0,0), 2,1,0)
    pose_writer = cart_ctrl.port("setpoint").writer
    pose_writer.write(target_pose)

    # Check if current pose == target pose in a loop
    reader_feedback = cart_ctrl.port("current_feedback").reader
    reader_ctrl_out = wbc.solver_output.reader
    while true

       joint_state.time = Types.base.Time.now
       writer_joint_state.write joint_state

       feedback = reader_feedback.read
       solver_output = reader_ctrl_out.read

       if feedback && solver_output
          print "Target position:  "
          target_pose.pose.position.data.each do |v| print "#{'%.04f' % v} " end
          print "\nCurrent position: "
          feedback.pose.position.data.each do |v| print "#{'%.04f' % v} " end
          print "\nSolver output: "
          solver_output.elements.each do |v| print "#{'%.04f' % v.speed} " end
          print "\n\n"

          if (target_pose.pose.position - feedback.pose.position).norm < 1e-2
             puts "Reached Target Position!"
             break
          end

          joint_state.elements = solver_output.elements
       end
       sleep 0.01
    end

    Readline.readline("Press Enter to exit")
end
