require 'orocos'

#
# For this example to run, you will need to install orogen/ctrl_lib and orogen/joint_control!
#
# If you want to have a visualization of the robot, checkout the kuka_lbr bundle, install rock-gui/gui-robot_model
# and type
#
#     rock-roboviz <kuka_bundle_path>/data/urdf/kuka_lbr.urdf -s driver:joint_state
#

Orocos.initialize

Orocos.load_typekit("base")
Orocos.load_typekit("wbc")
Orocos.load_typekit("ctrl_lib")

Orocos.run "wbc::WbcVelocityTask" => "wbc",
           "ctrl_lib::CartesianPositionController" => "controller",
           "joint_control::FakeJointDriverTask" => "driver" do

    controller = Orocos::TaskContext.get "controller"
    wbc        = Orocos::TaskContext.get "wbc"
    driver     = Orocos::TaskContext.get "driver"

    # Create robot model configs
    #

    robot_config  = Types::Wbc::RobotModelConfig.new
    robot_config.file = ENV["AUTOPROJ_CURRENT_ROOT"] + "/control/orogen/wbc/examples/data/kuka_lbr.urdf"

    object_config = Types::Wbc::RobotModelConfig.new
    object_config.file = ENV["AUTOPROJ_CURRENT_ROOT"] + "/control/orogen/wbc/examples/data/object.urdf"
    object_config.hook = "kuka_lbr_top_left_camera"
    object_config.initial_pose.position    = Types::Base::Vector3d.new(0,0,0.3)
    object_config.initial_pose.orientation = Types::Base::Quaterniond.new(1,0,0,0)

    wbc.robot_models = [robot_config, object_config]

    # Create Wbc config
    #

    cart_config = Types::Wbc::ConstraintConfig.new
    cart_config.name       = "cart_pos_ctrl"
    cart_config.type       = :cart
    cart_config.root       = "kuka_lbr_base"
    cart_config.tip        = "kuka_lbr_r_tcp"
    cart_config.ref_frame  = "kuka_lbr_base"
    cart_config.timeout    = 0
    cart_config.priority   = 0
    cart_config.activation = 1
    cart_config.weights    = [1,1,1,1,1,1]

    elbow_config = Types::Wbc::ConstraintConfig.new
    elbow_config.name        = "elbow_pos_ctrl"
    elbow_config.type        = :jnt
    elbow_config.joint_names = ["kuka_lbr_r_joint_3"]
    elbow_config.timeout    = 0
    elbow_config.priority   = 2
    elbow_config.activation = 1
    elbow_config.weights    = [1]

    elbow_config2 = Types::Wbc::ConstraintConfig.new
    elbow_config2.name        = "elbow_pos_ctrl2"
    elbow_config2.type        = :jnt
    elbow_config2.joint_names = ["kuka_lbr_r_joint_3"]
    elbow_config2.timeout    = 0
    elbow_config2.priority   = 0
    elbow_config2.activation = 1
    elbow_config2.weights    = [1]

    wbc.wbc_config            = [cart_config, elbow_config, elbow_config2]
    wbc.base_frame            = "kuka_lbr_base"
    wbc.initial_joint_weights = Types::Base::VectorXd.from_a [1]*7

    wbc.configure

    # Configure controller
    #

    controller.field_names         = ["X", "Y", "Z", "rotX", "rotY", "rotZ"]
    controller.prop_gain           = Types::Base::VectorXd.from_a [1]*6
    controller.max_control_output  = Types::Base::VectorXd.from_a [0.1]*6
    controller.dead_zone           = Types::Base::VectorXd.from_a [0]*6
    controller.ff_gain             = Types::Base::VectorXd.from_a [0]*6
    act_func = Types::CtrlLib::ActivationFunction.new
    act_func.type = :NO_ACTIVATION
    controller.activation_function = act_func

    controller.configure

    # Configure fake driver
    #

    driver.cycle_time = 0.01
    driver.urdf       = robot_config.file
    driver.meas_noise_std_dev = 1e-4
    driver.control_mode = "velocity"

    driver.configure

    # Connect ports
    #

    controller.port("control_output").connect_to wbc.port("ref_" + cart_config.name)

    wbc.port("state_" + cart_config.name ).connect_to controller.port("feedback")
    wbc.port("ctrl_out").connect_to driver.port("command")
    driver.port("joint_state").connect_to wbc.port("joint_state")

    # Run Control loop
    #

    driver.start
    controller.start
    wbc.start

    Readline.readline("Press Enter to start")

    target_pose = Types::Base::Samples::RigidBodyState.new
    target_pose.position = Types::Base::Vector3d.new(0.22, -1.44, 0.97)
    target_pose.orientation = Types::Base::Quaterniond.from_euler(Types::Base::Vector3d.new(1.71, -1.57, -0.14), 2,1,0)

    writer = controller.port("setpoint").writer
    writer.write(target_pose)

    reached = false
    reader = controller.port("current_feedback").reader
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
