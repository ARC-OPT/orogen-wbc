require 'rock/bundle'
require 'vizkit'

Orocos.initialize
Orocos.conf.load_dir('../config')
log_dir = "../../logs"
Dir.mkdir log_dir  unless File.exists?(log_dir)
Orocos.default_working_directory = log_dir

Orocos.run "wbc_controllers", "trajectories",
           "wbc::WbcVelocityQuadraticCostTask"               => "rh5v2_wbc",
           "raisim::Task"                                    => "rh5v2_raisim" do

    cart_ctrl_right  = Orocos::TaskContext.get "cart_ctrl_right"
    cart_ctrl_left   = Orocos::TaskContext.get "cart_ctrl_left"
    wbc              = Orocos::TaskContext.get "rh5v2_wbc"
    raisim           = Orocos::TaskContext.get "rh5v2_raisim"
    traj_right       = Orocos::TaskContext.get "trajectory_right"
    traj_left        = Orocos::TaskContext.get "trajectory_left"

    Orocos.conf.apply(wbc,             ["default", "rh5v2"])
    Orocos.conf.apply(raisim,          ["rh5v2"])
    Orocos.conf.apply(cart_ctrl_right, ["default", "arm_ctrl"])
    Orocos.conf.apply(cart_ctrl_left,  ["default", "arm_ctrl"])
    Orocos.conf.apply(traj_right,      ["arm_ctrl"])
    Orocos.conf.apply(traj_left,       ["arm_ctrl"])

    # Note: WBC will create dynamic ports for the constraints at configuration time, so configure already here
    wbc.configure
    raisim.configure
    cart_ctrl_right.configure
    cart_ctrl_left.configure
    traj_right.configure
    traj_left.configure

    # Priority 0: Cartesian Position control
    constraint_name = "cart_position_ctrl_right"
    cart_ctrl_right.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    wbc.port("status_" + constraint_name).connect_to cart_ctrl_right.port("feedback")
    traj_right.port("command").connect_to cart_ctrl_right.port("setpoint")
    cart_ctrl_right.port("current_feedback").connect_to traj_right.port("cartesian_state")

    constraint_name = "cart_position_ctrl_left"
    cart_ctrl_left.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    wbc.port("status_" + constraint_name).connect_to cart_ctrl_left.port("feedback")
    traj_left.port("command").connect_to cart_ctrl_left.port("setpoint")
    cart_ctrl_left.port("current_feedback").connect_to traj_left.port("cartesian_state")

    raisim.port("status_samples").connect_to wbc.port("joint_state")
    wbc.port("solver_output").connect_to raisim.port("command")

    # Run
    wbc.start
    raisim.start
    cart_ctrl_right.start
    cart_ctrl_left.start
    traj_right.start
    traj_left.start

    # Set target pose for Cartesian Controller: Fixed pose for left arm, waypoints for right arm
    target_pose_left = Types.base.samples.RigidBodyState.new
    target_pose_left.position = Types.base.Vector3d.new(0.5224617716695082, 0.4530745317541083, 0.18347208189138647)
    target_pose_left.orientation = Types.base.Quaterniond.new(0.37215647020420817, -0.4848711645310197, 0.7254688310517345, -0.31637713359834774)
    target_pose_left.velocity = Types.base.Vector3d.new(0,0,0)
    target_pose_left.angular_velocity = Types.base.Vector3d.new(0,0,0)
    pose_writer_left = traj_left.port("target").writer
    target_pose_left.time = Types.base.Time.now
    pose_writer_left.write(target_pose_left)

    target_pose_right = Types.base.samples.RigidBodyState.new
    target_pose_right.position = Types.base.Vector3d.new(0.5227778462609431, -0.4530455168766185, 0.18395383140492982)
    target_pose_right.orientation = Types.base.Quaterniond.new(0.3712945230258401, 0.48393782872237234, 0.7261301348554581, 0.3173004606950384)
    target_pose_right.velocity = Types.base.Vector3d.new(0,0,0)
    target_pose_right.angular_velocity = Types.base.Vector3d.new(0,0,0)
    pose_writer_right = traj_right.port("target").writer
    pose_writer_right.write(target_pose_right)

    timer = Qt::Timer.new
    sample_time = 2000.0 # ms
    sign = 1
    amplitude = 0.1
    timer.connect(SIGNAL('timeout()')) do
        target_pose_right.time = Types.base.Time.now
        target_pose_right.position[0] = 0.5227778462609431 + sign*amplitude
        pose_writer_right.write(target_pose_right)
        sign *= -1
    end
    timer.start(sample_time)

    # visualization
    vis_gui = Vizkit.default_loader.RobotVisualization
    vis_gui.modelFile = wbc.robot_model.file
    start_time = Types.base.Time.now
    raisim.port("status_samples").to_async.on_data do |sample|
        vis_gui.updateData(sample)
    end

    Vizkit.exec
end
