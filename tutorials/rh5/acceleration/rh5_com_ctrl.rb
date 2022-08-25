require 'rock/bundle'
require 'vizkit'

Orocos.initialize
Orocos.conf.load_dir('../config')
log_dir = "../logs"
Dir.mkdir log_dir  unless File.exists?(log_dir)
Orocos.default_working_directory = log_dir

Orocos.run "wbc::WbcAccelerationTask"                        => "rh5_wbc",
           "raisim::Task"                                    => "rh5_raisim",
           "ctrl_lib::CartesianPositionController"           => "rh5_com_ctrl",
           "trajectory_generation::RMLCartesianPositionTask" => "rh5_trajectory" do

    controller   = Orocos::TaskContext.get "rh5_com_ctrl"
    wbc          = Orocos::TaskContext.get "rh5_wbc"
    raisim       = Orocos::TaskContext.get "rh5_raisim"
    trajectory   = Orocos::TaskContext.get "rh5_trajectory"

    Orocos.conf.apply(wbc,         ["default", "rh5_legs", "com_ctrl"])
    Orocos.conf.apply(controller,  ["default", "com_ctrl_tsid"])
    Orocos.conf.apply(raisim,      ["default", "rh5_legs"])
    Orocos.conf.apply(trajectory,  ["default", "single_leg_ctrl"])

    # Note: WBC will create dynamic ports for the constraints at configuration time, so configure already here
    wbc.configure
    controller.configure
    raisim.configure
    trajectory.configure

    # Priority 0: Cartesian Position control
    constraint_name = "com_ctrl"
    controller.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    wbc.port("status_" + constraint_name).connect_to controller.port("feedback")
    controller.port("current_feedback").connect_to trajectory.port("cartesian_state")
    trajectory.port("command").connect_to controller.port("setpoint")

    # Connect to driver
    raisim.port("status_samples").connect_to wbc.port("joint_state")
    wbc.port("solver_output").connect_to raisim.port("command")
    raisim.port("base_pose").connect_to wbc.port("floating_base_state_deprecated")

    # Run
    controller.start
    raisim.start
    trajectory.start

    # Set target pose for Cartesian Controller
    target_pose = Types.base.samples.RigidBodyState.new
    target_pose.time = Types.base.Time.now
    target_pose.velocity  = Types.base.Vector3d.new(0,0,0.0)
    target_pose.orientation = Types.base.Quaterniond.from_euler(Types.base.Vector3d.new(0,0,0), 2,1,0)
    target_pose.angular_velocity  = Types.base.Vector3d.new(0,0,0)
    pose_writer = trajectory.port("target").writer
    timer = Qt::Timer.new
    sigma = 1
    target_pose.position = Types.base.Vector3d.new(0.0,0,0.8+0.03*sigma)      # Position
    pose_writer.write(target_pose)
    timer.connect(SIGNAL('timeout()')) do
        if not wbc.running?
            wbc.start
        end
        target_pose.time = Types.base.Time.now
        target_pose.position = Types.base.Vector3d.new(0.0,0,0.8+0.03*sigma)      # Position
        pose_writer.write(target_pose)
        sigma *= -1
    end
    timer.start(10000)

    # visualization
    proxy = Orocos::Async.proxy "rh5_wbc"
    vis_gui = Vizkit.default_loader.RobotVisualization
    vis_gui.modelFile = ENV["AUTOPROJ_CURRENT_ROOT"] + "/control/wbc/models/rh5/urdf/rh5_legs_floating_base.urdf"
    start_time = Types.base.Time.now
    proxy.port("full_joint_state").on_data do |sample|
        vis_gui.updateData(sample)
    end

    Vizkit.exec
end
