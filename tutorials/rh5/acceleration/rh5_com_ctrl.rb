require 'rock/bundle'
require 'vizkit'

Orocos.initialize
Orocos.conf.load_dir('../config')
log_dir = "../logs"
Dir.mkdir log_dir  unless File.exists?(log_dir)
Orocos.default_working_directory = log_dir

use_force_control = false

Orocos.run "wbc::WbcAccelerationTask"                        => "rh5_wbc",
           "raisim::Task"                                    => "rh5_raisim",
           "ctrl_lib::CartesianPositionController"           => "rh5_com_ctrl",
           "trajectory_generation::RMLCartesianPositionTask" => "rh5_trajectory",
           "ctrl_lib::JointPositionController"               => "rh5_joint_ctrl" do

    controller   = Orocos::TaskContext.get "rh5_com_ctrl"
    wbc          = Orocos::TaskContext.get "rh5_wbc"
    raisim       = Orocos::TaskContext.get "rh5_raisim"
    trajectory   = Orocos::TaskContext.get "rh5_trajectory"
    joint_ctrl   = Orocos::TaskContext.get "rh5_joint_ctrl"
    Orocos.conf.apply(wbc,         ["default", "rh5_legs", "com_ctrl"])
    if use_force_control
        Orocos.conf.apply(controller,  ["default", "com_ctrl_tsid_force_control"])
        Orocos.conf.apply(raisim,      ["default", "rh5_legs_force_control"])
    else
        Orocos.conf.apply(controller,  ["default", "com_ctrl_tsid"])
        Orocos.conf.apply(raisim,      ["default", "rh5_legs"])
    end
    Orocos.conf.apply(trajectory,  ["default", "com_ctrl"])
    Orocos.conf.apply(joint_ctrl,  ["default", "rh5_legs"])

    # Note: WBC will create dynamic ports for the constraints at configuration time, so configure already here
    wbc.configure
    controller.configure
    raisim.configure
    trajectory.configure
    joint_ctrl.configure

    # Priority 0: Cartesian Position control
    constraint_name = "com_ctrl"
    controller.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    wbc.port("status_" + constraint_name).connect_to controller.port("feedback")
    controller.port("current_feedback").connect_to trajectory.port("cartesian_state")
    trajectory.port("command").connect_to controller.port("setpoint")

    # constraint_name = "joint_ctrl"
    # joint_ctrl.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    # wbc.port("status_" + constraint_name).connect_to joint_ctrl.port("feedback")

    # Connect to driver
    raisim.port("status_samples").connect_to wbc.port("joint_state")
    #wbc.port("solver_output").connect_to raisim.port("command")
    raisim.port("base_pose").connect_to wbc.port("floating_base_state_deprecated")

    # Run
    controller.start
    raisim.start
    trajectory.start
    wbc.start
    joint_ctrl.start

    # Set target pose for Cartesian Controller
    target_pose = Types.base.samples.RigidBodyState.new
    target_pose.time = Types.base.Time.now
    target_pose.velocity  = Types.base.Vector3d.new(0,0,0.0)
    target_pose.orientation = Types.base.Quaterniond.from_euler(Types.base.Vector3d.new(0,0,0), 2,1,0)
    target_pose.angular_velocity  = Types.base.Vector3d.new(0,0,0)
    pose_writer = trajectory.port("target").writer
    timer = Qt::Timer.new
    sigma = 1
    #
    # start_pos = nil
    # reader = raisim.port("status_samples").reader
    # while true
    #     start_pos = reader.read_new
    #     if start_pos != nil
    #         start_pos.elements.each do |e|
    #             e.speed = 0
    #             e.acceleration = 0
    #         end
    #         break
    #     end
    # end
    # joint_pos_writer = joint_ctrl.port("setpoint").writer
    # joint_pos_writer.write(start_pos)

    timer.connect(SIGNAL('timeout()')) do
        target_pose.time = Types.base.Time.now
        target_pose.position = Types.base.Vector3d.new(0.0,0,0.87+0.02*sigma)
        pose_writer.write(target_pose)
        sigma *= -1
    end
    timer.start(2000)

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
