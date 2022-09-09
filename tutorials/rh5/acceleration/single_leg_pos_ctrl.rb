require 'rock/bundle'
require 'vizkit'

Orocos.initialize
Orocos.conf.load_dir('../config')

Orocos.run "wbc::WbcAccelerationTask"              => "rh5_wbc",
           "raisim::Task"                          => "rh5_raisim",
           "ctrl_lib::CartesianPositionController" => "rh5_leg_ctrl" do

    controller   = Orocos::TaskContext.get "rh5_leg_ctrl"
    wbc          = Orocos::TaskContext.get "rh5_wbc"
    raisim       = Orocos::TaskContext.get "rh5_raisim"

    Orocos.conf.apply(wbc,         ["default", "rh5_single_leg", "single_leg_ctrl"])
    Orocos.conf.apply(controller,  ["default", "single_leg_cart_ctrl_tsid"])
    Orocos.conf.apply(raisim,      ["default", "rh5_single_leg"])

    # Note: WBC will create dynamic ports for the constraints at configuration time, so configure already here
    wbc.configure
    controller.configure
    raisim.configure

    # Priority 0: Cartesian Position control
    constraint_name = "cart_position_ctrl"
    controller.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    wbc.port("status_" + constraint_name).connect_to controller.port("feedback")

    # Connect to driver
    raisim.port("status_samples").connect_to wbc.port("joint_state")
    wbc.port("solver_output").connect_to raisim.port("command")

    # Run
    controller.start
    #wbc.start
    raisim.start

    # Set target pose for Cartesian Controller
    target_pose = Types.base.samples.RigidBodyStateSE3.new
    target_pose.time = Types.base.Time.now
    target_pose.twist.linear  = Types.base.Vector3d.new(0,0,0.0)
    target_pose.acceleration.linear  = Types.base.Vector3d.new(0,0,0)
    target_pose.pose.orientation = Types.base.Quaterniond.from_euler(Types.base.Vector3d.new(0,0,0), 2,1,0)
    target_pose.twist.angular  = Types.base.Vector3d.new(0,0,0)
    target_pose.acceleration.angular  = Types.base.Vector3d.new(0,0,0)
    pose_writer = controller.port("setpoint").writer
    timer = Qt::Timer.new
    sigma = 1
    timer.connect(SIGNAL('timeout()')) do
        target_pose.time = Types.base.Time.now
        target_pose.pose.position = Types.base.Vector3d.new(0,0,-0.7+0.05*sigma)      # Position
        pose_writer.write(target_pose)
        sigma *= -1
    end
    timer.start(3000)

    # visualization
    proxy = Orocos::Async.proxy "rh5_raisim"
    vis_gui = Vizkit.default_loader.RobotVisualization
    vis_gui.modelFile = wbc.robot_model.file
    start_time = Types.base.Time.now
    proxy.port("status_samples").on_data do |sample|
        vis_gui.updateData(sample)
    end

    Vizkit.exec
end
