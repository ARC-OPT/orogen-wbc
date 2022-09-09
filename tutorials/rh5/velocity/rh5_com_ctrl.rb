require 'rock/bundle'
require 'vizkit'

Orocos.initialize
Orocos.conf.load_dir('../config')

Orocos.run "wbc::WbcVelocityQuadraticCostTask"     => "rh5_wbc",
           "raisim::Task"                          => "rh5_raisim",
           "ctrl_lib::CartesianPositionController" => "rh5_com_ctrl" do

    controller   = Orocos::TaskContext.get "rh5_com_ctrl"
    wbc          = Orocos::TaskContext.get "rh5_wbc"
    raisim       = Orocos::TaskContext.get "rh5_raisim"

    Orocos.conf.apply(wbc,         ["default", "rh5", "com_ctrl"])
    Orocos.conf.apply(controller,  ["default", "com_ctrl"])
    Orocos.conf.apply(raisim,      ["default", "rh5"])

    # Note: WBC will create dynamic ports for the constraints at configuration time, so configure already here
    wbc.configure
    controller.configure
    raisim.configure

    # Priority 0: Cartesian Position control
    constraint_name = "com_ctrl"
    controller.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    wbc.port("status_" + constraint_name).connect_to controller.port("feedback")

    # Connect to driver
    raisim.port("status_samples").connect_to wbc.port("joint_state")
    wbc.port("solver_output").connect_to raisim.port("command")
    raisim.port("base_pose").connect_to wbc.port("floating_base_state_deprecated")

    # Run
    controller.start
    wbc.start
    raisim.start

    # Set target pose for CoM Controller: A sinusoidal function
    target_pose = Types.base.samples.RigidBodyStateSE3.new
    target_pose.pose.orientation = Types.base.Quaterniond.from_euler(Types.base.Vector3d.new(0,0,0), 2,1,0)
    target_pose.twist.angular  = Types.base.Vector3d.new(0,0,0)
    pose_writer = controller.port("setpoint").writer
    timer = Qt::Timer.new
    delta = 0.0
    timer.connect(SIGNAL('timeout()')) do
        target_pose.time = Types.base.Time.now
        target_pose.pose.position = Types.base.Vector3d.new(0.1,0,1.05+0.03*Math.sin(delta)) # Position
        target_pose.twist.linear  = Types.base.Vector3d.new(0,0,0.03*Math.cos(delta))        # Feed forward velocity. This will improve trajectory tracking
        pose_writer.write(target_pose)
        delta += 0.05
    end
    timer.start(10)

    # visualization
    proxy = Orocos::Async.proxy "rh5_wbc"
    vis_gui = Vizkit.default_loader.RobotVisualization
    vis_gui.modelFile = ENV["AUTOPROJ_CURRENT_ROOT"] + "/control/wbc/models/rh5/urdf/rh5_floating_base.urdf"
    start_time = Types.base.Time.now
    proxy.port("full_joint_state").on_data do |sample|
        vis_gui.updateData(sample)
    end

    Vizkit.exec
end
