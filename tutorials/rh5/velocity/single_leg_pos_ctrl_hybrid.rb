require 'rock/bundle'
require 'vizkit'

Orocos.initialize
Orocos.conf.load_dir('../config')

Orocos.run "wbc::WbcVelocityQuadraticCostTask"     => "rh5_wbc",
           "wbc::LoopBackDriver"                   => "rh5_joints",
           "ctrl_lib::CartesianPositionController" => "rh5_leg_ctrl",
           "hyrodyn::ForwardTask"                  => "rh5_hyrodyn_fk" do

    controller   = Orocos::TaskContext.get "rh5_leg_ctrl"
    wbc          = Orocos::TaskContext.get "rh5_wbc"
    joints       = Orocos::TaskContext.get "rh5_joints"
    hyrodyn      = Orocos::TaskContext.get "rh5_hyrodyn_fk"

    Orocos.conf.apply(wbc,         ["default", "rh5_single_leg_hybrid", "single_leg_ctrl"])
    Orocos.conf.apply(controller,  ["single_leg_cart_ctrl"])
    Orocos.conf.apply(joints,      ["rh5_single_leg_hybrid"])
    Orocos.conf.apply(hyrodyn,     ["rh5_single_leg_hybrid"])

    # Note: WBC will create dynamic ports for the constraints at configuration time, so configure already here
    wbc.configure
    controller.configure
    joints.configure
    hyrodyn.configure

    # Priority 0: Cartesian Position control
    constraint_name = "cart_position_ctrl"
    controller.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    wbc.port("status_" + constraint_name).connect_to controller.port("feedback")

    # Connect to joints
    joints.port("joint_state").connect_to wbc.port("joint_state")
    wbc.port("solver_output").connect_to hyrodyn.port("actuator_status")
    hyrodyn.port("joint_status").connect_to joints.port("command")

    # Run
    controller.start
    wbc.start
    joints.start
    hyrodyn.start

    # Set target pose for Cartesian Controller: A sinusoidal function
    target_pose = Types.base.samples.RigidBodyStateSE3.new
    target_pose.pose.position = Types.base.Vector3d.new(0,0,0.8)
    target_pose.pose.orientation = Types.base.Quaterniond.from_euler(Types.base.Vector3d.new(0,0,0), 2,1,0)
    target_pose.twist.angular  = Types.base.Vector3d.new(0,0,0)
    pose_writer = controller.port("setpoint").writer
    timer = Qt::Timer.new
    delta = 0.0
    timer.connect(SIGNAL('timeout()')) do
        target_pose.time = Types.base.Time.now
        target_pose.pose.position = Types.base.Vector3d.new(0,0,-0.7+0.1*Math.sin(delta)) # Position
        target_pose.twist.linear  = Types.base.Vector3d.new(0,0,0.1*Math.cos(delta))     # Feed forward velocity. This will improve trajectory tracking
        pose_writer.write(target_pose)
        delta += 0.1
    end
    timer.start(10)

    # visualization
    proxy = Orocos::Async.proxy "rh5_joints"
    vis_gui = Vizkit.default_loader.RobotVisualization
    vis_gui.modelFile = wbc.robot_model.file
    start_time = Types.base.Time.now
    proxy.port("joint_state").on_data do |sample|
        vis_gui.updateData(sample)
    end

    Vizkit.exec
end
