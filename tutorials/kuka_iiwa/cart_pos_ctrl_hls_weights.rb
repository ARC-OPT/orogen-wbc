#
# Same Velocity-based example as in tutorial 01. Only that the tasks weights are modified, so that the solution only considers the
# task space position, not the orientation.
#
require 'vizkit'
require 'readline'

Orocos.initialize
Orocos.conf.load_dir('config')

Orocos.run "wbc::WbcVelocityTask"                  => "kuka_iiwa_wbc",
           "wbc::LoopBackDriver"                   => "kuka_iiwa_joints",
           "ctrl_lib::CartesianPositionController" => "kuka_iiwa_controller" do

    joints       = Orocos::TaskContext.get "kuka_iiwa_joints"
    wbc          = Orocos::TaskContext.get "kuka_iiwa_wbc"
    controller   = Orocos::TaskContext.get "kuka_iiwa_controller"

    Orocos.conf.apply(wbc,         ["default", "cart_pos_ctrl_weights"])
    Orocos.conf.apply(joints,      ["kuka_iiwa"])
    Orocos.conf.apply(controller,  ["cart_ctrl"])

    # Note: WBC will create dynamic ports for the constraints at configuration time, so configure already here
    wbc.configure
    joints.configure
    controller.configure

    # Connect WBC to robot joints
    joints.port("joint_state").connect_to wbc.port("joint_state")
    wbc.port("solver_output").connect_to joints.port("command")

    # Priority 0: Cartesian Position control
    constraint_name = wbc.wbc_config[0].name
    controller.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    wbc.port("status_" + constraint_name).connect_to controller.port("feedback")

    # Run
    joints.start
    wbc.start
    controller.start

    # Set activation for constraint
    wbc.activateConstraint(constraint_name,1)

    # Set target pose for Cartesian Controller
    target_pose = Types.base.samples.RigidBodyStateSE3.new
    target_pose.pose.position = Types.base.Vector3d.new(0,0,0.8)
    target_pose.pose.orientation = Types.base.Quaterniond.from_euler(Types.base.Vector3d.new(0,0,0), 2,1,0)
    target_pose.twist.angular  = Types.base.Vector3d.new(0,0,0)
    pose_writer = controller.port("setpoint").writer
    timer = Qt::Timer.new
    delta = 0.0
    timer.connect(SIGNAL('timeout()')) do
        target_pose.time = Types.base.Time.now
        target_pose.pose.position = Types.base.Vector3d.new(0,0,0.9+0.1*Math.sin(delta)) # Position
        target_pose.twist.linear  = Types.base.Vector3d.new(0,0,0.1*Math.cos(delta))     # Feed forward velocity. This will improve trajectory tracking
        pose_writer.write(target_pose)
        delta += 0.2
    end
    timer.start(10)

    # visualization
    begin
        proxy = Orocos::Async.proxy "kuka_iiwa_wbc"
        vis_gui = Vizkit.default_loader.RobotVisualization
        vis_gui.modelFile = wbc.robot_model.file
        start_time = Types.base.Time.now
        proxy.port("full_joint_state").on_data do |sample|
            vis_gui.updateData(sample)
        end
    rescue
        puts "Loading the robot vizualization failed!"
        puts "To enable visualization, you have to install 'roboviz' as follows:"
        puts "     aup/amake gui/robot_model    "
    end

    Vizkit.exec
end
