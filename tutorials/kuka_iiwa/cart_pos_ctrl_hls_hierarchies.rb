#
# Velocity-based example for a simple task hierarchy: Cartesian position control of the end effector on the lower priority
# and joint position control of joint 5 on the highest priority.
#
require 'vizkit'
require 'readline'

Orocos.initialize
Orocos.conf.load_dir('config')

Orocos.run "wbc::WbcVelocityTask"                  => "kuka_iiwa_wbc",
           "wbc::LoopBackDriver"                   => "kuka_iiwa_joints",
           "ctrl_lib::CartesianPositionController" => "kuka_iiwa_controller",
           "ctrl_lib::JointPositionController"     => "kuka_iiwa_jnt_ctrl" do

    joints       = Orocos::TaskContext.get "kuka_iiwa_joints"
    controller   = Orocos::TaskContext.get "kuka_iiwa_controller"
    wbc          = Orocos::TaskContext.get "kuka_iiwa_wbc"
    jnt_ctrl     = Orocos::TaskContext.get "kuka_iiwa_jnt_ctrl"

    Orocos.conf.apply(wbc,        ["default", "cart_pos_ctrl_hierarchies"])
    Orocos.conf.apply(joints,     ["kuka_iiwa"])
    Orocos.conf.apply(controller, ["cart_ctrl"])
    Orocos.conf.apply(jnt_ctrl,   ["jnt_pos_ctrl"])

    # Note: WBC will create dynamic ports for the constraints at configuration time, so configure already here
    wbc.configure
    joints.configure
    controller.configure
    jnt_ctrl.configure

    # Connect WBC to robot joints
    joints.port("joint_state").connect_to wbc.port("joint_state")
    wbc.port("solver_output").connect_to joints.port("command")

    # Priority 0: Cartesian Position control
    constraint_name = wbc.wbc_config[0].name
    jnt_ctrl.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    wbc.port("status_" + constraint_name).connect_to jnt_ctrl.port("feedback")

    # Priority 1: Cartesian Position control
    constraint_name = wbc.wbc_config[1].name
    controller.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    wbc.port("status_" + constraint_name).connect_to controller.port("feedback")

    # Run
    jnt_ctrl.start
    joints.start
    controller.start
    wbc.start

    # Set activation for constraints
    wbc.activateConstraint(wbc.wbc_config[0].name,1)
    wbc.activateConstraint(wbc.wbc_config[1].name,1)

    # Set target pose for Joint Controller
    target_jnt_pos = Types.base.samples.Joints.new
    js = Types.base.JointState.new
    js.position  = 1.0
    target_jnt_pos.elements = [js]
    target_jnt_pos.names = ["kuka_lbr_l_joint_6"]
    jnt_pos_writer = jnt_ctrl.port("setpoint").writer
    jnt_pos_writer.write(target_jnt_pos)

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
