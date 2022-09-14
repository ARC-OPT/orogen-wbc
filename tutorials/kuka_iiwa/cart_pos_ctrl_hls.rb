#
# Simple Velocity-based example, Cartesian position control on a kuka iiwa 7 dof arm. The solution is computed using the hierarchical least
# squares solver (hls), which provides an analytical solution.
# The robot end effector is supposed to move on a sinusoidal curve in z-direction. As there is only one task, the soluton degrades to a simple pseudo inversion / linear least squares.
# The task weights are all set to 1 initially, i.e., all task variables are treated with equal priority. A 7-dof arm is redundant by 1 dof when considering the task of, e.g.,
# controlling the full pose (position and orientation) of a robot's end effector. In principle, the number of dof can be arbitrary in WBC, as redundancy resolution is automatically handled by the solver.
#
require 'vizkit'
require 'readline'

Orocos.initialize
Orocos.conf.load_dir('config')
log_dir = "../logs"
Dir.mkdir log_dir  unless File.exists?(log_dir)
Orocos.default_working_directory = log_dir

Orocos.run "wbc::WbcVelocityTask"                  => "kuka_iiwa_wbc",
           "wbc::LoopBackDriver"                   => "kuka_iiwa_joints",
           "ctrl_lib::CartesianPositionController" => "kuka_iiwa_controller" do

    wbc          = Orocos::TaskContext.get "kuka_iiwa_wbc"
    joints       = Orocos::TaskContext.get "kuka_iiwa_joints"
    controller   = Orocos::TaskContext.get "kuka_iiwa_controller"

    Orocos.conf.apply(wbc,        ["default", "cart_pos_ctrl_hls"])
    Orocos.conf.apply(joints,     ["kuka_iiwa"])
    Orocos.conf.apply(controller, ["cart_ctrl"])

    # Note: WBC will create dynamic ports for the tasks at configuration time, so configure already here
    wbc.configure
    joints.configure
    controller.configure

    # Connect WBC to robot joints
    joints.port("joint_state").connect_to wbc.port("joint_state")
    wbc.port("solver_output").connect_to joints.port("command")

    # Priority 0: Cartesian Position control
    task_name = wbc.wbc_config[0].name
    controller.port("control_output").connect_to wbc.port("ref_" + task_name)
    wbc.port("status_" + task_name).connect_to controller.port("feedback")

    # Run
    joints.start
    wbc.start
    controller.start

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
