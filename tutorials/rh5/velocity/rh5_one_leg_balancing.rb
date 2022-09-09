require 'rock/bundle'
require 'vizkit'
require 'readline'

Bundles.initialize

com_offset = 0.015
deployments = ["wbc_qp", "wbc_controllers", "trajectories"]
deployments << {"raisim::Task" => "raisim"} if not real_robot

Bundles.run , :output => nil do

    cart_ctrl_right    = Orocos::TaskContext.get "cart_ctrl_right"
    cart_ctrl_left    = Orocos::TaskContext.get "cart_ctrl_left"
    com_ctrl           = Orocos::TaskContext.get "cart_ctrl"
    wbc                = Orocos::TaskContext.get "wbc_qp"
    trajectory_com     = Orocos::TaskContext.get "trajectory"
    trajectory_right   = Orocos::TaskContext.get "trajectory_right"
    trajectory_left    = Orocos::TaskContext.get "trajectory_left"
    robot_world_pose = nil
    if use_mocap
        robot_world_pose = Orocos::TaskContext.get("mocap_rigid_body_pose").port("rh5_floating_base")
    else
        #robot_world_pose = Orocos::TaskContext.get("rh5_pose_estimation").port("pose_body_estimated")
        robot_world_pose = Orocos::TaskContext.get("raisim").port("base_pose")
    end
    robot_joint_state = Orocos::TaskContext.get("rh5_dispatcher").port("joint_state")

    Orocos.conf.apply(wbc,                ["default", "rh5_one_leg_balancing"])
    Orocos.conf.apply(cart_ctrl_right,    ["default", "cart_pos_ctrl"])
    Orocos.conf.apply(cart_ctrl_left,     ["default", "cart_pos_ctrl"])
    Orocos.conf.apply(com_ctrl,           ["default", "com_pos_ctrl"])
    Orocos.conf.apply(trajectory_com,     ["com_ctrl"])
    Orocos.conf.apply(trajectory_left,    ["com_ctrl"])
    Orocos.conf.apply(trajectory_right,   ["com_ctrl"])

    # Note: WBC will create dynamic ports for the constraints at configuration time, so configure already here
    wbc.configure
    cart_ctrl_right.configure
    cart_ctrl_left.configure
    com_ctrl.configure
    trajectory_left.configure
    trajectory_right.configure
    trajectory_com.configure

    # Priority 0: COM Position control
    constraint_name = "com_position"
    com_ctrl.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    wbc.port("com").connect_to com_ctrl.port("feedback")
    trajectory_com.port("command").connect_to com_ctrl.port("setpoint")
    com_ctrl.port("current_feedback").connect_to trajectory_com.port("cartesian_state")

    # Priority 0: Right leg control
    constraint_name = "right_leg_position"
    cart_ctrl_right.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    wbc.port("status_" + constraint_name).connect_to cart_ctrl_right.port("feedback")
    trajectory_right.port("command").connect_to cart_ctrl_right.port("setpoint")
    cart_ctrl_right.port("current_feedback").connect_to trajectory_right.port("cartesian_state")

    # Priority 0: Left leg control
    constraint_name = "left_leg_position"
    cart_ctrl_left.port("control_output").connect_to wbc.port("ref_" + constraint_name)
    wbc.port("status_" + constraint_name).connect_to cart_ctrl_left.port("feedback")
    trajectory_left.port("command").connect_to cart_ctrl_left.port("setpoint")
    cart_ctrl_left.port("current_feedback").connect_to trajectory_left.port("cartesian_state")

    # Connect to driver
    robot_world_pose.connect_to wbc.port("floating_base_state_deprecated")
    wbc.port("solver_output").connect_to Orocos::TaskContext.get("rh5_dispatcher").port("command")
    Orocos::TaskContext.get("rh5_dispatcher").port("joint_state").connect_to wbc.port("joint_state")
    #wbc.port("solver_output").connect_to Orocos::TaskContext.get("rh5_hyrodyn_client").port("joint_status")
    robot_joint_state.connect_to wbc.port("joint_state")

    # Run
    cart_ctrl_right.start
    cart_ctrl_left.start
    com_ctrl.start
    wbc.start
    trajectory_left.start
    trajectory_right.start
    trajectory_com.start

    # Read the right and left foot frames position
    reader_right = Orocos::TaskContext.get("cart_ctrl_right").port("current_feedback").reader
    sleep(0.01)
    pose_right_foot = reader_right.read
    fr_x = pose_right_foot.pose.position.x()
    fr_y = pose_right_foot.pose.position.y()
    fr_z = pose_right_foot.pose.position.z()
    puts "pose_right_foot"
    puts fr_x, fr_y, fr_z

    reader_left = Orocos::TaskContext.get("cart_ctrl_left").port("current_feedback").reader
    sleep(0.01)
    pose_left_foot = reader_left.read
    fl_x = pose_left_foot.pose.position.x()
    fl_y = pose_left_foot.pose.position.y()
    fl_z = pose_left_foot.pose.position.z()
    puts "pose_left_foot"
    puts fl_x, fl_y, fl_z

    Readline.readline("Press Enter to move the COM to the support leg")

    writer_com = trajectory_com.port("target").writer
    com_target = Types.base.samples.RigidBodyState.new
    #com_target.position    = Types.base.Vector3d.new(0.135,0.14,1.090)
    #com_target.position    = Types.base.Vector3d.new(0.140,0.1425,1.05)
    if lift_left
      com_target.position    = Types.base.Vector3d.new(fr_x,fr_y-com_offset,1.07)
    else
      com_target.position    = Types.base.Vector3d.new(fl_x,fl_y+com_offset,1.07)
    end
    com_target.orientation = Types.base.Quaterniond.new(1,0,0,0)
    com_target.velocity = Types.base.Vector3d.new(0,0,0)
    com_target.angular_velocity = Types.base.Vector3d.new(0,0,0)
    writer_com.write(com_target)

    reader_com = wbc.port("com").reader
    sample = nil
    distance = 1e10
    while distance > 0.005
        sample = reader_com.read_new
        if sample != nil
            distance = (com_target.position - sample.pose.position).norm
        end
    end

    Readline.readline("Press Enter to remove foot contact")

    # Read the right and left foot frames position
    reader_right = Orocos::TaskContext.get("cart_ctrl_right").port("current_feedback").reader
    sleep(1)
    pose_right_foot = reader_right.read
    fr_x = pose_right_foot.pose.position.x()
    fr_y = pose_right_foot.pose.position.y()
    fr_z = pose_right_foot.pose.position.z()
    puts "pose_right_foot"
    puts fr_x, fr_y, fr_z

    reader_left = Orocos::TaskContext.get("cart_ctrl_left").port("current_feedback").reader
    sleep(1)
    pose_left_foot = reader_left.read
    fl_x = pose_left_foot.pose.position.x()
    fl_y = pose_left_foot.pose.position.y()
    fl_z = pose_left_foot.pose.position.z()
    puts "pose_left_foot"
    puts fl_x, fl_y, fl_z

    if lift_left
      writer_left_leg = trajectory_left.port("target").writer
      left_leg_target = Types.base.samples.RigidBodyState.new
      left_leg_target.position    = Types.base.Vector3d.new(fl_x,fl_y, fl_z)
      left_leg_target.orientation = Types.base.Quaterniond.new(1,0,0,0)
      left_leg_target.velocity = Types.base.Vector3d.new(0,0,0)
      left_leg_target.angular_velocity = Types.base.Vector3d.new(0,0,0)
      writer_left_leg.write(left_leg_target)
    else
      writer_right_leg = trajectory_right.port("target").writer
      right_leg_target = Types.base.samples.RigidBodyState.new
      #right_leg_target.position    = Types.base.Vector3d.new(0.13,-0.03,-0.025)
      right_leg_target.position    = Types.base.Vector3d.new(fr_x,fr_y, fr_z)
      right_leg_target.orientation = Types.base.Quaterniond.new(1,0,0,0)
      right_leg_target.velocity = Types.base.Vector3d.new(0,0,0)
      right_leg_target.angular_velocity = Types.base.Vector3d.new(0,0,0)
      writer_right_leg.write(right_leg_target)
    end

    writer_contacts = wbc.port("active_contacts").writer
    contacts = Types.wbc.ActiveContacts.new
    contacts.names = ["LLAnkle_FT", "LRAnkle_FT"]

    if lift_left
      contacts.elements = [0,1]
    else
      contacts.elements = [1,0]
    end
    writer_contacts.write contacts

    Readline.readline("Press Enter to lift up leg")
    if lift_left
      left_leg_target.position = Types.base.Vector3d.new(fl_x,fl_y, fl_z+0.1)
      writer_left_leg.write(left_leg_target)
    else
      #right_leg_target.position = Types.base.Vector3d.new(0.13,-0.03,0.1)
      right_leg_target.position = Types.base.Vector3d.new(fr_x,fr_y, fr_z+0.1)
      writer_right_leg.write(right_leg_target)
    end


    Readline.readline("Press Enter to place leg back on the floor")
    if lift_left
      left_leg_target.position = Types.base.Vector3d.new(fl_x,fl_y, fl_z)
      writer_left_leg.write(left_leg_target)
    else
      #right_leg_target.position = Types.base.Vector3d.new(0.13,-0.03,0.1)
      right_leg_target.position = Types.base.Vector3d.new(fr_x,fr_y, fr_z)
      writer_right_leg.write(right_leg_target)
    end


    Readline.readline("Press Enter to exit")

end
