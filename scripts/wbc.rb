#!/usr/bin/env ruby

require 'pry'
require 'orocos'
require 'rock/bundle'

Orocos::CORBA.max_message_size = 8400000
Bundles.initialize

Orocos.transformer.load_conf('transforms.rb')

with_ft = false
with_head = false

Bundles.run  'aila_wbc', 'aila_wbc_dynamics', "robot_frames::ChainPublisher" => "servoing_generator" do 
    servoing_generator = Orocos::TaskContext.get 'servoing_generator'
    Orocos.conf.apply(servoing_generator, ['default', 'servoing_generation'])
    servoing_generator.configure
    servoing_generator.start

    right_setpoint_proxy = Orocos::TaskContext.get "aila_right_setpoint_proxy"
    right_setpoint_proxy.start
    left_setpoint_proxy = Orocos::TaskContext.get "aila_left_setpoint_proxy"
    left_setpoint_proxy.start
    body_setpoint_proxy = Orocos::TaskContext.get "aila_body_setpoint_proxy"
    body_setpoint_proxy.start
    keep_hands_parallel_proxy = Orocos::TaskContext.get "aila_keep_hands_parallel_proxy"
    keep_hands_parallel_proxy.start
    if with_head
       head_setpoint_proxy = Orocos::TaskContext.get "aila_head_setpoint_proxy"
       head_setpoint_proxy.start
    end
    
    wbc = Orocos::TaskContext.get 'aila_wbc'
    solver = Orocos::TaskContext.get 'aila_wbc_solver'
    robot_model = Orocos::TaskContext.get 'aila_wbc_robot_model'
    self_collision = Orocos::TaskContext.get 'aila_self_collision_ctrl'
    interpolator = Orocos::TaskContext.get 'aila_interpolation_velocity'
    dispatcher = Orocos::TaskContext.get 'aila_dispatcher'
    cart_pos_ctrl_right_arm = Orocos::TaskContext.get 'aila_cart_pos_ctrl_right_arm'
    cart_pos_ctrl_left_arm = Orocos::TaskContext.get 'aila_cart_pos_ctrl_left_arm'
    body_posture_ctrl = Orocos::TaskContext.get 'aila_body_posture_ctrl'
    keep_parallel_ctrl = Orocos::TaskContext.get 'aila_keep_hands_parallel_ctrl'
    manipulability_ctrl_r = Orocos::TaskContext.get 'aila_manipulability_ctrl_r'
    manipulability_ctrl_l = Orocos::TaskContext.get 'aila_manipulability_ctrl_l'
    if with_head
       head_ctrl = Orocos::TaskContext.get 'aila_head_ctrl'
    end
    joint_pos_ctrl = Orocos::TaskContext.get 'aila_joint_pos_ctrl'
    joint_limit_avoidance = Orocos::TaskContext.get 'aila_joint_limit_avoidance'
    self_collision_avoidance = Orocos::TaskContext.get 'aila_self_collision_avoidance'
    self_collision_check = Orocos::TaskContext.get 'aila_self_collision_check'
    if with_ft
        ft_proc_right = Orocos.name_service.get 'aila_right_hand_offset_comp'
        ft_proc_left = Orocos.name_service.get 'aila_left_hand_offset_comp'
        ft_ctrl_right = Orocos.name_service.get 'aila_force_ctrl_right_arm'
        ft_ctrl_left = Orocos.name_service.get 'aila_force_ctrl_left_arm'
    end

    Orocos.conf.apply( wbc, ['dynamics'], true )
    Orocos.conf.apply( solver, ['default'], true )
    Orocos.conf.apply( robot_model, ['default'], true )
    Orocos.conf.apply( cart_pos_ctrl_right_arm, ['cart_pos_ctrl_right_arm'], true )
    Orocos.conf.apply( cart_pos_ctrl_left_arm, ['cart_pos_ctrl_left_arm'], true )
    Orocos.conf.apply( body_posture_ctrl, ['body_posture_ctrl'], true )
    Orocos.conf.apply( keep_parallel_ctrl, ['keep_hands_parallel_ctrl'], true )
    Orocos.conf.apply( manipulability_ctrl_r, ['default', 'right_arm'], true )
    Orocos.conf.apply( manipulability_ctrl_l, ['default', 'left_arm'], true )
    if with_head
       Orocos.conf.apply( head_ctrl, ['aila_head_ctrl'], true )
    end
    Orocos.conf.apply( joint_pos_ctrl, ['no_hands_but_wrists'], true )
    Orocos.conf.apply( joint_limit_avoidance, ['no_hands_but_wrists'], true )
    Orocos.conf.apply( self_collision_avoidance, ['default'], true ) 
    if with_ft
        Orocos.conf.apply(ft_ctrl_right , ['default', 'force_ctrl_right_arm'])
        Orocos.conf.apply(ft_ctrl_left , ['default', 'force_ctrl_left_arm'])
    end

    # Configure here to create ports
    wbc.configure 
    
    Orocos.transformer.setup(cart_pos_ctrl_right_arm)  
    Orocos.transformer.setup(cart_pos_ctrl_left_arm)  
    Orocos.transformer.setup(body_posture_ctrl)  
    Orocos.transformer.setup(keep_parallel_ctrl)  
    if with_head
       Orocos.transformer.setup(head_ctrl)  
    end
    if with_ft
        Orocos.transformer.setup(ft_ctrl_right) 
        Orocos.transformer.setup(ft_ctrl_left)
    end
  
  
    # WBC and controllers

    # Force Control - prio 0
    if with_ft
        ft_ctrl_right.port("wrench").connect_to ft_proc_right.port("external_wrench_filtered")
        ft_ctrl_left.port("wrench").connect_to ft_proc_left.port("external_wrench_filtered")
        ft_ctrl_right.port("ctrl_out").connect_to wbc.port("ref_p0_cart_force_ctrl_right_arm")
        ft_ctrl_right.port("activation").connect_to wbc.port("weight_p0_cart_force_ctrl_right_arm")
        ft_ctrl_left.port("ctrl_out").connect_to wbc.port("ref_p0_cart_force_ctrl_left_arm")
        ft_ctrl_left.port("activation").connect_to wbc.port("weight_p0_cart_force_ctrl_left_arm")
    end
    
    # Cartesian Control - prio 1
    wbc.port("ref_p0_cart_cart_pos_ctrl_right_arm").connect_to cart_pos_ctrl_right_arm.port("ctrl_out")
    wbc.port("ref_p1_cart_cart_pos_ctrl_left_arm").connect_to cart_pos_ctrl_left_arm.port("ctrl_out")
    
    # Keep Parallel Ctrl - prio 1
    wbc.port("ref_p1_cart_keep_hands_parallel_ctrl").connect_to keep_parallel_ctrl.port("ctrl_out")
    
    # Head Control - prio 1
    if with_head
       wbc.port("ref_p1_jnt_head_ctrl").connect_to head_ctrl.port("ctrl_out")
       dispatcher.port("all_joint_state").connect_to head_ctrl.port("joint_state")
    end

    # Self Collision Avoidance - prio 1
    self_collision_avoidance.port("collision_info").connect_to self_collision_check.port("collision_info")
    wbc.port("ref_p0_jnt_self_collision_avoidance").connect_to self_collision_avoidance.port("ctrl_out")
    self_collision_avoidance.port("activation").connect_to wbc.port("weight_p0_jnt_self_collision_avoidance")
    
    # Joint Position Control - prio 2
    joint_pos_ctrl.port("feedback").connect_to dispatcher.port("bodywithhead_joint_state")
    wbc.port("ref_p2_jnt_joint_pos_ctrl").connect_to joint_pos_ctrl.port("ctrl_out")
    
    # Joint Limits Avoidance - prio 3
    joint_limit_avoidance.port("feedback").connect_to dispatcher.port("bodywithhead_joint_state")
    wbc.port("ref_p3_jnt_joint_limit_avoidance").connect_to joint_limit_avoidance.port("ctrl_out")
    wbc.port("weight_p3_jnt_joint_limit_avoidance").connect_to joint_limit_avoidance.port("activation")
    
    # Manipulability Control - prio 4
    manipulability_ctrl_r.port("joint_state").connect_to dispatcher.port("bodywithhead_joint_state")
    manipulability_ctrl_l.port("joint_state").connect_to dispatcher.port("bodywithhead_joint_state")
    manipulability_ctrl_r.port("ctrl_out").connect_to wbc.port("ref_p4_jnt_manipulability_ctrl_r")
    manipulability_ctrl_l.port("ctrl_out").connect_to wbc.port("ref_p4_jnt_manipulability_ctrl_l")

    # Body Posture Control - prio 5
    wbc.port("ref_p5_cart_body_posture_ctrl").connect_to body_posture_ctrl.port("ctrl_out")
    
    wbc.port("joint_state").connect_to dispatcher.port("bodywithhead_joint_state")
    wbc.port("solver_input").connect_to solver.port("solver_input")
    wbc.port("solver_output").connect_to solver.port("ctrl_out")
    solver.port("ctrl_out").connect_to interpolator.port("velocity_target")
    robot_model.port("joint_state").connect_to dispatcher.port("bodywithhead_joint_state")
    robot_model.port("task_frames").connect_to wbc.port("task_frames")
    
    # Head
    if with_head 
       dispatcher.port("bodywithhead_joint_state").connect_to servoing_generator.port('input')
    end
     
    if with_ft 
        ft_ctrl_left.configure
        ft_ctrl_right.configure
    end
    cart_pos_ctrl_right_arm.configure 
    keep_parallel_ctrl.configure
    cart_pos_ctrl_left_arm.configure 
    body_posture_ctrl.configure 
    if with_head
       head_ctrl.configure 
    end
    joint_pos_ctrl.configure
    joint_limit_avoidance.configure
    self_collision_avoidance.configure
    manipulability_ctrl_r.configure
    manipulability_ctrl_l.configure
    solver.configure
    robot_model.configure
  
    if with_ft
        ft_ctrl_right.start
        ft_ctrl_left.start
    end
    cart_pos_ctrl_right_arm.start
    cart_pos_ctrl_left_arm.start
    body_posture_ctrl.start     
    keep_parallel_ctrl.start
    if with_head
       head_ctrl.start 
    end
    joint_pos_ctrl.start
    joint_limit_avoidance.start
    self_collision_avoidance.start
    manipulability_ctrl_r.start
    manipulability_ctrl_l.start
    solver.start
    robot_model.start
    wbc.start

    Orocos.log_all
   
    puts "Press ENTER to stop!"
    STDIN.readline
end
