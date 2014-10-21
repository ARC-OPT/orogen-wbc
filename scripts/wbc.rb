#!/usr/bin/env ruby

require 'pry'
require 'vizkit'

Orocos.initialize
Orocos.conf.load_dir('config')

Orocos.transformer.load_conf('config/transforms.rb')

Orocos.run 'wbc_dynamic' do 
   
   cart_pos_ctrl_right_proxy = Orocos::TaskContext.get "cart_pos_ctrl_right_proxy"
   cart_pos_ctrl_right_proxy.start

   robot_control = Orocos.name_service.get 'aila_self_collision_ctrl'    
   solver = Orocos.name_service.get 'wbc_solver'    
   wbc = Orocos.name_service.get 'wbc'
   robot_model = Orocos.name_service.get 'wbc_robot_model'
   dispatcher = Orocos.name_service.get 'aila_dispatcher'
   cart_pos_ctrl_right_arm = Orocos.name_service.get 'cart_pos_ctrl_right_arm'
   joint_limit_avoidance = Orocos.name_service.get 'joint_limit_avoidance'
   joint_pos_ctrl = Orocos.name_service.get 'joint_pos_ctrl'

   Orocos.conf.apply(solver, ['default'])
   Orocos.conf.apply(robot_model, ['default'])   
   Orocos.conf.apply(wbc, ['default'])
   Orocos.conf.apply(cart_pos_ctrl_right_arm, ['cart_pos_ctrl_right_arm'])
   Orocos.conf.apply(joint_limit_avoidance, ['default'])
   Orocos.conf.apply(joint_pos_ctrl, ['default'])

   robot_model.joint_state.connect_to dispatcher.all_with_rover_joint_state
   robot_model.task_frames.connect_to wbc.task_frames
   wbc.linear_eqn_pp.connect_to solver.linear_eqn_pp
   wbc.solver_output.connect_to solver.solver_output
   wbc.joint_state.connect_to dispatcher.all_with_rover_joint_state
   wbc.ctrl_out.connect_to dispatcher.bodywithhead_and_rover_command #robot_control.command

   # Configure here to create ports
   wbc.configure

   Orocos.transformer.setup(cart_pos_ctrl_right_arm)  
   joint_limit_avoidance.feedback.connect_to dispatcher.all_joint_state
   joint_pos_ctrl.feedback.connect_to dispatcher.all_joint_state

   # Connect controllers to wbc
   cart_pos_ctrl_right_arm.ctrl_out.connect_to wbc.ref_cart_pos_ctrl_right_arm
   joint_limit_avoidance.ctrl_out.connect_to wbc.ref_joint_limit_avoidance
   joint_limit_avoidance.activation.connect_to wbc.weight_joint_limit_avoidance
   joint_pos_ctrl.ctrl_out.connect_to wbc.ref_joint_pos_ctrl


   joint_pos_ctrl.configure
   cart_pos_ctrl_right_arm.configure
   joint_limit_avoidance.configure
   solver.configure
   robot_model.configure
   joint_pos_ctrl.start
   cart_pos_ctrl_right_arm.start
   joint_limit_avoidance.start
   robot_model.start
   solver.start
   wbc.start
   
   puts "Press ENTER to stop!"
   STDIN.readline
end
