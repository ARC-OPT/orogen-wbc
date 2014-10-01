#!/usr/bin/env ruby

require 'pry'
require 'orocos'

Orocos.initialize
Orocos.conf.load_dir('config')

Orocos.run 'wbc_dynamic' do 
   
   solver = Orocos.name_service.get 'wbc_solver'    
   wbc = Orocos.name_service.get 'wbc'
   robot_model = Orocos.name_service.get 'wbc_robot_model'
   dispatcher = Orocos.name_service.get 'aila_dispatcher'

   Orocos.conf.apply(solver, ['default'])
   Orocos.conf.apply(robot_model, ['default'])   
   Orocos.conf.apply(wbc, ['default'])

   robot_model.joint_state.connect_to dispatcher.all_joint_state
   robot_model.task_frames.connect_to wbc.task_frames
   wbc.solver_input.connect_to solver.solver_input
   wbc.solver_output.connect_to solver.solver_output
   wbc.joint_state.connect_to dispatcher.all_joint_state

   wbc.configure
   solver.configure
   robot_model.configure
   robot_model.start
   solver.start
   wbc.start
   
   puts "Press ENTER to stop!"
   STDIN.readline
end
