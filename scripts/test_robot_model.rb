require 'rock/bundle'

Bundles.initialize
Orocos.conf.load_dir('config')


Orocos.run 'wbc::RobotModelKDL' => 'robot_model' do
   
   robot_model = Orocos.name_service.get 'robot_model'
   dispatcher = Orocos.name_service.get 'aila_dispatcher'
   Orocos.conf.apply(robot_model, ['default'])

   robot_model.joint_state.connect_to dispatcher.all_joint_state

   robot_model.configure
   robot_model.addTaskFrame("Hand_r")
   robot_model.start

   puts "Press ENTER to stop!"
   STDIN.readline
end
