require 'rock/bundle'

Bundles.initialize
Orocos.conf.load_dir('config')

Orocos.run do# 'wbc::WbcVelocityTask' => 'wbc' do
    
   robot_model = Orocos.name_service.get 'robot_model'
   wbc = Orocos.name_service.get 'orogen_default_wbc__WbcVelocityTask'
   
   Orocos.conf.apply(wbc, ['default'])

   robot_model.task_frames.connect_to wbc.task_frames

   wbc.configure
   wbc.start
   puts "Press ENTER to stop!"
   STDIN.readline
    
end
