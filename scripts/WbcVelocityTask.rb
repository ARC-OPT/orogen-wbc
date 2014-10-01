require 'rock/bundle'

Orocos.initialize
Orocos.conf.load_dir('config')

Orocos.run 'wbc::WbcVelocityTask' => 'wbc' do
    
   wbc = Orocos.name_service.get 'wbc'
   
   Orocos.conf.apply(wbc, ['default'])
   wbc.configure

   wbc.start
   puts "Press ENTER to stop!"
   STDIN.readline
    
end
