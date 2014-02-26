require 'orocos'
require 'pry'
require 'vizkit'

if !ARGV[0]
    STDERR.puts "usage: test_wbc.rb <urdf_file>"
    exit 1
end

include Orocos

Orocos.initialize
Orocos.conf.load_dir('config')

urdf_file = ARGV[0] if ARGV[0]
Orocos.run do
    
   driver = Orocos.name_service.get 'driver'
   
   wbc = Orocos::TaskContext.get 'orogen_default_wbc__WbcVelocityTask'
   Orocos.conf.apply(wbc, ['default'])
   wbc.urdf = urdf_file

   driver.joint_state.connect_to wbc.joint_state

   wbc.configure
   wbc.start
   Vizkit.exec
    
end
