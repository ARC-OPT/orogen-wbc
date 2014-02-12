require 'orocos'
require 'pry'
require 'vizkit'

if !ARGV[0] or !ARGV[1] 
    STDERR.puts "usage: test_wbc.rb <urdf_file> <srdf_file>"
    exit 1
end

include Orocos

Orocos.initialize
Orocos.conf.load_dir('config')

urdf_file = ARGV[0] if ARGV[0]
srdf_file = ARGV[1] if ARGV[1]

Orocos.run 'wbc::WbcVelocityTask' => 'wbc' do
    
   driver = Orocos.name_service.get 'driver'
   
   wbc = Orocos::TaskContext.get 'wbc'
   Orocos.conf.apply(wbc, ['default'])
   wbc.urdf = urdf_file
   wbc.srdf = srdf_file

   driver.joint_state.connect_to wbc.joint_state

   wbc.configure
   wbc.start
   Vizkit.exec
    
end
