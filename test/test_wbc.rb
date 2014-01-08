require 'orocos'
require 'vizkit'
require 'readline'

if !ARGV[0] or !ARGV[1] or !ARGV[2]
    STDERR.puts "usage: test_wbc.rb <urdf_file> <srdf_file> <wbc_config_file>"
    exit 1
end

include Orocos

Orocos.initialize

urdf_file = ARGV[0] if ARGV[0]
srdf_file = ARGV[1] if ARGV[1]
wbc_config_file = ARGV[2] if ARGV[2]

Orocos.run 'wbc::Task' => 'wbc',
           'joint_control::FakeJointDriverTask' => 'driver',
           'robot_model_tools::Task' => 'robot_model'  do
    
   robot_model = Orocos.name_service.get 'robot_model'
   robot_model.urdf_file = urdf_file
   robot_model.configure

   driver = Orocos.name_service.get 'driver'
   limits = robot_model.GetJointLimits()
   driver.cycle_time = 0.005
   driver.joint_names = robot_model.GetJointNames
   initial_joint_state = Types::Base::Samples::Joints.new
   initial_joint_state.names = driver.joint_names
   limits.elements.each do |l|
      init_state = Types::Base::JointState.new
      init_state.position = (l.max.position + l.min.position)/2
      init_state.speed = init_state.effort = init_state.raw = 0
      initial_joint_state.elements << init_state
   end
   driver.initial_joint_state = initial_joint_state
   driver.meas_noise_std_dev = 1e-5

   wbc = Orocos.name_service.get 'wbc'
   wbc.urdf = urdf_file
   wbc.srdf = srdf_file
   wbc.wbc_config = wbc_config_file
   wbc.wbc_mode = 0

   driver.joint_status.connect_to wbc.joint_status

   driver.configure   
   wbc.configure
    
   driver.start
   wbc.start
   Vizkit.exec
    
end
