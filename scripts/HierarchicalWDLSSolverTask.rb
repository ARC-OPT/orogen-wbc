require 'rock/bundle'

Orocos.initialize
Orocos.conf.load_dir('config')

Orocos.run 'wbc::HierarchicalWDLSSolverTask' => 'solver' do
    
   solver = Orocos.name_service.get 'solver'   
   Orocos.conf.apply(solver, ['default'])

   solver.configure
   solver.start
   puts "Press ENTER to stop!"
   STDIN.readline
    
end
