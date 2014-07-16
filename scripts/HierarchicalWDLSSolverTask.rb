require 'rock/bundle'

Bundles.initialize
Orocos.conf.load_dir('config')

Orocos.run 'wbc::HierarchicalWDLSSolverTask' => 'solver' do
    
   wbc = Orocos.name_service.get 'wbc'
   solver = Orocos.name_service.get 'solver'   
   Orocos.conf.apply(solver, ['default'])

   solver.solver_input.connect_to wbc.solver_input

   solver.configure
   solver.start
   puts "Press ENTER to stop!"
   STDIN.readline
    
end
