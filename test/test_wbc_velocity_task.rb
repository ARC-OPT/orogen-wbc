require "orocos"
require "test/unit"

class TestWbcVelocityTask < Test::Unit::TestCase
    def testConfigureSuccess
        Orocos.run "wbc::WbcVelocityTask" => "task" do
            task = Orocos::TaskContext.get "task"
            Orocos.conf.load_dir("./config")
            Orocos.conf.apply(task, ["default", "configure_success"])
            assert_nothing_raised(Orocos::StateTransitionFailed){task.configure}
        end    
    end
end
