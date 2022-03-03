require "orocos"
require "test/unit"
require "pry"

$tc_name = nil

class TestWbcTask < Test::Unit::TestCase

    # Test successful configuration, check that all dynamic ports exist
    def testConfigureSuccess
        Orocos.run $tc_name => "task" do
            task = Orocos::TaskContext.get "task"
            Orocos.conf.load_dir("./config")
            Orocos.conf.apply(task, ["default", "configure_success"])
            assert_nothing_raised(Orocos::StateTransitionFailed){task.configure}

            task_config = task.wbc_config[0]
            assert_equal(task.has_port?("ref_" + task_config.name), true)
            assert_equal(task.has_port?("weight_" + task_config.name), true)
            assert_equal(task.has_port?("activation_" + task_config.name), true)
            assert_equal(task.has_port?("status_" + task_config.name), true)
            assert_equal(task.has_port?("constraint_" + task_config.name), true)
        end
    end

    # Test reconfiguration; ensure dynamic ports are not cleared if configuration does not change
    def testReconfigure
        Orocos.run $tc_name => "task" do
            task = Orocos::TaskContext.get "task"
            Orocos.conf.load_dir("./config")
            Orocos.conf.apply(task, ["default", "configure_success"])
            assert_nothing_raised(Orocos::StateTransitionFailed){task.configure}
            assert_nothing_raised(Orocos::StateTransitionFailed){task.start}

            port_writer_reference  = task.ref_cart_pos_ctrl.writer
            port_writer_weight     = task.weight_cart_pos_ctrl.writer
            port_writer_activation = task.activation_cart_pos_ctrl.writer
            port_reader_status     = task.status_cart_pos_ctrl.reader
            port_reader_constraint = task.constraint_cart_pos_ctrl.reader

            assert_equal(port_writer_reference.write(Types.base.samples.RigidBodyStateSE3.new), true)
            assert_equal(port_writer_weight.write(Types.base.VectorXd.new(6)), true)
            assert_equal(port_writer_activation.write(1), true)
            assert_equal(port_reader_status.read, nil)
            assert_equal(port_reader_constraint.read, nil)

            assert_nothing_raised(Orocos::StateTransitionFailed){task.stop}
            assert_nothing_raised(Orocos::StateTransitionFailed){task.cleanup}
            assert_nothing_raised(Orocos::StateTransitionFailed){task.configure}
            assert_nothing_raised(Orocos::StateTransitionFailed){task.start}

            assert_equal(port_writer_reference.write(Types.base.samples.RigidBodyStateSE3.new), true)
            assert_equal(port_writer_weight.write(Types.base.VectorXd.new(6)), true)
            assert_equal(port_writer_activation.write(1), true)
            assert_equal(port_reader_status.read, nil)
            assert_equal(port_reader_constraint.read, nil)
        end
    end

    # Test reconfiguration with different config; ensure unused dynamic ports are cleared
    def testReconfigure2
        Orocos.run $tc_name => "task" do
            task = Orocos::TaskContext.get "task"
            Orocos.conf.load_dir("./config")
            Orocos.conf.apply(task, ["default", "configure_success"])
            assert_nothing_raised(Orocos::StateTransitionFailed){task.configure}
            assert_nothing_raised(Orocos::StateTransitionFailed){task.start}

            task_config = task.wbc_config[0]
            assert_equal(task.has_port?("ref_" + task_config.name), true)
            assert_equal(task.has_port?("weight_" + task_config.name), true)
            assert_equal(task.has_port?("activation_" + task_config.name), true)
            assert_equal(task.has_port?("status_" + task_config.name), true)
            assert_equal(task.has_port?("constraint_" + task_config.name), true)

            assert_nothing_raised(Orocos::StateTransitionFailed){task.stop}
            assert_nothing_raised(Orocos::StateTransitionFailed){task.cleanup}
            Orocos.conf.apply(task, ["default", "configure_success_2"])
            assert_nothing_raised(Orocos::StateTransitionFailed){task.configure}
            assert_nothing_raised(Orocos::StateTransitionFailed){task.start}

            assert_equal(task.has_port?("ref_" + task_config.name), false)
            assert_equal(task.has_port?("weight_" + task_config.name), false)
            assert_equal(task.has_port?("activation_" + task_config.name), false)
            assert_equal(task.has_port?("status_" + task_config.name), false)
            assert_equal(task.has_port?("constraint_" + task_config.name), false)

            task_config_new = task.wbc_config[0]
            assert_equal(task.has_port?("ref_" + task_config_new.name), true)
            assert_equal(task.has_port?("weight_" + task_config_new.name), true)
            assert_equal(task.has_port?("activation_" + task_config_new.name), true)
            assert_equal(task.has_port?("status_" + task_config_new.name), true)
            assert_equal(task.has_port?("constraint_" + task_config_new.name), true)
        end
    end

    def testConfigureSuccessFloatingBase
        Orocos.run $tc_name => "task" do
            task = Orocos::TaskContext.get "task"
            Orocos.conf.load_dir("./config")
            Orocos.conf.apply(task, ["default", "configure_success_floating_base"])
            assert_nothing_raised(Orocos::StateTransitionFailed){task.configure}
        end
    end

    def testConfigureFail
        Orocos.run $tc_name => "task" do
            task = Orocos::TaskContext.get "task"
            Orocos.conf.load_dir("./config")

            Orocos.conf.apply(task, ["default", "configure_fail_invalid_urdf_file"])
            assert_raise(Orocos::StateTransitionFailed){task.configure}

            Orocos.conf.apply(task, ["default", "configure_fail_invalid_joint_name"])
            assert_raise(Orocos::StateTransitionFailed){task.configure}

            Orocos.conf.apply(task, ["default", "configure_fail_invalid_floating_base_state"])
            assert_raise(Orocos::StateTransitionFailed){task.configure}

            Orocos.conf.apply(task, ["default", "configure_fail_invalid_blacklist"])
            assert_raise(Orocos::StateTransitionFailed){task.configure}
        end

    end
end
