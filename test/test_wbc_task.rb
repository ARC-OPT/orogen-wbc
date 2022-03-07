require "orocos"
require "test/unit"

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

            Orocos.conf.apply(task, ["default", "configure_fail_invalid_floating_base_state"])
            assert_raise(Orocos::StateTransitionFailed){task.configure}

            Orocos.conf.apply(task, ["default", "configure_fail_invalid_blacklist"])
            assert_raise(Orocos::StateTransitionFailed){task.configure}
        end
    end

    def testSingleTask
        Orocos.run $tc_name => "task" do
            task = Orocos::TaskContext.get "task"
            Orocos.conf.load_dir("./config")
            Orocos.conf.apply(task, ["default", "configure_success"])
            assert_nothing_raised(Orocos::StateTransitionFailed){task.configure}
            assert_nothing_raised(Orocos::StateTransitionFailed){task.start}

            joint_state = Types.base.samples.Joints.new
            joint_state.names = ["kuka_lbr_l_joint_1",
                                 "kuka_lbr_l_joint_2",
                                 "kuka_lbr_l_joint_3",
                                 "kuka_lbr_l_joint_4",
                                 "kuka_lbr_l_joint_5",
                                 "kuka_lbr_l_joint_6",
                                 "kuka_lbr_l_joint_7"]
            for i in (0..6)
                js = Types.base.JointState.new
                js.position = 1.0
                js.speed = 0.0
                joint_state.elements << js
            end
            joint_state.time = Types.base.Time.now

            writer_joint_state = task.joint_state.writer
            writer_joint_state.write joint_state

            assert_nothing_raised(){task.activateConstraint("cart_pos_ctrl", 1.0)}

            reference = Types.base.samples.RigidBodyStateSE3.new
            reference.twist.linear  = reference.acceleration.linear = Types.base.Vector3d.new(rand()/10,rand()/10,rand()/10)
            reference.twist.angular = reference.acceleration.angular = Types.base.Vector3d.new(rand()/10,rand()/10,rand()/10)
            reference.time = Types.base.Time.now

            writer_ref = task.ref_cart_pos_ctrl.writer
            writer_ref.write reference

            constraint_status = nil
            reader_constraint_status = task.constraint_cart_pos_ctrl.reader
            while constraint_status == nil
                constraint_status = reader_constraint_status.read_new
            end

            (0..2).each do |i|
                assert_in_delta(reference.acceleration.linear[i],  constraint_status.y_solution[i],   1e-4)
                assert_in_delta(reference.acceleration.angular[i], constraint_status.y_solution[i+3], 1e-4)
            end

        end
    end
end
