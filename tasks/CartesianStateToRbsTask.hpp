/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WBC_CARTESIANSTATETORBSTASK_TASK_HPP
#define WBC_CARTESIANSTATETORBSTASK_TASK_HPP

#include "wbc/CartesianStateToRbsTaskBase.hpp"
#include <base/samples/RigidBodyState.hpp>
#include <ctrl_types/RigidBodyStateSE3.hpp>

namespace wbc{

    /*! \class CartesianStateToRbsTask
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
Convert samples of type base/samples/RigidBodyState to type base/samples/CartesianState

     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','wbc::CartesianStateToRbsTask')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class CartesianStateToRbsTask : public CartesianStateToRbsTaskBase
    {
	friend class CartesianStateToRbsTaskBase;
    protected:

        typedef RTT::InputPort<base::samples::RigidBodyStateSE3> CartesianStateInputPort;
        typedef std::shared_ptr< RTT::InputPort<base::samples::RigidBodyStateSE3> > CartesianStateInputPortPtr;
        typedef RTT::OutputPort<base::samples::RigidBodyState> RbsOutputPort;
        typedef std::shared_ptr< RTT::OutputPort<base::samples::RigidBodyState> > RbsOutputPortPtr;

        typedef std::map<std::string, CartesianStateInputPortPtr> CartesianStateInputPortMap;
        typedef std::map<std::string, RbsOutputPortPtr> RbsOutputPortMap;

        base::samples::RigidBodyState toRigidBodyState(const base::samples::RigidBodyStateSE3& in);

        CartesianStateInputPortMap input_port_map;
        RbsOutputPortMap output_port_map;

    public:
        /** TaskContext constructor for CartesianStateToRbsTask
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        CartesianStateToRbsTask(std::string const& name = "wbc::CartesianStateToRbsTask");

        /** Default deconstructor of CartesianStateToRbsTask
         */
	~CartesianStateToRbsTask();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

