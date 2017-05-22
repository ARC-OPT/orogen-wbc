#ifndef wbc_TYPES_HPP
#define wbc_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <string>
#include <base/samples/RigidBodyState.hpp>

namespace wbc {
class RobotModelConfig{
public:
    RobotModelConfig(){
        initial_pose.setTransform(Eigen::Affine3d::Identity());
    }
    RobotModelConfig(const std::string& _file,
                     const std::string& _hook = "",
                     const base::samples::RigidBodyState& _initial_pose = base::samples::RigidBodyState()) :
        file(_file),
        hook(_hook),
        initial_pose(_initial_pose){
    }

    std::string file;                            /** Path to robot model file*/
    std::string hook;                            /** Frame to which this robot model is attached in the overall model*/
    base::samples::RigidBodyState initial_pose;  /** Initial pose of this model relative to the hook frame*/
};
}

#endif

