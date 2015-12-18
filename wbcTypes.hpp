#ifndef wbc_TYPES_HPP
#define wbc_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <base/samples/RigidBodyState.hpp>

namespace wbc {

struct URDFModel{
    URDFModel(){
        initial_pose.setTransform(Eigen::Affine3d::Identity());
    }
    URDFModel(const std::string _file,
              const base::samples::RigidBodyState _initial_pose = base::samples::RigidBodyState(),
              const std::string _hook = "") :
        file(_file),
        hook(_hook),
        initial_pose(_initial_pose){
    }

    std::string file;
    std::string hook;
    base::samples::RigidBodyState initial_pose;
};

}

#endif

