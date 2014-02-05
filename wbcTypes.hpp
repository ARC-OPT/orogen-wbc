#ifndef wbc_TYPES_HPP
#define wbc_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <wbc/SubTaskConfig.hpp>

namespace wbc {

struct SubTaskConfigSRDF{
    /** Whole body task type, can be joint space or Cartesian for now */
    task_type type;

    /** Joint group that is associated with the task as defined in srdf file */
    std::string joint_group;

    /** Priority of this subtask. 0-based. 0 ^= highest priority */
    unsigned int  priority;
};	
}

#endif

