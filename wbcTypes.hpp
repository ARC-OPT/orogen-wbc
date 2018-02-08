#ifndef wbc_TYPES_HPP
#define wbc_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <wbc/LinearEqualityConstraints.hpp>
#include <vector>

namespace wbc {
class HierarchicalLEConstraints{
public:
    /** Time stamp*/
    base::Time time;

    /** Vector of linear equality constraints, where the index of the vector entry corresponds to the priority of the constraints (0 is the highest priority)*/
    std::vector<LinearEqualityConstraints> constraints;

    /** Names of the joints in the same order as in the constraint matrix and the weight vector Wq of each priority*/
    std::vector<std::string> joint_names;
};
}

#endif

