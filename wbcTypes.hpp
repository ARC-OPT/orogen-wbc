#ifndef wbc_TYPES_HPP
#define wbc_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <string>
#include <vector>
#include <base/Eigen.hpp>

namespace wbc {

struct SubChainConfig{
    std::string root;
    std::string tip;
};

struct SolverInput{
    std::vector<base::MatrixXd> A;  /** Prioritized equation system constructed from Task Jacobians */
    std::vector<base::VectorXd> Wt; /** Task Weight Vectors per priority */
    base::VectorXd Wq;              /** Joint Weight Vector */
    std::vector<base::VectorXd> y;  /** Constraint variables */
};
}

#endif

