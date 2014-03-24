#ifndef wbc_TYPES_HPP
#define wbc_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <string>
#include <wbc/HierarchicalWDLSSolver.hpp>
#include <base/time.h>
#include <base/Types.hpp>

namespace wbc {

struct SubChainConfig{
    std::string root;
    std::string tip;
};
}

#endif

