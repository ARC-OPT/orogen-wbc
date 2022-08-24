#ifndef wbc_TYPES_HPP
#define wbc_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace wbc {

struct TimingStats{
    double desired_period;
    double actual_period;
    double time_per_cycle;
    double time_robot_model_update;
    double time_task_update;
    double time_scene_update;
    double time_solve;
};

}

#endif

