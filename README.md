# WBC - Whole-Body Control

[Code API](http://bob.dfki.uni-bremen.de/apis/dfki-control/wbc/orogen-wbc)  | [Full Documentation](https://git.hb.dfki.de/wbc/documentation/wikis/home)

This task library provides a Rock interface for the Whole-Body Control library. It facilitates intuitive specification and execution of reactive robot control problems that involve multiple simultaneously running tasks. 

WBC was initiated and is currently developed at the [Robotics Innovation Center](http://robotik.dfki-bremen.de/en/startpage.html) of the [German Research Center for Artificial Intelligence (DFKI)](http://www.dfki.de) in Bremen.

<img src="doc/images/DFKI_Logo_e_schrift.jpg" alt="drawing" width="300"/>

## Motivation

WBC is a framework for optimization-based control of redundant robots. It contains various implementations of whole-body feedback control approaches on velocity-, acceleration- and force/torque-level. WBC is meant for controlling robots with redundant degrees of freedom, like humanoids or other legged robots with floating base, but also fixed-base systems like mobile manipulators, dual-arm systems or even simple manipulators. It is also meant for controlling multiple tasks simultaneously while taking into account the physical constraints of the robot. E.g., on a humanoid robot do ... (1) keep balance (2) Grasp an object with one arm (3) maintaining an upright body posture (4) Consider the joint torque limits,  etc... WBC is a purely reactive approach, i.e., it does not involve any motion planning or trajectory optimization. However, it can be used to stabilize trajectories coming from a motion planner or trajectory optimizer and integrate them with other objectives and physical constraints of the robot.


The general idea of optimization-based robot control is to formulate simultaneously running robot tasks as constraints or within the cost function of an instantaneous optimization problem. 
Now, in each control cycle ...

  * The constraints/cost functions are updated with the current robot state/control reference
  * The optimization problem is solved
  * The solution is applied to the actuators of the robot

The online solution of this problem is the robot joint control signal that complies with all tasks, while integrating physical constraints like actuator limits. An advantage of this approach is that complex tasks can be composed from low-dimensional descriptors, which are typically  easier to specify and control than the complete task are once. Also, the redundancy of the robot is nicely exploited utilizing  all the dof of the system (whole body).

## Getting Started

* Please check out the tutorials section in the [documentation](https://git.hb.dfki.de/dfki-control/wbc/documentation/-/wikis/home) for examples of usage.
* Extensive examples can be found in the [wbc_examples bundle](https://git.hb.dfki.de/dfki-control/wbc/bundle-wbc_examples).

## Requirements / Dependencies

Currently supported OS: Ubuntu18.04, Ubuntu20.04

This task library is for sole use within the Rock framework (https://www.rock-robotics.org/). I.e., you require a full Rock installation to use it. In addition, it has the following 1st order dependencies:

* WBC library (https://git.hb.dfki.de/dfki-control/wbc/wbc)
* Hyrodyn library (https://git.hb.dfki.de/dfki-mechanics/hyrodyn/hyrodyn)


## Installation

* New Bootstrap: See [here](https://git.hb.dfki.de/wbc/buildconf)
* Existing Rock Installation: Add the wbc package set to your autoproj/manifest file: 
    ```
    package_sets:
    - dfkigit: dfki-control/wbc/package_set
    ```    
  followed by `aup control/orogen/wbc` and then `amake control/orogen/wbc`

If you want to use the WBC gui, do

```
aup & amake gui/wbc_gui
```

## Testing

TODO

## Contributing

Please use the [issue tracker](https://git.hb.dfki.de/dfki-control/wbc/orogen-wbc/-/issues) to submit bug reports and feature requests.

Please use merge requests as described [here](https://git.hb.dfki.de/dfki-control/wbc/orogen-wbc/-/blob/master/CONTRIBUTING.md) to add/adapt functionality. 

## License

orogen-wbc is distributed under the [3-clause BSD license](https://opensource.org/licenses/BSD-3-Clause).

## Maintainer / Authors / Contributers

Dennis Mronga, dennis.mronga@dfki.de

Copyright 2017, DFKI GmbH / Robotics Innovation Center

