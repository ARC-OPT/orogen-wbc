This task library provides a Rock interface for the Whole-Body Control library. 

# Installation
A buildconf for installation can be found here: https://git.hb.dfki.de/dfki-control/wbc/buildconf. 

If want to use your own buildconf, just add the [wbc package set](https://git.hb.dfki.de/dfki-control/wbc/package_set) to your autoproj manifest file. Then do the following:

```
aup & amake control/orogen/wbc
aup & amake control/orogen/ctrl_lib
```

If you want to use the WBC gui, do

```
aup & amake gui/wbc_gui
```

# Usage

* Simple introductory examples can be found [here](https://git.hb.dfki.de/dfki-control/wbc/orogen-wbc/-/tree/master/tutorials/kuka_iiwa).
* Extensive examples can be found in the [wbc_examples bundle](https://git.hb.dfki.de/dfki-control/wbc/bundle-wbc_examples).



