This task library provides a Rock interface for the Whole-Body Control library. 

# Installation
A buildconf with installation instructions can be found here: https://git.hb.dfki.de/wbc/buildconf. 

If want to use your own buildconf, just add the [wbc package set](https://git.hb.dfki.de/wbc/wbc_package_set) to your autoproj manifest file. Then do the following:

```
aup & amake control/orogen/wbc
aup & amake control/orogen/ctrl_lib
```

If you want to use the WBC gui, do

```
aup & amake gui/wbc_status_gui
aup & amake gui/wbc_ctrl_gui
```

Example for usage can be found here: https://git.hb.dfki.de/wbc/wbc_examples



