# ForceShoesPlugin

This plugin integrates the force sensors of the `Xsens force shoes` into `mc_rtc`.

It depends on:
- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc) >= 2.14.0

## Installation

After installing the dependencies,

```sh
git clone git@github.com:Hugo-L3174/mc_force_shoe_plugin.git
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j8
sudo make install
```

## Running instructions

Run using your `mc_rtc` interface of choice, add `ForceShoesPlugin` to the Plugins configuration entry or enable the autoload option.

### Plugin configuration


The default configuration of the plugin contains default settings for all sensors availnle at `LIRMM`. As this was a prototype from `XSens` it is unlikely that one will need to add further sensors there, but this is possible in `ForceShoePlugin.in.yaml`. All calibration matrices were obtained from the `ForceShoe V1.0 Serial Numbers CNRS.pdf` (not included in this repository due as we did not ask `Xsens` for permission).

By default the set baudrate is `921600` and the port is `/dev/ttyUSB0` since this is intended to be used on a linux machine but this can be changed in the `ForceShoePlugin.in.yaml` configuration file.

The full configuration of the plugin looks as follows:

```yaml
comPort: /dev/ttyUSB0
baudrate: 921600
# Path where the calibration is stored
# If the file is not present the plugin automatically starts in calibration mode, otherwise it starts in measurement mode
calibFile: /tmp/force-shoe-calib.yaml

# This information is provided in ForceShoe V1.0 Serial Numbers CNRS.pdf
# All sensors available in LIRMM are
#
# Each force sensor is connected together with a motion tracker
forceShoeSensors:
- motionTracker:
    serialNumber: !!str "02325040"
  forceSensor:
    serialNumber: !!str "FT12068"
    calibrationMatrix:
      [
        [343.630637281982,  105.105983793954,  -2953.26625126029, 22314.9264488265,  3260.3934704084,   -21719.9644384895],
        [1623.0777459875,   -25552.3321523685, -1349.53867217226, 12844.2106735018,  -2356.21149902445, 12510.2165878903],
        [-30142.1049892255, 562.224608578618,  -30329.979677345,  697.861864133106,  -30505.173520053,  336.661667405005],
        [4.92731520408949,  -176.535546824105, 480.527538919159,  73.0865549273997,  -511.800932029994, 95.7465164537783],
        [-575.310737786879, 14.7864098252503,  306.87524317224,   -161.111324281113, 261.864725287537,  144.440513232962],
        [-18.7543249035664, 329.16211996994,   -43.6261968319026, 327.208881044527,  -45.4574714985197, 316.294118203176]
      ]
```

However in your user code, you only need to specify which sensors to use and how to link them with `ForceSensor` of your `RobotModule`:

```yaml
# If defined and the sensor is connected,
# This links force sensors from the robot module with force measurement from the shoes
robotForceSensors:
  rhps1:
  - robotForceSensorName: LeftHandForceSensor
    serialNumber: !!str "FT15249"
    guiScale: 10
    flipMeasurementAxis:
      x: true
      y: false
      z: false
  - robotForceSensorName:  RightHandForceSensor
    serialNumber: !!str "FT15251"
    guiScale: 10
    flipMeasurementAxis:
      x: true
      y: false
      z: false
```

with this configuration the `ForceShoePlugin` will be available, and the data from sensor `FT15249` available in the `robot("rhps1").forceSensor("LeftHandForceSensor")` sensor (resp. `FT15251/RightHandForceSensor`). Thus it can be used as any other sensors by tasks such as the `AdmittanceTask/ImpedanceTask`, etc.

Calibration is done through the `GUI`, by calibration we mean zeroing out the initial sensor measurement that is otherwise random. If no calibration is present it starts in calibration mode, in which case any weight measured by the sensor will be set to zero (that is the end-effector weight after the sensor is ignored). If a calibration file has previously been saved it is used.
