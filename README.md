# ForceShoesPlugin

This plugin integrates the force sensors of the Xsens force shoes into `mc_rtc`.

It depends on:
- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc)

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
By default the set baudrate is 921k6 and the port is "/dev/ttyUSB0" since this is intended to be used on a linux machine but this can be changed in the ForceShoePlugin.in.yaml configuration file.

The plugin starts the force shoes and looks for an existing calibration file in '/tmp/force-shoe-calib.yaml' (see the configuration file). If it exists it loads this calibration reads the force data, otherwise it calibrates the shoes on 100 iterations.

> This means that if you do not have a calibration file when starting you must wait before putting on the shoes ! Calibration is fast but do not start with shoes already on.

## Use in a controller

The plugin creates [datastore calls](https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/datastore.html) that can be used in any controller to read the force value of the 4 force sensors, such as:
```cpp
  sva::ForceVecd LFShoe_ = datastore().call<sva::ForceVecd>("ForceShoePlugin::GetLFForce");
  sva::ForceVecd LBShoe_ = datastore().call<sva::ForceVecd>("ForceShoePlugin::GetLBForce");
  sva::ForceVecd RFShoe_ = datastore().call<sva::ForceVecd>("ForceShoePlugin::GetRFForce");
  sva::ForceVecd RBShoe_ = datastore().call<sva::ForceVecd>("ForceShoePlugin::GetRBForce");
```
LF, LB, RF, RB being the left front, left back, right front and right back sensors on the shoes.

GUI elements are also created to recalibrate at runtime if needed.

## Combine with replay

The plugin also handles [replay](https://jrl-umi3218.github.io/mc_rtc/tutorials/tools/mc_rtc_ticker-and-replay.html#using-the-replay-plugin) of the datastore entries if the `log-to-datastore.in.yaml` file of your replay configuration has the following entries:
```yaml
ForceShoes_LBShoeMeasure: ReplayPlugin::LBForce
ForceShoes_LFShoeMeasure: ReplayPlugin::LFForce
ForceShoes_RBShoeMeasure: ReplayPlugin::RBForce
ForceShoes_RFShoeMeasure: ReplayPlugin::RFForce
```
With the left keys being the corresponding log entries.

In this case you must add either
```yaml
liveMode: false
```
to this plugin's configuration, or
```yaml
ForceShoes:
    liveMode: false
```
to your controller's configuration.
