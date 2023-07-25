# ForceShoesPlugin

This plugin integrates the force sensors of the `Xsens force shoes` into `mc_rtc`.

It depends on:
- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc)
- [The MT SDK](https://content.xsens.com/hubfs/Downloads/Software/mtsdk3.3_setup.zip)

## Installation

After installing the dependencies,

```sh
git clone git@github.com:Hugo-L3174/Force_shoes.git
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j8
sudo make install
```

## Running instructions

Run using your `mc_rtc` interface of choice, add `ForceShoesPlugin` to the Plugins configuration entry or enable the autoload option.
By default the set baudrate is 921k6 and the port is "/dev/ttyUSB0" since this is intended to be used on a linux machine but this can be changed in the ForceShoePlugin.in.yaml configuration file.