# pyGzUAVSim

This is only supported for Ubuntu, specifically 22.04LTS.

## Installing Gazebo

You will need Gazebo Garden (maybe Fortress works too), NOT gazebo classic. Install from https://gazebosim.org/docs/garden/install_ubuntu


## Building the modules 

Pre-requisites (some maybe missing, let me know)
```shell
sudo apt install cmake build-essentials libboost-all-dev
```

Build (this will create the `.so`'s in `gz-plugins/libexec`)
```shell
make gz-plugins
```


## Run

Uncomment the drone model you want in `worlds/cyberzoo.sdf`. Make sure your FTDI is at `/dev/ttyUSB0`, and your Raspberry PI at `10.0.0.1` (if using external position). If that's not the case, adapt `model.sdf`

```shell
GZ_SIM_RESOURCE_PATH="models:external/Cyberzoo_gazebo_models/models" GZ_SIM_SYSTEM_PLUGIN_PATH="gz-plugins/libexec" gz sim worlds/cyberzoo.sdf
```



