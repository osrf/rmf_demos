# Installation

## Setup

Setup your computer to accept Gazebo packages from packages.osrfoundation.org.

```bash
sudo apt update
sudo apt install -y wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```
Install all non-ROS dependencies of RMF packages,

```bash
sudo apt update && sudo apt install \
  git cmake python3-vcstool curl \
  qt5-default \
  libboost-system-dev libboost-date-time-dev libboost-regex-dev libboost-random-dev \
  python3-shapely python3-yaml python3-requests \
  libignition-common3-dev libignition-plugin-dev \
  g++-8 \
  -y
```

Setup a new ROS 2 workspace and pull in the demo repositories using `vcs`,

```bash
mkdir -p ~/rmf_demos_ws/src
cd ~/rmf_demos_ws
wget https://raw.githubusercontent.com/osrf/rmf_demos/master/rmf_demos.repos
vcs import src < rmf_demos.repos
```

Ensure all ROS 2 prerequisites are fulfilled,

```bash
cd ~/rmf_demos_ws
rosdep install --from-paths src --ignore-src --rosdistro <ROS_DISTRO> -yr
```

The models required for each of the demo worlds will be automatically downloaded into `~/.gazebo/models` from Ignition [Fuel](https://app.ignitionrobotics.org/fuel) when building the package `rmf_demo_maps`. If you notice something wrong with the models in the simulation, your `~/.gazebo/models` path might contain deprecated models not from `Fuel`. An easy way to solve this is to remove all models except for `sun` and `ground_plane` from `~/.gazebo/models`, and perform a clean rebuild of the package `rmf_demo_maps`.

## Compiling Instructions

#### Ubuntu 20.04 and ROS 2 Foxy:

```bash
cd ~/rmf_demos_ws
source /opt/ros/foxy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELEASE
```

#### Ubuntu 18.04 and ROS 2 Eloquent:

```bash
cd ~/rmf_demos_ws
source /opt/ros/eloquent/setup.bash
CXX=g++-8 colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELEASE
```
> Note: The build will fail if the compiler is not set to g++ version 8 or above.
