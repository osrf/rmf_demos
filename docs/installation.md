# Installation

## Install ROS 2 Foxy

First, please follow the installation instructions for ROS 2 Foxy.
If you are on an Ubuntu 20.04 LTS machine (as recommended), [here is the binary install page for ROS 2 Foxy on Ubuntu 20.04](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/)

## Install rosdep
`rosdep` helps install dependencies for ROS packages across various distros. It can be installed with:
```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

## Additional Dependencies

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
  python3-shapely python3-yaml python3-requests \
  -y
sudo apt-get install python3-colcon*
```

## Download the source code
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
rosdep install --from-paths src --ignore-src --rosdistro foxy -yr
```

The models required for each of the demo worlds will be automatically downloaded into `~/.gazebo/models` from Ignition [Fuel](https://app.ignitionrobotics.org/fuel) when building the package `rmf_demo_maps`. If you notice something wrong with the models in the simulation, your `~/.gazebo/models` path might contain deprecated models not from `Fuel`. An easy way to solve this is to remove all models except for `sun` and `ground_plane` from `~/.gazebo/models`, and perform a clean rebuild of the package `rmf_demo_maps`.

## Compiling Instructions

#### Ubuntu 20.04:

```bash
cd ~/rmf_demos_ws
source /opt/ros/foxy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

NOTE: The first time the build occurs, many simulation models will be downloaded from Ignition Fuel to populate the scene when the simulation is run.
As a result, the first build can take a very long time depending on the server load and your Internet connection (for example, 60 minutes).

#### Ubuntu 18.04:

Running on Ubuntu 18.04 is not officially supported and requires building [Foxy from source](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/), you will also need to install g++8 and the ignition libraries manually:

```
sudo apt install g++-8 libignition-common3-dev libignition-plugin-dev -y
```

```bash
cd ~/rmf_demos_ws
source ~/ros2_foxy/install/setup.bash
CXX=g++-8 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
> Note: The build will fail if the compiler is not set to g++ version 8 or above.

## Install RMF Panel Dashbaord
```bash
sudo apt install npm
python3 -m pip install Flask flask-socketio flask-cors
cd ~/rmf_demos_ws
npm install --prefix src/rmf_demos/rmf_demo_panel/rmf_demo_panel/static/
npm run build --prefix src/rmf_demos/rmf_demo_panel/rmf_demo_panel/static/
colcon build --packages-select rmf_demo_panel
```
