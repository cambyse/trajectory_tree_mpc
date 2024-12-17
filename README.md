This repo contains the code for MPC examples using trajectory-tree optimization.

# Building the Examples

For a straightforward installation that works seamlessly on most Ubuntu systems (20.04 and later) and MacOS, we recommend using the Dockerized setup. However, this may result in slightly reduced performance and less smooth visualization due to graphical performance limitations within Docker.

Alternatively, a native system installation is possible but may require additional steps to have all correct dependencies necessary to compile the examples.

## Building using Docker

#### Clone repository containing the Docker file (https://github.com/cambyse/trajectory_tree_mpc_deployment)

```bash
git clone git@github.com:cambyse/trajectory_tree_mpc_deployment.git
```

#### Build Docker image

```bash
cd trajectory_tree_mpc_deployment
docker build -t mpc .
```

This creates a docker image called `mpc`. It installs the dependencies, clones repositories and compiles the examples. 


#### Run Docker image

The easiest way to run the examples is to open two terminal connecting to the same docker image. To this end, first launch the image in detached mode

```bash
docker run --privileged --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -it -d --name shared-mpc mpc
```

The above command runs the docker image and opens a bash terminal. The options `--privileged --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix` are to forward the host display to the Docker container.

## Building on the native system

To build the examples on the native system, one has to follow the same steps as in Dockerfile (https://github.com/cambyse/trajectory_tree_mpc_deployment). ⚠️ This is tested only on `Ubuntu 20.04.6 LTS`.

### Install dependencies

#### ros
The planner is intergated as a ros node. It has been developped with ros noetic but should be compatible with merely any recent ros distro.
[Ros installation](https://www.ros.org/install/)

#### gazebo
Car dynamic is simulated with Gazebo.
[Gazebo installation](http://gazebosim.org/tutorials?tut=ros_wrapper_versions&cat=connect_ros)

#### osqp
We use the QP solver OSQP as a baseline (see paper).
[OSQP installation](https://osqp.org/docs/installation/cc++)

#### other dependencies

In addition, the following packages are necessary: gnuplot, libjsoncpp-dev, libx11-dev, liblapack-dev, libf2c2-dev, libeigen3-dev, libglew-dev, freeglut3-dev.
They can be installed by calling `sudo apt install PACKAGE_NAME`.

### Clone repository and its submodules
Clone the repository and its submodules into your catkin workspace:
```bash
git clone --recursive git@github.com:cambyse/trajectory_tree_mpc.git
```
This will clone the code related to the examples and two submodules (https://github.com/cambyse/rai_for_mpc, and https://github.com/cambyse/tamp_for_mpc).

### Install gazebo models
The car, obstacle and pedestrian gazebo models are part of the repository and must be copied to the gazebo folder containing all models.
This gazebo folder is typicall under `~/.gazebo/models`.
```bash
cd trajectory_tree_mpc/lgp_car_models
cp -r * ~/.gazebo/models
```

### Build Rai
The rai submodule doesn't use cmake as build system, it has to be build separatly.
```bash
cd ../control_tree_car/externals/rai
make
```
The build can take a few minutes.

#### Build ros nodes
The rest of the code can be built as standard ros nodes using `catkin_make` in the source directory of the catkin workspace.
```bash
catkin_make
```

#### Execute tests
Open a terminal in the folder corresponding to the `control_tree_car` package in `build` folder of the catkin workspace.
This is typically `${CATKIN_WORKSPACE}/build/trajectory_tree_mpc/control_tree_car` where `CATKIN_WORKSPACE` is the root of the catkin workspace.

The tests can be launched using ctest:
```bash
ctest .
```

# Launch examples

Open two terminals in the catkin workspace. When running the examples within docker, one needs additionally to connect to the shared docker image:

```bash
docker exec -it shared-mpc /bin/bash
```

Source the ros environment.
```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

### Pedestrian example


In the first terminal, launch the planner and the visualization RViz.
```bash
roslaunch control_tree_car pedestrian.launch
```

In the second terminal, launch the simulator.
```bash
gzserver src/trajectory_tree_mpc/lgp_car_gazebo_plugin/world/pedestrian_4.world
```

![Image](control_tree_car/data/doc/pedestrians.png)

### Obstacle avoidance example

In the first terminal, launch the planner and the visualization RViz.
```bash
roslaunch control_tree_car obstacle_avoidance.launch
```

In the second terminal, launch the simulator.
```bash
gzserver src/trajectory_tree_mpc/lgp_car_gazebo_plugin/world/obstacle_avoidance_2.world
```

![Image](control_tree_car/data/doc/obstacles.png)

> **Note:** A standalone version of the solver can be found by following the link [Solver only](https://github.com/ControlTrees/solver).
This version needs less dependencies (e.g. no ros) and doesn't contain the examples of the paper.

