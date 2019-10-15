# Simulation Package
Gazebo will not find the models inside this package by default.
Therefore the model path need to be extended.
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<path_of_catkin_ws>/src/simulation/models
```

If the location of this project is in your home folder it will probably look like this:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/picar_mum/catkin_ws/src/90_simulation/simulation/models
```
Additionally to help gazebo find the world file:
```bash
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:<path_of_catkin_ws>/src/90_simulation/simulation/worlds
```
If the location of this project is in your home folder it will probably look like this:
```bash
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/picar_mum/catkin_ws/src/90_simulation/simulation/worlds

```

```bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/picar_mum/catkin_ws/src/90_simulation/plugins/build
```
