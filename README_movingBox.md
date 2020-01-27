# moving box with simcar 
cd ~/picar_mum/catkin_ws/src/90_simulation/simulation/worlds/build
export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATHroslaunch picar simple_loop_new.launch ns:=simcar sim:=true

# recompile
cd ~/picar_mum/catkin_ws/src/90_simulation/simulation/worlds/
rm -r build
mkdir build
cd build
cmake ../
make
export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH
cd ..
roslaunch picar simple_loop_new.launch ns:=simcar sim:=true
