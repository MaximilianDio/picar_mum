# set env for moving box with simcar ->  .bashrc
export GAZEBO_PLUGIN_PATH="/home/$USER/picar_mum/catkin_ws/src/90_simulation/simulation/worlds/build":$GAZEBO_PLUGIN_PATH

# set aliases for common ros and ssh commands -> .bash_aliases
alias picarssh = 'ssh -o HostKeyAlgorithms='ssh-rsa' pi@picar04.local'
alias picarcopy ='cd ~/picar_mum/catkin_ws/src && bash copy_pi.sh picar04'

#set functions for common ros and ssh commands -> .bashrc 

picarremote() {
if "$1"==simcar
roslaunch picar keyboard_control.launch ns:="$1" sim:=true
fi
roslaunch picar keyboard_control.launch ns:=picar04
} 

picarleadercontrol() {
if "$1"==simcar
roslaunch picar leader_control.launch ns:="$1" sim:=true
fi
roslaunch picar leader_control.launch ns:=picar04
}





