ROS_LOC=/home/student/ros_ws

Workspace() {
  cd $ROS_LOC
  ./baxter.sh sim
}

StartBaxter() {
  roslaunch baxter_gazebo baxter_world.launch
}

EnableBaxter() {
  rosrun baxter_tools enable_robot.py -e
  SetupChannel
  rosrun baxter_examples baxterJointCommandAndRead.py
}

SetupChannel() {
  ach -l -C "baxter_ref" -m 20 -n 3000
  ach -l -C "baxter_state" -m 20 -n 3000
}

case $1 in 
  'sim')
    StartBaxter
  ;;
  'start')
    EnableBaxter
  ;;
  'workspace')
    Workspace
  ;;
  *)
    echo 'Options:'
    echo 'sim       - setup the simulator for baxter'
    echo 'start     - enable baxter, setup channels and run baxterJoint file'
    echo 'workspace - enter baxter workspace'
  ;;
esac

