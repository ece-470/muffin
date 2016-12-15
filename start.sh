ROS_LOC=/home/student/ros_ws
ROS_SCR=/home/student/ros_ws/src/baxter_examples/scripts/
MUFFIN=/home/student/projects/muffin

Workspace() {
  cd $ROS_LOC
  ./baxter.sh sim
  cd $MUFFIN
}

StartBaxter() {
  roslaunch baxter_gazebo baxter_world.launch
}

EnableBaxter() {
  cd $ROS_SCR
  rosrun baxter_tools enable_robot.py -e
  SymLink
  SetupChannel
  RunBaxterJoint
}

SetupChannel() {
  ach -1 -C "baxter_ref" -m 20 -n 3000
  ach -1 -C "baxter_state" -m 20 -n 3000
}

SymLink() {
  ln -s ~/projects/muffin/baxterJointCommandAndRead.py baxterJointCommandAndRead.py
  ln -s ~/projects/muffin/baxterStructure.py baxterStructure.py
}

RunBaxterJoint() {
  rosrun baxter_examples baxterJointCommandAndRead.py
}

case $1 in 
  'sim')
    StartBaxter
  ;;
  'enable')
    EnableBaxter
  ;;
  'ws')
    Workspace
  ;;
  'link')
    SymLink
  ;;
  'run')
    RunBaxterJoint
  ;;
  *)
    echo 'Options:'
    echo 'sim       - setup the simulator for baxter'
    echo 'enable    - enable baxter, setup channels and run baxterJoint file'
    echo 'ws        - enter baxter workspace'
    echo 'link      - symbolic link into baxter_example'
    echo 'run       - run baxter joint command and read file'
  ;;
esac

