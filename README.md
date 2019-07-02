# obstacle-avoidance

To run:

(

Move octomap_server provided to octomap_mapping

source ~/.bashrc

catkin build

)

roslaunch robot_control motion_computation.launch

rosservice call /mavros/cmd/arming "value: true" ; rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'offboard'"

Doesn't work with python 3.

publish goal location, for example:

rostopic pub /goal_position geometry_msgs/Vector3 "x: 6.0
y: 0.0
z: 1.0" -r 20
