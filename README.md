# multi_agent_project

Dependencies:
Gazebo
Turtlebot 3

To install the turtlebot3 model: 
put turtlebot3_burger_gripper.urdf.xarco and turtlebot3_burger_gripper.gazebo.xarco in the turtlebot3_model/turtlebot3/turtlebot3_description/urdf folder.

To run the multi_agent_avoid package: 
roslaunch multi_agent_avoid run.launch

To run the multi_agent_cluster package:
start gazebo roslaunch multi_agent_cluster gazebo.launch
start the robots roslaunch multi_agent_cluster run.launch

To change the amount of objects in the environment the world file will need to be edited namely under the population tag and the model count.

<population name="box_population1">
  <model_count>30</model_count>
</population>

The world file is located in the multi_agent_avoid/worlds folder and is called turtlebot3_stage_prime.world.
