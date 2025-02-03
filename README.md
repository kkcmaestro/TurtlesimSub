# TurtlesimSub
 Assingment submission
This files contains all the code for assigments submissions.

After sourcing everything and starting turtlesim_node, follow these steps to run code for each result.

Goal 1:

Use this command:

ros2 run pid_controller spawn_node 

This would spawn a new turtle in the env which can be controlled by /turtle1/cmd_vel publisher.
This node corresponds to the Spawn.py file

To move the turtle to a desired location run.

ros2 run pid_controller pid_controller_node.

enter the desired X and Y values and it would find the fastest way to reach there.
This corresponds to the pid_controller.py file.

Goal 2:

To acheive the deceleration effect first run a cmd_vel publisher (use circle_node)

ros2 run pid_controller circle_node 

and in another terminal

ros2 run pid_controller deceleration_node 

this corresponds to (Deceleration_node.py)
When you interupt the first circle_node using ctrl+c ther deceleration node will gradually decrease the speed and bring it to a halt.

Goal 3:

Run the command 

ros2 run pid_controller circle_node 

corresponds to Circles.py

Goal 4:

For this to work well first you have to use these commands
open new terminal (start new turtlesim_node)

ros2 run pid_controller pid_controller

enter values X = 5,  Y = 5

let it move to that location

then run 

ros2 run pid_controller circle_node 

wait for it to form one full circle
now run

ros2 run pid_controller police_turtle_node

this corresponds to Police_turtle.py (*not Police_turle)

Goal 5:

Do the same steps as Goal4 but use this as the last command (instead of ros2 run pid_controller police_turtle_node)

ros2 run pid_controller half_vel_node

corresponds to PT_RT_hald_vel.py

Goal 6:

Same steps as Goal four and use this as the last command (instead of ros2 run pid_controller police_turtle_node)

ros2 run pid_controller rt_pt_noise_node

corresponds to PT_RT_half_vel.py