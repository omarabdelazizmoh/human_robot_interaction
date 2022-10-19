# human_robot_interaction

**Run the launch file "human_behavior_detection.launch". This launch file runs the following:

1- Creates an empty world.
2- Spawns the husky robot into the empty world.
3- Runs the "human_move.py" node.
4- Runs the "state_recognition.py" node.

---------------------------------------------------------------------------------------------------------------
**The husky robot is by default spawned at the origin, but that can be changed by editing the argument defined in the launch file "human_behavior_detection.launch".

**The "human_move.py" node defines a target position (x,y) for the human to move to, as well as a velocity to move with to that target.

**The "state_recognition.py" node is used to identify the scenario that the human and robot are experiencing at any given moment. These scenarios may either be "passing" or "crossing" scenarios.

