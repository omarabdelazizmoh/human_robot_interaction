# Repository for Human Robot Interaction based Project

- NOTE: Robot's approaching speeds within human's comfort zones should fall in the range of 0.254 m/s and 0.381 m/s.
- Average human speed is 1 m/s.

## Current Development
- **actor_to_occupancyGrid.py** : Add Dynamic human motion as occupied cells in static free map-> code is written debugging is needed
### Test it
-   Mapping Occupancy Grid from Human Location
    - Node is currently incomplete and publishes static Map
    ```
    roslaunch human_robot_interaction human_behavior_with_husky.launch test:=true

    ```
    ```
    rosrun human_robot_interaction actor_to_occupancyGrid.py
    ```
    - Adding dynamic objects motion
    
    This node subscribes to /gazebo/model_states to obtain human position information. This node publishes the current human position to the OccupancyGrid.

## Working Features
- To bring human and husky in simulation
    - Passing Scenario
    ```
    roslaunch human_robot_interaction human_behavior_with_husky.launch passing:=true

    ```
    - Crossing Scenario
    ```
    roslaunch human_robot_interaction human_behavior_with_husky.launch crossing:=true

    ```
    - Overtaking Scenario
    ```
    roslaunch human_robot_interaction human_behavior_with_husky.launch overtaking:=true

    ```


## Installations
```
sudo apt-get install ros-noetic-husky-*
sudo apt-get install ros-noetic-navigation
sudo apt-get install ros-noetic-gazebo-ros
sudo apt-get install ros-noetic-gazebo-plugins
```

## Files Explaination ( OLD )

Run the launch file "human_behavior_detection.launch". This launch file runs the following:

1. Creates an empty world.
2. Spawns the husky robot into the empty world.
3. Runs the "human_move.py" node.
4. Runs the "state_recognition.py" node.
5. Runs the "robot_interaction.py" node.

