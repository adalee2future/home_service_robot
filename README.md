## Home Service Robot

### Run
```
bash src/scripts/home_service.sh
```

```
# launch my world and robot
roslaunch my_robot world.launch 

# amcl package to localize the robot
## map file is located in src/map/map.yaml, which is generate by my privious project "Map My World"
roslaunch my_robot amcl.launch

# rviz
rosrun rviz rviz -d src/rvizConfig/navigation.rviz

# add markers and pick
rosrun pick_objects pick_objects
rosrun add_markers add_markers

```

### Steps

1. use gazebo to build my world (`ada.world`) and robot (`my_robot.xacro`)
2. launch my world and robot (`roslaunch my_robot world.launch`)
3. create a map
  * start `rtabmap` (`roslaunch my_robot mapping.launch`)
  * run `teleop_twist_keyboard` node (`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`)to drive to the robot around to get rgbd images
     * set velocity small could help make better map
     * make the paths for turns as similar as possible
     * turn 360 degree at fixed position would be helpful to make better map
     * exit rtabmap node to generate a 3D map DB
     * transfer 3D map DB to 2D map (`map.yaml`)
4. localization (`roslaunch my_robot acml.launch`)
  * start `map_server` node to provide a map service
  * start `amcl` node to localize
    * notes for tuning amcl parameters
      * set correctly for initial pose
      * set correctly for the laser scanner product
      * make `update_min_d` and `update_min_a` smaller to let to particle cloud converge faster
5. use `rviz` to visualize (`navigation.rviz`)
  * RobotModel
  * LaserScan
  * PointCloud2
  * Map
  * Group: Global Map
    * Map
    * Path
  * Group: Local Map
    * Map
    * Path
    * PointCloud2
  * PoseArray
  * Path
  * Marker
  * MoveCamera
* pick_objects
  * send target navigation goals to `rviz` to navigate
* add_markers
  * visualize a box marker and subscribe `/amcl_pose` topic to check if the robot is near the box or  drop off zone
