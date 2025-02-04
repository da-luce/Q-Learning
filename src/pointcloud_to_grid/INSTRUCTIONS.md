# Launching Lidar and RViz for Occupancy Grid in ROS

Follow these steps to build and launch the occupancy grid from the converted point cloud of the lidar:

### Code
```bash
cd ~/catkin_ws
catkin build pointcloud_to_grid
source devel/setup.bash
roslaunch pointcloud_to_grid occupancy_grid.launch
```

This setup should successfully launch the lidar and visualize the occupancy grid from the converted point cloud in RViz.