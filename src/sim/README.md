1. `docker compose up`
2. `docker exec -it --user root autobike_dev bash`
3. `build`
4. `ros2 launch sim robot_launch.py`
5. Give Webot a minute
6. Enable "Use Rosetta for x86/amd64 emulation on Apple Silicon", otherwise it is horribly slow (still is extremely slow)
7. `docker exec -it --user root autobike_dev bash` for another terminal
8. `ros2 topic pub /cmd_vel geometry_msgs/Twist  "linear: { x: 0.1 }"`

Following THIS tutorial!

<https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html>

other versions of ROS WILL NOT WORK
