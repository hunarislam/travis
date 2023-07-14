# travis
Indoor autonomous rover

---

## Startup inctructions

### Mapping mode(use only when you need to map a new environment):

* Open a terminal window.
* Type and run `base` to start lidar, rosserial_esp32, odometry, etc.
* Wait for 20 seconds.
* Open another terminal window.
* Type and run `map` to start xbox controller, and gmapping.
* Wait for the rviz to open up.
* Move the robot around using the xbox controller to map the environment for the first time.
* Open up another terminal.
* Type and run `save` to save the map to memory.
* Close everything and move on to autonomous navigation mode.

### Autonomous navigation mode:

* Open a terminal window.
* Type and run `travis` to start lidar, rosserial_esp32, odometry, SLAM and autonomous navigation.
* Wait for rviz to open up
* select nav2 goal button from the top tools bar(click on the drop down arrow on the top right if the nav2 goal button is not visible)
* give it a goal on the map and the robot should plan a path and move to the goal while avoiding obstacles in real-time.