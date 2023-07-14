# travis
Indoor autonomous rover

---

## Startup inctructions

* Open a terminal window.
* Type and run `base`` to start lidar, rosserial_esp32, odometry, etc.
* Wait for 20 seconds.
* Open another terminal window.
* Tyep and run `travis` to start SLAM and autonomous navigation.
* Wait for rviz to open up
* select nav2 goal button from the top tools bar(click on the drop down arrow on the top right if the nav2 goal button is not visible)
* give it a goal on the map and the robot should plan path and move to the goal while avoiding obstacles in real time.