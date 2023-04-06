**Simulation world**<br><br>
apartment.launch _# simulation for a house with tables and cabinets_<br>
bigscene.launch _# simulation for big indoor environment_<br>
simple_world.launch _# simulation for simple indoor environment_<br>
traffic_world.launch _# simulation for factory with one-way traffic rule_<br>
vslam_world.launch _# simulation for visual slam in a house with many pictures_<br>

### vslam_world.launch needs to change environment variables, take a look at script/vsim.sh<br>

### For ackermann model, install following package:<br>
sudo apt install ros-melodic-ros-control<br>
sudo apt install ros-melodic-ros-controllers<br>
sudo apt install ros-melodic-ackermann-msgs<br>