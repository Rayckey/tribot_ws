# Tribot

Tribot is a gazebo simulation for a 3 legs robot 

## Installation

```bash
git clone https://github.com/Rayckey/tribot_ws.git
cd tribot_ws
catkin build
source devel/setup.bash 
```

## View single leg robot simulation (needs multiple terminal )

```bash
git checkout 2d
roslaunch singlebot_control singlerot_launch.launch
rosrun singlebot_control singlebot_control.py 
```


## View tribot simulation (needs multiple terminal )

```bash
git checkout tribot
roslaunch trirot_launch.launch
rosrun singlebot_control tribot_control.py
```

## Playing 
Once both the simulation and script are launched, press play on gazebo to begin hopping

## License
Uh