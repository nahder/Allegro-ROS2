# A fun Simon Says game with an Allegro Hand

The Allegro Hand is a robotic hand with four fingers and 16 degrees of freedom. In this project, it is used to play a challenging Simon Says game with a human player. The hand performs increasingly longer sequences of gestures that the player must mimic in the correct order. 

For more details and a longer demo, refer to my [portfolio post](https://nahder.github.io/manipulation_projects/simon_says/).

## Video Demo
<video src="https://github.com/nahder/Allegro-ROS2/assets/71537050/bdaf1176-4fe3-4757-9f8f-dbc4102a4fc1"></video>




## Setup 
These instructions are adapted from [SimLab](https://github.com/simlabrobotics/allegro_hand_linux):

1. Download, build, and install PCAN-USB driver for Linux: libpcan
```
tar -xzvf peak-linux-driver-x.x.tar.gz
cd peak-linux-driver-x.x
make NET=NO
sudo make install
```
2. Download, build, and install PCAN-Basic API for Linux: libpcanbasic
```
tar -xzvf PCAN_Basic_Linux-x.x.x.tar.gz
cd PCAN_Basic_Linux-x.x.x/pcanbasic
make
sudo make install
```

3. Download, build, and install Grasping Library for Linux, "libBHand": Grasping_Library_for_Linux

4. Build Allegro Hand Project using cmake "out of source build" style.
```
unzip AllegroHand.zip
cd AllegroHand
mkdir build
cd build
cmake ..
make
make install
```

Note: Using cmake "out of source build" style, the entire build tree is created under "build" directory so that you can delete "build" directory without worrying about the sources.

## Usage 
Connect PCAN-USB and Allegro Hand while it is powered off. Then, power on the hand.

To run the MoveIt configuration demo and control the hand, run:
```
ros2 launch allegro_moveit_config demo.launch.py
ros2 run control_hand moveit_controller
```
This will enable you to query goal positions within the rviz environment.

To play the game,
`ros2 launch allegro_game start_game.launch.xml`

Have fun!


