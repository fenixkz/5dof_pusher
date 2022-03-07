# Reinforcement learning environment - 5 degree-of-freedom planar robot pushing an object.

### Project overview
This project contains the neccessary files to run the reinforcement learning (RL) experiments. The environment is designed with the OpenAI Gym guideline. 
The objective of the experiment is to push the box to the maximum distance without losing a contact with it. Everything is spawned in Gazebo simulation engine. 

Initial configuration - 
<p align="center">
  <img width="640" height="480" src="https://github.com/fenixkz/5dof_pusher/blob/main/pics/pusher_ex.png">
</p>

Goal (desired) configuration - 
<p align="center">
  <img width="640" height="480" src="https://github.com/fenixkz/5dof_pusher/blob/main/pics/pusher_ex1.png">
</p>


### Requirements
This project is dependable on different libraries. Please, be sure to install them before trying to run this project. This branch was tested and implemented on ROS Noetic. For other ROS distributions some adjustement must be made beforehand. 
  - MoveIt
    `$ sudo apt-get install ros-<your_distro>-moveit`
  - ros-controllers
    `$ sudo apt-get install ros-noetic-effort-controllers`
  - openai-gym
    `$ pip3 install gym`
  - matplotlib
    `$ pip3 install matplotlib`
  - yaml 
    `$ pip3 install pyyaml`
  - tensorflow
    `$ pip3 install tensorflow`
  - keras
    `$ pip3 install keras`

### Running
To run the project, first compile the workspace by executing `catkin_make` or `catkin build`. After that, open two terminals and in the first one launch:<br/>
`roslaunch gazebo_robot gazebo.launch` <br/>
In the second:
```
$ cd /gazebo_pusher
$ python3 train_dqn.py
```
After that, the robot starts the training.

### Some results
To evaluate the results, at the end the graph is plotted showing the distance the box has moved and the total reward. One of the obtained results - 

<p align="center">
  <img width="1600" height="900" src="https://github.com/fenixkz/5dof_pusher/blob/main/pics/ddqn8layers07512.png">
</p>
