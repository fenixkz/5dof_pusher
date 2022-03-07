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
This project is dependable on different libraries. Please, be sure to install them before trying to run this project.
  - MoveIt
    '$ sudo apt-get install ros-<your_distro>-moveit'  
  - ros-controllers
$ sudo apt-get install ros-noetic-effort-controllers
openai-gym
pip install gym
matplotlib
pip install matplotlib
yaml 
pip install pyyaml
tensorflow
pip3 install tensorflow
keras
pip3 install keras

