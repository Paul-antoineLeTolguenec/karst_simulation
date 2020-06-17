# karst_simulation
Modeling of an exploration robot evolving in a karst

## Introduction
Within the project of the Karstic exploration undertaken by LIRMM, it was necessary to set up a simulation to test a new SLAM method using Interval analysis.
This github repository allows the simulation of an automnome robot evolving in a Karst.
I used the gazebo simulator because it allows to be interfaced with ROS which is a tool widely used in robotics.
This simulation was possible thanks to the work of:
<blockquote><p>
@inproceedings{Manhaes_2016,
    doi = {10.1109/oceans.2016.7761080},
    url = {https://doi.org/10.1109%2Foceans.2016.7761080},
    year = 2016,
    month = {sep},
    publisher = {{IEEE}},
    author = {Musa Morena Marcusso Manh{\~{a}}es and Sebastian A. Scherer and Martin Voss and Luiz Ricardo Douat and Thomas Rauschenbach},
    title = {{UUV} Simulator: A Gazebo-based package for underwater intervention and multi-robot simulation},
    booktitle = {{OCEANS} 2016 {MTS}/{IEEE} Monterey}
}
</p></blockquote>

## Gazebo 
All the physics and visuals run essentially on gazebo

[![video](https://github.com/Paul-antoineLeTolguenec/karst_simulation/blob/master/doc/video/simu_gazebo.gif)]

## RVIZ
RVIZ allows the display of the trajectory, the odometry, the camera view and even the mapping made thanks to sonars.
It is a very powerful tool for all visualization of sensor data.

[![video](https://github.com/Paul-antoineLeTolguenec/karst_simulation/blob/master/doc/video/simu_RVIZ.gif)]


## Note
In this Markdown, I explain the installation of each element in an exhaustive way because this repository is created in order to bring together people with different backgrounds around the same formalism.
If you have bases in ROS and Gazebo, install this package the way you want.

## Dependencies

* **ROS Melodic** 
  
    First you need to install ROS (I use ROS Melodic but you can install the version you want).

    You need to get ”Desktop-Full Install" so you get Gazebo.

    you can follow the instructions [here](http://wiki.ros.org/melodic/Installation/Ubuntu)

* **WorkspaceRos** 
  
    here is a procedure to configure your ROS workspace:

        mkdir -p ~/workspaceRos/src
        cd ~/workspaceRos
        catkin_make
        echo "source ~/workspaceRos/devel/setup.bash" >> ~/.bashrc

* **UUV Simulator** 

    Now we can install the package allowing the simulation of underwater robots:

        sudo apt install ros-melodic-uuv-simulator

    Or if you prefer : [Installation from source](https://uuvsimulator.github.io/installation/)

* **The Karst** 

    The package developed by the research team is very powerful because it has many features.
    One of them is that you can create your own world, so that you can model it as closely as possible to your own experience.
    I made my own world in order to recreate a karst.

    To download it you need to go [there](https://github.com/Paul-antoineLeTolguenec/uuv_cave_world) 

* **The Simulation with the autonomous robot**

    To make the robot autonomous I had to edit the file: waypoints.yaml.
    Indeed, this file contains different points contained in the cave and the robot follows these points in the order written in the file. So if you create a new cave you'll have to redo a waypoint file.

    To get the simulation go to the terminal and run :

        cd ~/workspaceRos/src
        git clone https://github.com/Paul-antoineLeTolguenec/karst_simulation.git
        cd ~/workspaceRos
        catkin_make

## Run the simulation
Now that you have all the necessary elements for the simulation, you can run it.
Go to the terminal and run :

        roslaunch karst_simulation simulation.launch



