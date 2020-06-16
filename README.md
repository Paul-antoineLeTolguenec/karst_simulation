# karst_simulation
Modeling of an exploration robot evolving in a karst

## Introduction
Within the framework of the Karstic exploration project undertaken by LIRMM, it was necessary to set up a simulation to test a new SLAM method: Interval analysis.
This github deposit allows the simulation of an automnome robot evolving in a Karst.
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
