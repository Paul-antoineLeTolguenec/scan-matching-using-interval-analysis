# scan-matching-using-interval-analysis
This repository contains a basic simulation in C++ allowing to visualize the results of the interval analysis in a relocation problem using the scan matching of a sonar between 2 instants t1 and t2.

## Introduction
The SLAM problem is one of the major problems of underwater robotics.
Indeed, underwater robots must imperatively locate themselves via the IMU and the various sonars.
When the robot moves in a known area, it can identify remarkable positions already known to refine its position (landmarks). 
See an example [here](http://simon-rohou.fr/research/tubex-lib/doc/tutorial/03-static-rangebearing/index.html).
But when the environment is totally unknown, you have to consider the whole environment that you can perceive through sonar.
To simplify the exercise we will do the study in 2D.

## Objective
When a robot moves between a time T1 and T2, it describes an evolution q that can be obtained with a certain precision thanks to the IMU sensor. The problem is that this q-evolution is approximate. 

By integration, the position at T2 is known with a larger error than at T1.
The objective of this exercise is to refine our knowledge of q and thus of the robot position at T2.
On the picture below we can see in red the box including the position at T1 and in green the box including the position at T2.

<p align="center">
    <img src="https://github.com/Paul-antoineLeTolguenec/scan-matching-using-interval-analysis/blob/master/docs/images/objective.png" >
</p>

## Mathematical approach

At each moment the robot carries out a measuring set thanks to the side sonars.
The measurements allow to locate certain points of the cave in the robot's reference frame.

The challenge is to carry out a "scan matching" between a set of data taken at time T1 and a set of data taken at time T2. 
This "scan matching" will allow us to know the q evolution of the robot.
The question we want to answer is:

What was the robot's evolution q between T1 and T2 so that the robot sees the environment that way at T1 and this way at T2 ?

<p align="center">
    <img src="https://github.com/Paul-antoineLeTolguenec/scan-matching-using-interval-analysis/blob/master/docs/images/measuring.png" >
</p>




## Built With

* [ibex-lib](https://github.com/ibex-team/ibex-lib) - Interval Analysis library
* [tubex-lib](https://github.com/SimonRohou/tubex-lib) - Tubex is a C++ library providing tools for computations over sets of trajectories
* [VIBes](https://github.com/ENSTABretagneRobotics/VIBES) - Visualizer for Intervals and Boxes 