# QEA presents NEATO: The Gauntlet

Jack Greenberg

<details>
    <summary style="font-weight: bold;">Table of Contents</summary>
    - [The Hero](#the-hero)
    	- [The Challenge](#the-challenge)
    	- [The World](#the-world)
    - [Some Graphs](#some-graphs)
    - [The Code](#the-code)
    	- [RANSAC](#RANSAC)
    	- [Gradient Descent](#gradient-descent)
</details>



## The Challenge

The objective of this task is to get the NEATO to navigate a playpen until it reaches the "Barrel of Benevolence" (a cylinder), while avoiding boxes.

### The Hero

<img src="neato-blank.png" width="50%" />

The NEATO is a differential drive, two-wheeled robot that was simulated using a series group of docker containers, and interfaced with using the ROS (Robotic Operating System) toolbox in MATLAB.

### The World

<img src="gauntlet.png" width="50%" />



## Some Graphs

<img src="./pen-map.png" />

<p align="center">Map of the Gauntlet as seen by the NEATO's LIDAR scanner, with features detected by the RANSAC algorithm.</p>



<img src="./contour-map.png" />

<p align="center">Gauntlet map with contour lines shown.</p>



<img src="./vector-map.png" />

<p align="center">Gauntlet map with vector field shown.</p>



<img src="./planned-path.png" />

<p align="center">Planned path of the NEATO at the beginning. This will update as the NEATO progesses forward and generates new LIDAR scans.</p>



## The Code

### RANSAC

Coming soon...

### Gradient Descent

See above...