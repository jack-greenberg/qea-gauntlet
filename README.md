# QEA presents NEATO: The Gauntlet

Jack Greenberg

<details>
    <summary style="font-weight: bold;">Table of Contents</summary>
    <ul>
        <li><a href="#the-hero">The Hero</a></li>
        <ul>
            <li><a href="#the-challenge">The Challenge</a></li>
            <li><a href="#the-world">The World</a></li>
        </ul>
        <li><a href="#some-graphs">Some Graphs</a></li>
        <li><a href="#some-code">Some Code</a></li>
        <ul>
            <li><a href="#ransac">RANSAC</a></li>
            <li><a href="#gradient-descent">Gradient Descent</a></li>
        </ul>
    </ul>
</details>



## The Challenge

The objective of this task is to get the NEATO to navigate a playpen until it reaches the "Barrel of Benevolence" (a cylinder), while avoiding boxes.

### The Hero

<p align="center"><img src="graphics/neato-blank.png" width="50%" /></p>

The NEATO is a differential drive, two-wheeled robot that was simulated using a series group of docker containers, and interfaced with using the ROS (Robotic Operating System) toolbox in MATLAB.

### The World

<p align="center"><img src="graphics/gauntlet.png" width="50%" /></p>



## Some Graphs

<p align="center"><img src="graphics/pen-map.png" /></p>

<p align="center">Map of the Gauntlet as seen by the NEATO's LIDAR scanner, with features detected by the RANSAC algorithm.</p>



<p align="center"><img src="graphics/contour-map.png" /></p>

<p align="center">Gauntlet map with contour lines shown.</p>



<p align="center"><img src="graphics/vector-map.png" /></p>

<p align="center">Gauntlet map with vector field shown.</p>



<p align="center"><img src="graphics/planned-path.png" /></p>

<p align="center">Planned path of the NEATO at the beginning. This will update as the NEATO progesses forward and generates new LIDAR scans.</p>



<p align="center"><img src="graphics/NEATO-path.png" /></p>

<p align="center">Final path of the NEATO.</p>



<p align="center"><img src="graphics/composite-plot.png" /></p>

<p align="center">Final composite map of the Gauntlet in the global frame (NEATO's odometry frame, with origin at (0,0) and x and y axes.</p>



## The Code

### RANSAC

Coming soon...

### Gradient Descent

See above...
