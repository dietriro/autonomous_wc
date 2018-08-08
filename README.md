# Research: Autonomous Wheelchair

## Project Description

This repository includes an experience-based path planner which has been developed as part of a project with the goal of creating a fully autonomous wheelchair at Oregon State University. The planner is a modified A* algorithm operating on 3D grid-maps. The third dimension is a discretized representation of the robots orientation. The values for the map are *learned* by driving the robot around manually and incrementing the cells the robot traverses. The A* algorithm then calculates a path according to this experience (cells with higher values are preferred) instead of the shortest path.
