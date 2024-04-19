# Terrain-Detection-for-a-Quadruped-Robot-using-Point-Cloud
The algorithm designed to enable quadruped robots with the ability to autonomously detect complex terrains and identify non-steppable regions of grassy surfaces and water drain grills on road.

# Terrain Understanding for Robotics

## Overview

This project provides a proof of concept for terrain understanding, specifically designed for robotics applications. It focuses on identifying "steppable" surfaces in point clouds generated from terrain images captured by downward-facing cameras on robots. The algorithm classifies grated surfaces as flat, steppable regions while identifying grassy areas and water drain grills on roads as non-steppable to prevent the robot from stepping where its feet might sink.

## Installation

This project requires Python 3 and the following Python libraries:
- NumPy
- Open3D
- SciPy

You can install these dependencies using pip:

```bash
pip install numpy open3d scipy
```

## How It Works

The terrain understanding algorithm implemented in this project aims to differentiate between steppable and non-steppable surfaces in environments with varied terrains, such as those containing grassy areas and grated surfaces like water drain grids. The process involves loading point cloud data, estimating point normals, and applying a series of filters based on geometric properties and density. Here's a breakdown of each step:

### 1. Loading the Point Cloud

The algorithm begins by loading a point cloud from a `.npy` file. The point cloud is expected to represent the surface of the terrain captured by downward-facing cameras on a robot. Each point in the cloud includes three-dimensional coordinates representing a small part of the terrain's surface.

### 2. Estimating Normals

Next, the algorithm estimates the normals for each point in the cloud using Open3D's hybrid search parameter. Normals are essential for understanding the orientation of the terrain's surface at each point, which helps in distinguishing between different types of surfaces.

### 3. Filtering Based on Normals

The algorithm filters points based on the angle of their normals. Steppable surfaces, such as flat terrains and grated surfaces of water drain grids, have normals that are almost perpendicular to the direction of gravity. By contrast, the normals of non-steppable surfaces, like the sides of grassy areas where the toes might sink, significantly deviate from this perpendicular orientation. The filter retains points with normals within a certain threshold of the vertical axis, indicative of potentially steppable surfaces.

### 4. Filtering Based on Height Difference

After filtering by normals, the algorithm examines the height differences between neighboring points. Steppable areas, like flat terrains and grated surfaces, have minimal height variation. The algorithm eliminates points with height differences exceeding a specified threshold, reducing the likelihood of stepping on uneven or sloped surfaces unsuitable for stable footing.

### 5. Density-Based Filtering

The final filtering stage involves analyzing the density of points within a specified window around each point. This step is crucial for identifying surfaces like water drain grids, which, although they might appear steppable due to their flatness and minimal height variation, can be distinguished by the density of points. Grated surfaces will have a high density of points in the window due to the grid's structure, while grassy areas, despite being flat, will have a lower point density that indicates the potential for the robot's toes to sink.

### 6. Visualization of Steppable Surfaces

The algorithm concludes by visualizing the filtered point cloud, now representing only the areas deemed safe to step on. This visualization helps in evaluating the effectiveness of the terrain understanding process and provides insights into the algorithm's decision-making process.

By applying these steps, the algorithm effectively distinguishes between steppable and non-steppable surfaces, enabling robots to navigate more safely and efficiently across complex terrains.

## Result
![image](https://github.com/Humayun-Akhtar/Terrain-Detection-for-a-Quadruped-Robot-using-Point-Cloud/assets/115849836/4c4e3ad5-66bd-4600-bf17-305e0d05484f)
