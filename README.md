# ICP Registration Project

This repository contains the code necessary to perform Iterative Closest Point (ICP) registration between two point clouds using Python and the Open3D library. The ICP algorithm iteratively minimizes the difference between the two point clouds by finding the optimal rigid-body transformation (rotation and translation).

## Repository Structure

The repository consists of two main files:

- `main.py`: This script executes the ICP algorithm, aligning two point clouds and visualizing the results.
- `utils.py`: This file includes helper functions used by `main.py` to perform various tasks such as finding correspondences, computing the distance between point clouds, and performing Procrustes analysis.

## Dependencies

To run the code in this repository, you will need the following:

- Python 3.x
- NumPy
- Open3D

You can install the necessary libraries using `pip`:

`pip install numpy open3d`

## How to Run
To execute the ICP registration, simply run the main.py script:

`python main.py`

## Visualization
This code uses Open3D's visualization capabilities to display the point clouds before and after the ICP registration process. The target point cloud is colored red, and after alignment, the source point cloud is displayed in its original color.

## Notes
Ensure you have the latest version of Open3D installed, as older versions may not support some functionalities used in this code.
The termination threshold for the ICP algorithm is set to 0.001. This can be adjusted in the main.py script depending on the desired precision.
