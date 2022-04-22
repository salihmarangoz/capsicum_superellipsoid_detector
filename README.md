# Capsicum Superellipsoid Detector (ROS1)

![demo](imgs/demo.gif)

**Table of Contents**

* [Introduction](#introduction)
* [Installation](#installation)
   * [Related Packages](#related-packages)
   * [Package Dependencies](#package-dependencies)
   * [Compile](#compile)
* [Running](#running)
* [ROS Topics, Transforms, and Services](#ros-topics-transforms-and-services)
   * [Parameters](#parameters)
   * [Subscribed Topics](#subscribed-topics)
   * [Published Topics](#published-topics)
   * [Transforms](#transforms)
   * [Services](#services)
* [Future Work](#future-work)

Author: Salih Marangoz - s6samara@uni-bonn.de

Many thanks to [Tobias Zaenker](https://www.hrl.uni-bonn.de/Members/tzaenker/tobias-zaenker) and [Prof. Dr. Maren Bennewitz](https://www.hrl.uni-bonn.de/Members/maren) for the opportunity and help with the project of [HRL](https://www.hrl.uni-bonn.de/).

## Introduction

Autonomous crop monitoring is a difficult task due to the complex structure of plants. Occlusions from leaves can make it impossible to obtain complete views about all fruits of plants (e.g. Capsicum). Therefore, accurately estimating the shape and volume of fruits from partial information is crucial to enable further advanced automation tasks such as yield estimation and automated fruit picking. In this work, we present an approach for faster and better estimating shapes of fruits by fitting superellipsoids. The ROS node applies; 

- Euclidean Clustering to the input point cloud for fruit separation,
- Computes surface normals then estimates fruit centers with the least-squares intersection of lines approach,
- Matches superellipsoids to the clustered points with a non-linear least-squares approach. Also, some priors are used (estimated center, superellipsoid scaling constraints),
- Predicts missing surfaces on a fruit. This step is done by uniform-like sampling of the superellipsoid surface and then only selecting sampled points having the closest distance to data points higher than the threshold.

[Youtube video link](https://www.youtube.com/watch?v=kX0oy-pKSh4) for the demo of this project: 

[![](https://img.youtube.com/vi/kX0oy-pKSh4/0.jpg)](https://www.youtube.com/watch?v=kX0oy-pKSh4)

Meeting notes (for HiWi) can be found [here](MEETING_NOTES.md).

Some prototypes and experiments (implemented in Python) can be found below. Selected ideas are already implemented in C++:

| Related File                                                 | Description                                                  |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| [detector_prototype.py](scripts/detector_prototype.py)       | First prototype Python code with ROS1 support. New ideas are not implemented here. |
| [simulate_depth_noise.py](scripts/simulate_depth_noise.py)   | A simple code for adding noises (gaussian noise, shadowing effect, salt and pepper, waves) to the pointcloud data obtained from Gazebo simulator. |
| [optimization.ipynb](notebooks/optimization.ipynb)           | Least-Squares optimization for fitting superellipsoid to partial pointcloud. |
| [intersection_of_lines.ipynb](notebooks/intersection_of_lines.ipynb) | Least-Squares estimation of capsicum centroid using surface normals. |
| [cost_functions.ipynb](notebooks/cost_functions.ipynb)       | Analyzing of different cost functions.                       |
| [find_missing_part_of_spherical_data.ipynb](notebooks/find_missing_part_of_spherical_data.ipynb) | Experiments for finding missing parts of spherical data. Can be extended to superellipsoidical data. (**Note:** Embedded videos may not be properly visualized on Github. I recommend [this](https://chrome.google.com/webstore/detail/jupyter-notebook-viewer/ocabfdicbcamoonfhalkdojedklfcjmf) Chrome extension for opening the notebook.) |
| [superellipsoid_fibonacci_projection_sampling.ipynb](notebooks/superellipsoid_fibonacci_projection_sampling.ipynb) | Uniform-like sampling of superellipsoid surface points.      |



## Installation

### Related Packages

Packages needed for running launch files. Try running `start_bag.launch` if you don't want to install these packages.

- **[voxblox](https://voxblox.readthedocs.io/en/latest/pages/Installation.html)** (3D mapping)

```
$ cd ~/catkin_ws/src
$ mkdir voxblox
$ cd voxblox
$ git clone git@github.com:ethz-asl/voxblox.git
$ wstool init . ./voxblox/voxblox_ssh.rosinstall
```

- [**ur_with_cam_gazebo**](https://github.com/Eruvae/ur_with_cam_gazebo) (simulator)
- [**roi_viewpoint_planner**](https://github.com/Eruvae/roi_viewpoint_planner) (robotic arm planner)
- [**pointcloud_roi**](https://github.com/Eruvae/pointcloud_roi) (alternative for agrobot_mrcnn_ros on simulation environment for mask extraction)
- agrobot_mrcnn_ros (deep learning model for detecting sweet peppers in camera images)

### Package Dependencies

Dependencies needed **only** for compiling and running the node (excluding launch files).

- Ubuntu 20.04 + ROS Noetic

- **[superellipsoid_msgs](https://github.com/salihmarangoz/superellipsoid_msgs)**
- **[octomap_vpp](https://github.com/Eruvae/octomap_vpp)**
  - Only used for publishing **`~superellipsoids_volume_octomap`**. 
  - Also, **[octomap_vpp_rviz_plugin](https://github.com/Eruvae/octomap_vpp_rviz_plugin)** may be useful for visualization.


- **[Ceres Solver](http://ceres-solver.org/installation.html):** (I have developed the project using the version 2.x.x but both versions should be OK.)

  - Apt Installation (version 1.x.x)

  ```bash
  $ sudo apt install libceres-dev
  ```

  - Manual Installation (version 2.x.x)

  ```bash
  $ sudo apt-get install cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev
  $ cd ~/catkin_ws # or another path... don't delete the folder afterwards to be able to uninstall it
  $ git clone https://github.com/ceres-solver/ceres-solver.git
  $ cd ceres-solver
  $ mkdir build
  $ cd build
  $ cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DBUILD_SHARED_LIBS=ON
  $ make -j8
  $ sudo make install # run "sudo make uninstall" for uninstalling if needed
  ```

- Others: Octomap, PCL, etc. Defined in `package.xml`.

```bash
$ cd catkin_ws/
$ rosdep install --from-paths src --ignore-src -r # install missing dependencis
```

### Compile

```bash
$ cd catkin_ws/
$ catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Running

For **running** the node use the following command:

```bash
$ $ roslaunch capsicum_superellipsoid_detector start_bag.launch # for a quick demo
# OR
$ roslaunch capsicum_superellipsoid_detector start_sim.launch # for simulation
# OR
$ roslaunch capsicum_superellipsoid_detector start_real.launch # for real world
```

## ROS Topics, Transforms, and Services

### Parameters

- **`p_cost_type`** **: 2**

  Optimization cost function for matching superellipsoids to the input points. 

  Note: `RADIAL_EUCLIDIAN_DISTANCE` and `SOLINA` works pretty well. Others have several problems.

  ![f](imgs/formulas/f_.png)

  - `CostFunctionType::NAIVE = 0`

  ![naive](imgs/formulas/naive_.png)

  - `CostFunctionType::LEHNERT = 1`

  ![lenhert](imgs/formulas/lenhert_.png)

  - `CostFunctionType::RADIAL_EUCLIDIAN_DISTANCE = 2`

  ![radial](imgs/formulas/radial_.png)

  - `CostFunctionType::SOLINA = 3`

  ![solina](imgs/formulas/solina_.png)

  - `CostFunctionType::SOLINA_DISTANCE = 4`

  ![solina_dist](imgs/formulas/solina_dist_.png)

- **`p_prior_center`: 0.1**

  ![prior_center](imgs/formulas/prior_center_.png)

  - Alpha value for the prior which enforces optimized center **t** and center estimated via surface normals **p** to be close to each other. Higher values increases the regularization.

- **`p_prior_scaling`: 0.1**

  ![prior_scaling](imgs/formulas/prior_scaling_.png)

  - Beta value for the prior which enforces **a**, **b**, **c** values which defines scaling of superellipsoid to be close to each other. Higher values increases the regularization. 

- **`p_missing_surfaces_num_samples`: 300**

  - Number of points for sampling with projected fibonacci sphere method for missing surface points prediction.

- **`p_missing_surfaces_threshold`: 0.015**

  - In meters. Points sampled with projected fibonacci sphere method are compared to the input data points. If the distance is higher than the threshold sampled point is marked as a missing surface point.

- **`p_min_cluster_size`: 100**
  
  - Discards clusters smaller than **p_min_cluster_size**.
  
- **`p_max_cluster_size`: 10000**

  - Discards clusters larger than **p_max_cluster_size**.

- **`p_max_num_iterations`: 100**

  - Maximum number of non-linear least-squares optimization.

- **`p_cluster_tolerance`: 0.01**

  - In meters. sGroups two points having smaller distance than **p_cluster_tolerance** into the same cluster.

- **`p_estimate_normals_search_radius`: 0.015**

  - In meters. Uses points closer than **p_estimate_normals_search_radius** for normal vector computation.

- **`p_estimate_cluster_center_regularization`: 2.5**

  - Regularization for intersection of lines computation. Defines bias towards mean of cluster points. Higher values brings the result towards the bias point. Useful when there are not enough surfaces.

- **`p_pointcloud_volume_resolution`: 0.001**

  - In meters. Resolution of **~superellipsoids_volume** message.

- **`p_octree_volume_resolution`: 0.001**

  - In meters. Resolution of **~superellipsoids_volume_octomap** message.

- **`p_print_ceres_summary`: false**

  - Prints cost, gradients, extra information, etc. for each optimization step.

- **`p_use_fibonacci_sphere_projection_sampling`: false**

  - This only affects the the output of **~superellipsoids_surface** message. I personaly find surface sampled with parametric representation easier to perceive.
  - If true, uses our approach for uniform-like sampling of superellipsoid .
  - If false, uses parametric representation which is not uniform-like.
  - See [superellipsoid_fibonacci_projection_sampling.ipynb](notebooks/superellipsoid_fibonacci_projection_sampling.ipynb) for the comparsion.

- **`p_world_frame`: "world"**

  - World transform frame.

### Subscribed Topics

**`~pc_in`** ("sensor_msgs/PointCloud2")

- RGBXYZ pointcloud as the input (e.g. voxblox output can be used as the input). Currently RGB information is not used. Some modifications may be needed to feed XYZ only pointcloud.

### Published Topics

- **`~superellipsoids`** ("superellipsoid_msgs/SuperellipsoidArray")
  - Optimized superellipsoids output. Headers are the same for all superellipsoids.
- **`~clusters`** ("sensor_msgs/PointCloud2")
  - RGBXYZ pointcloud with each cluster has a different RGB color. Colors may be changed between messages for the same clusters. Recommended only for debugging.
- **`~superellipsoids_surface`** ("sensor_msgs/PointCloud2")
  - XYZ pointcloud for the surface of superellipsoids.
- **`~centers_prior`** ("sensor_msgs/PointCloud2")
  - XYZ pointcloud for predicted centers via only using surface normals. Recommended only for debugging.
- **`~centers_optimized`** ("sensor_msgs/PointCloud2")
  - XYZ pointcloud for centers computed after the superellipsoid optimization. Centers for failed optimizations will not be published. Recommended only for debugging.
- **`~superellipsoids_volume`** ("sensor_msgs/PointCloud2")
  - XYZ pointcloud for the volume of superellipsoids. The volume is sampled uniform with a fixed resolution and then all points are transformed to the real position. 
- **`~superellipsoids_volume_octomap`** ("octomap_msgs::Octomap")
  - XYZ octomap_vpp::CountingOcTree for the volume of superellipsoids. The volume is sampled uniform with a fixed resolution and then all points are transformed to the real position. The count value represents cluster index of the superellipsoid.
- **`~surface_normals_marker`** ("visualization_msgs::MarkerArray")
  - Arrow markers for visualizing surface normals. Surface normals are computed w.r.t. predicted cluster center. Only recommended for debugging. Use `~xyz_label_normal` for further processing.
- **`~xyz_label_normal`** ("sensor_msgs/PointCloud2")
  - XYZLNormal pointcloud for all clustered points. Labels are indicating cluster indexes and normals are computed w.r.t. predicted cluster center. 
- **`~missing_surfaces`** ("sensor_msgs/PointCloud2")
  - XYZLNormal pointcloud representing (estimated) missing data points on an superellipsoid. Labels are indicating cluster indexes and normals are directed towards the optimized center. 

### Transforms

- `world` -> Input Pointcloud Frame

### Services

There are no services for the written node. But voxblox needs a `std_srvs/Empty` for publishing pointclouds of mapped plants which will be also triggering the computation of the superellipsoid detector node. Currently this task is assigned to `scripts/trigger_voxblox.py` which calls the related service in a fixed interval.



## Future Work

- Accessing to voxblox mesh (vertices and normas) directly would be better. This can take away the need to estimate normals. But this needs some workarounds and code modifications in voxblox.
- Better clustering / instance segmentation.
- 3D mapping with Instance segmentation. Mapping with masking pointcloud impacts the quality.
- Use of surface normals instead of a single estimated center in the optimization process. This may work better for non-sphere like capsicums.
- Sometimes capsicums may have weird shapes (not like a sphere nor superellipsoid, not symmetrical, etc.). Combination of multiple superellipsoids for modeling the fruit surface would be better. On the other hand, estimating the missing parts of the shape becomes difficult this way.
- Loss functions can be used against outliers: http://ceres-solver.org/nnls_modeling.html#lossfunction
