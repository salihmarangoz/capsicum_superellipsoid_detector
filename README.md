# capsicum_superellipsoid_detector

toc todo

**Salih Marangoz - s6samara@uni-bonn.de**

## Introduction

**TODO**

- example bag file
- ros parameters
- list contributions?
- experimental clustering?
- missing parts of the fruit
- fix the problem with simulator launch file
- clean git history from pdf and ipynb files.

**TODO**

- Meetings notes can be found [here](MEETING_NOTES.md).

- Prototypes and experiments (implemented in Python) can be found below:

| Related File                                                 | Description                                                  |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| [detector_prototype.py](scripts/detector_prototype.py)       | First prototype Python code with ROS1 support. New ideas are not implemented here. |
| [optimization.ipynb](notebooks/optimization.ipynb)           | Least-Squares optimization for fitting superellipsoid to partial pointcloud. |
| [intersection_of_lines.ipynb](notebooks/intersection_of_lines.ipynb) | Least-Squares estimation of capsicum centroid using surface normals. |
| [cost_functions.ipynb](notebooks/cost_functions.ipynb)       | Analyzing of different cost functions.                       |
| [find_missing_part_of_spherical_data.ipynb](notebooks/find_missing_part_of_spherical_data.ipynb) | Experiments for finding missing parts of spherical data. Can be extended to superellipsoidical data. |
| [superellipsoid_fibonacci_projection_sampling.ipynb](notebooks/superellipsoid_fibonacci_projection_sampling.ipynb) | Uniform-like sampling of superellipsoid surface points.      |



## Installation

### Related Packages

Packages needed for running launch files.

- **[voxblox](https://voxblox.readthedocs.io/en/latest/pages/Installation.html)**

```
$ cd ~/catkin_ws/src
$ mkdir voxblox
$ cd voxblox
$ git clone git@github.com:ethz-asl/voxblox.git
$ wstool init . ./voxblox/voxblox_ssh.rosinstall
```

- [**ur_with_cam_gazebo**](https://github.com/Eruvae/ur_with_cam_gazebo)
- [**roi_viewpoint_planner**](https://github.com/Eruvae/roi_viewpoint_planner)
- agrobot_mrcnn_ros

### Package Dependencies

Dependencies needed **only** for compiling and running the node (excluding launch files).

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

### Compile

```bash
$ cd catkin_ws/
$ rosdep install --from-paths src --ignore-src -r # install missing dependencis
$ catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Running

For **running** the node use the following command:

```bash
$ roslaunch capsicum_superellipsoid_detector start_sim.launch # for simulation
# OR
$ roslaunch capsicum_superellipsoid_detector start_real.launch # for real world
```

## ROS Topics, Transforms, and Services

### Parameters

todo

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
  - XYZLNormal pointcloud for all all clustered points. Labels are indicating cluster indexes and normals are computed w.r.t. predicted cluster center. 

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
