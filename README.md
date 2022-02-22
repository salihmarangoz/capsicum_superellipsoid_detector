# capsicum_superellipsoid_detector

[toc]

**Salih Marangoz - s6samara@uni-bonn.de - salih285@gmail.com**

## Meeting Notes

Can be found [here](MEETING_NOTES.md).

## Installation

### Workspace Packages

TODO

### Dependencies

- Voxblox (See for more: https://voxblox.readthedocs.io/en/latest/pages/Installation.html)

```bash
$ cd ~/catkin_ws/src
$ mkdir voxblox
$ cd voxblox
$ git clone git@github.com:ethz-asl/voxblox.git
$ wstool init . ./voxblox/voxblox_ssh.rosinstall
```

- Ceres Solver (Both versions should be OK. But I have developed the project using the version 2.x.x)

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

- Others

```bash
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src -r # for other dependencis
```

### Compile

```bash
$ cd catkin_ws/
$ catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Backup Forks

In case of projects if disappear.

- https://github.com/salihmarangoz/voxblox
- https://github.com/salihmarangoz/perception_pcl
- https://github.com/salihmarangoz/OpenChisel (not needed for current version)
- https://github.com/salihmarangoz/DirectionalTSDF (not needed for current version)

## Running & Debugging

For **running** the node run the following command:

```bash
$ roslaunch capsicum_superellipsoid_detector start_sim.launch
# OR
$ roslaunch capsicum_superellipsoid_detector start_real.launch
```

For **debugging**, there are two lines in `CMakeLists.txt` which enable AddressSanitizer and add debug symbols. For detecting heap corruptions, stack overflow, etc this method would be better. Also, AddressSanitizer slows the application by 2x, while the performance impact is 10x with GDB. If you can't see the line number in the AddressSanitizer output, some dependencies should be installed:

```bash
$ sudo apt install clang llvm  # note: clang may not be revelant
```



## ROS Topics, Transforms, and Services

### Subscribed Topics

**`~pc_in`** ("sensor_msgs/PointCloud2")

- RGBXYZ pointcloud as the input (e.g. Voxblox output can be used as the input). Currently RGB information is not used. Some modifications may be needed to feed XYZ only pointcloud.

### Published Topics

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
- **`~surface_normals_marker`** ("visualization_msgs::MarkerArray")
  - Arrow markers for visualizing surface normals. Surface normals are computed w.r.t. predicted cluster center. Only recommended for debugging. Use `~xyz_label_normal` for further processing.
- **`~xyz_label_normal`** ("sensor_msgs/PointCloud2")
  - XYZLNormal pointcloud for all all clustered points. Labels are indicating cluster indexes and normals are computed w.r.t. predicted cluster center. 

### Transforms

- `world` -> Input Pointcloud Frame

### Services

There are no services for the written node. But Voxblox needs a `std_srvs/Empty` to publish pointclouds of mapped plants which is also triggering the computation of the Superellipsoid Detector node. Currently this task is assigned to `scripts/trigger_voxblox.py` which calls the related service in a fixed interval.
