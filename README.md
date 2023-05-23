# Capsicum Superellipsoid Detector (ROS1)

Autonomous crop monitoring is a difficult task due to the complex structure of plants. Occlusions from leaves can make it impossible to obtain complete views of all fruits of plants (e.g. Capsicum). Therefore, accurately estimating the shape and volume of fruits from partial information is crucial to enable further advanced automation tasks such as yield estimation and automated fruit picking. In this work, we present an approach for faster and better estimating the shapes of fruits by fitting superellipsoids.

This ROS package; 

1. Applies Euclidean Clustering to the input point cloud,

2. Computes surface normals to estimate fruit centers with the least-squares intersection of lines approach,

3. Fits superellipsoids to the clustered points with a non-linear least-squares approach.

4. Predicts missing surfaces on a fruit, which may be useful for planning.

Also, some prototypes and experiments (implemented in Python) can be found in the [prototypes](prototypes) folder.

![demo](imgs/demo.gif)



## Paper and Video

Screen recording while running the project can be seen [here](https://www.youtube.com/watch?v=kX0oy-pKSh4).

If using our project for scientific publications, please cite our paper available [here](https://doi.org/10.1109/CASE49997.2022.9926466) and [here](https://doi.org/10.48550/arXiv.2203.15489):

S. Marangoz, T. Zaenker, R. Menon and M. Bennewitz, **"Fruit Mapping with Shape Completion for Autonomous Crop Monitoring,"** *2022 IEEE 18th International Conference on Automation Science and Engineering (CASE)*, 2022, pp. 471-476.

```
@INPROCEEDINGS{marangoz2022fruit,
  author={Marangoz, Salih and Zaenker, Tobias and Menon, Rohit and Bennewitz, Maren},
  booktitle={2022 IEEE 18th International Conference on Automation Science and Engineering (CASE)}, 
  title={Fruit Mapping with Shape Completion for Autonomous Crop Monitoring}, 
  year={2022},
  pages={471-476},
  doi={10.1109/CASE49997.2022.9926466}
}
```




## Installation

### Dependencies

Dependencies needed for compiling and running the node.

- Ubuntu 20.04 + ROS Noetic
- **[superellipsoid_msgs](https://github.com/salihmarangoz/superellipsoid_msgs)**: `git clone https://github.com/salihmarangoz/superellipsoid_msgs`


- **[Ceres Solver](http://ceres-solver.org/installation.html)**: `$ sudo apt install libceres-dev # tested with 1.14.0`

- **[OpenCV](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html)**: `sudo apt install libopencv-dev # tested with 4.2.0`

- Other ROS dependencies:

  ```bash
  $ cd catkin_ws/
  $ rosdep install --from-paths src --ignore-src -r
  ```

### Optional Dependencies

Packages needed for some launch files.

- **[voxblox](https://voxblox.readthedocs.io/en/latest/pages/Installation.html)**

  ```bash
  $ cd ~/catkin_ws/src
  $ mkdir voxblox
  $ cd voxblox
  $ git clone git@github.com:ethz-asl/voxblox.git
  $ wstool init . ./voxblox/voxblox_ssh.rosinstall
  ```

- [**ur_with_cam_gazebo**](https://github.com/Eruvae/ur_with_cam_gazebo)

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

Two executables described below are available for ROS. Single or both can be run depending on the needs. Parameters are mostly shared between two executables.

- **`superellipsoid_detector_node`**: For processing a stream of pointclouds.
- **`superellipsoid_detector_service`**: For processing a single service request (remote procedure call). (No publisher/subscriber is initialized, except TF)

### Parameters

**NOTE: Other parameters can be found in [`cfg/SuperellipsoidDetector.cfg`](cfg/SuperellipsoidDetector.cfg).**

- **`cost_type`** **: 2**

  The cost function for fitting superellipsoids to the input points. `RADIAL_EUCLIDIAN_DISTANCE` and `SOLINA` are recommended options. In order to show the residuals, the formulas below are not simplified.

  ![f](imgs/formulas/f_.png)

  - `CostFunctionType::NAIVE = 0`: Uses the superellipsoid parametric representation directly.

  ![naive](imgs/formulas/naive_.png)

  - `CostFunctionType::LEHNERT = 1`: Lehnert's implementation squares the parametric function value compared to Solina's method.

  ![lenhert](imgs/formulas/lenhert_.png)

  - `CostFunctionType::RADIAL_EUCLIDIAN_DISTANCE = 2`: Works better with non-equilateral superellipsoids and estimates inside the shape better. Tends to estimate equally scaled superellipsoids.

  ![radial](imgs/formulas/radial_.png)

  - `CostFunctionType::SOLINA = 3`: Solina's distance approximation method with volume constraint.  Tends to estimate smallest possible superellipsoids.

  ![solina](imgs/formulas/solina_.png)

  - `CostFunctionType::SOLINA_DISTANCE = 4`: Solina's distance approximation method.

  ![solina_dist](imgs/formulas/solina_dist_.png)

- **`prior_center`: 0.1**

  ![prior_center](imgs/formulas/prior_center_.png)

  - The prior (alpha value in the formula) which enforcing optimized center **t** and center estimated via surface normals **p** to be close to each other. Higher values increases the regularization.

- **`prior_scaling`: 0.1**

  ![prior_scaling](imgs/formulas/prior_scaling_.png)

  - The prior (beta value in the formula) which is enforcing **a**, **b**, **c** values define the scaling of superellipsoid to be close to each other. Higher values increase the regularization. Different than the volume constraint.

**NOTE: Other parameters can be found in [`cfg/SuperellipsoidDetector.cfg`](cfg/SuperellipsoidDetector.cfg).**

### Subscribed Topics

Note: Available only for the node.

**`~pc_in`** ("sensor_msgs/PointCloud2")

- RGBXYZ pointcloud as the input (e.g. voxblox output can be used as the input). Currently RGB information is not used. 
  - TODO: Some modifications may be needed to feed XYZ only pointcloud.


### Published Topics

Note: Available only for the node. Also, computation resources will only be used for subscribed topics.

- **`~superellipsoids`** ("superellipsoid_msgs/SuperellipsoidArray")
  - Optimized superellipsoids output. Headers are the same for all superellipsoids.
- **`~superellipsoids_surface`** ("sensor_msgs/PointCloud2")
  - XYZL pointcloud for the surface of superellipsoids.
- **`~centers_prior`** ("sensor_msgs/PointCloud2")
  - XYZL pointcloud for predicted centers via only using surface normals. Recommended only for debugging.
- **`~centers_optimized`** ("sensor_msgs/PointCloud2")
  - XYZL pointcloud for centers computed after the superellipsoid optimization. Centers for failed optimizations will not be published. Recommended only for debugging.
- **`~pc_preprocessed`** ("sensor_msgs/PointCloud2")
  - XYZLNormal pointcloud after applying clustering and surface normal estimation on to the inputs pointcloud.
- **`~missing_surfaces`** ("sensor_msgs/PointCloud2")
  - XYZL pointcloud representing (estimated) missing data points on an superellipsoid.
- **`~surface_normals_marker`** ("visualization_msgs::MarkerArray")
  - Use only for debugging/visualization. Arrow markers for visualizing surface normals. Surface normals are flipped towards the predicted cluster center.

### Transforms

- `world_frame` -> Pointcloud Frame

### Services

- **`~trigger`** ("std_srvs/Empty")
  - Triggers the node to process next input if `processing_mode` parameter is set to `ON_REQUEST`.

- **`/fit_superellipsoids`** (["capsicum_superellipsoid_detector/FitSuperellipsoids"](srv/FitSuperellipsoids.srv))
  - Only available with the service executable (`superellipsoid_detector_service`).

## C++ API

For minimal use, only the template header [`superellipsoid.h`](include/capsicum_superellipsoid_detector/superellipsoid.h) with Ceres Solver, Boost, and PCL libraries is needed. A simple example without any extra dependencies would be like this:

```c++
// Load input points
pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZ>);

// Fill in_cloud with points...

// Filter Invalid Points
std::shared_ptr<std::vector<int>> indices(new std::vector<int>);
pcl::removeNaNFromPointCloud(*in_cloud, *in_cloud, *indices);

// Create a superellipsoid object
superellipsoid::Superellipsoid<pcl::PointXYZ> se(in_cloud);

// Optimization
se.estimateNormals(0.015); // Normals are accesible via getNormals()
se.estimateClusterCenter(2.5);
if (!se.fit(true, 100, RADIAL_EUCLIDIAN_DISTANCE))
{
  printf("Optimization diverged/failed!");
}

// Get optimized parameters
std::map<std::string, double> optimized_parameters = se.getParameters();

// Sample superellipsoid surface
pcl::PointCloud<pcl::PointXYZ>::Ptr surface_cloud = se.sampleSurface();

// Sample superellipsoid volume
pcl::PointCloud<pcl::PointXYZ>::Ptr volume_cloud = se.sampleVolume(0.01);

// Find missing surfaces
pcl::PointCloud<pcl::PointXYZ>::Ptr missing_surface_cloud = se.estimateMissingSurfaces(0.015, 500);
```

ROS message conversions are defined in [`conversions.h`](include/capsicum_superellipsoid_detector/conversions.h). 

For extra features like clustering, object tracking, etc. check the ROS node/service code as a reference.



## HiWi

This project is completed under a HiWi job at [Uni-Bonn Humanoid Robotics Lab](https://www.hrl.uni-bonn.de/). Meeting notes can be found [here](hiwi).



## To-Do

- [ ] Better clustering / instance segmentation.
- [ ] Use of surface normals instead of a single estimated center in the optimization process. This may work better for non-sphere like capsicums.
- [ ] Sometimes capsicums may have weird shapes (not like a sphere nor superellipsoid, not symmetrical, etc.). Combination of multiple superellipsoids for modeling the fruit surface would be better. On the other hand, estimating the missing parts of the shape becomes difficult this way.
- [ ] Loss functions can be used against outliers: http://ceres-solver.org/nnls_modeling.html#lossfunction
- [ ] skip clustering if input pointcloud has labels
- [ ] simplify readme
- [ ] fix ros service, maybe add ros messages
- [ ] update the comments in superellipsoid msgs
- [ ] use tf2 instead of tf
- [ ] input pointcloud must have homogeneous density for as much as possible. maybe use  voxelgrid filter limiting density in some regions.
