# capsicum_superellipsoid_detector

[toc]



## Compile & Run

```bash
# todo: ceres-solver dependency

$ cd catkin_ws/
$ rosdep install --from-paths src --ignore-src -r
$ catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release

$ source catkin_ws/devel/setup.bash
$ roslaunch capsicum_superellipsoid_detector start.launch
```







## Meetings

### 15-July-2021

- Surface normal extraction and predicting mass center using Least Squares Intersectıon of Lines is working, results look promising.
- Code is implemented in C++ mainly, prototype can be found in `notebooks/intersection_of_lines.ipynb`

ToDo:

- Seperated ROI pointclouds will be mapped with a higher precision Octomap and the algorithm will be tested on it.
- Evaluation is needed. Firstly try with default ground truths, after getting better results increase the resolution. If the orientation of the pepper is needed figure out a way.
- Implement superellipsoid optimizer using a non-linear least-squares library (e.g. ceres-solver, Eigen)

See:

- https://silo.tips/download/least-squares-intersection-of-lines
- http://ceres-solver.org/

Screenshots:

![](imgs/2_mass_center_predictor/1.png)

![](imgs/2_mass_center_predictor/2.png)



### 8-July-2021

(may not be complete because this documentation is started on 15 July)

- Superellipsoid optimization is implemented in Python. Prototype can be found in ` notebooks/detector.ipynb`

ToDo:

- Optimization gets stuck in local solutions (can be seen in the screenshot, the pepper on the upper right). Needs a prediction of the center for peppers.
- Use raw pointcloud from sensor.

Screenshots:

![](imgs/1_python_superellipsoid.png)
