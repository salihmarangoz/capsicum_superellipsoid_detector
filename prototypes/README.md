# Prototypes / Experiments

Some prototypes and experiments (implemented in Python) can be found below. Selected ideas are already implemented in C++:

| Related File                                                 | Description                                                  |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| [detector_prototype.py](detector_prototype.py)               | Simple prototype with ROS1 support. Following ideas are not implemented here. |
| [simulate_depth_noise.py](simulate_depth_noise.py)           | A simple code for adding noises (gaussian noise, shadowing effect, salt and pepper, waves) to the pointcloud data obtained from Gazebo simulator. |
| [optimization.ipynb](optimization.ipynb)                     | Least-Squares optimization for fitting superellipsoid to partial pointcloud. |
| [intersection_of_lines.ipynb](intersection_of_lines.ipynb)   | Least-Squares estimation of capsicum centroid using surface normals. |
| [cost_functions.ipynb](cost_functions.ipynb)                 | Analyzing different cost functions.                          |
| [find_missing_part_of_spherical_data.ipynb](find_missing_part_of_spherical_data.ipynb) | Experiments for finding missing parts of spherical data. Can be extended to superellipsoidical data. (**Note:** Embedded videos may not be properly visualized on Github. |
| [superellipsoid_fibonacci_projection_sampling.ipynb](superellipsoid_fibonacci_projection_sampling.ipynb) | Uniform-like sampling of superellipsoid surface points. See [my blog post](https://salihmarangoz.github.io/blog/Superellipsoid_Sampling/) for in-detail analyze. |
