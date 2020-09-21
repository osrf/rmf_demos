This document is a declaration of software quality for the `rmf_gazebo_plugins` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# `rmf_gazebo_plugins` Quality Declaration

The package `rmf_gazebo_plugins` claims to be in the **Quality Level 4** category.

Below are the detailed rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Requirements for Quality Level 4 in REP-2004](https://www.ros.org/reps/rep-2004.html).

## Version Policy [1]

### Version Scheme [1.i]

`rmf_gazebo_plugins` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#versioning).

### Version Stability [1.ii]

`rmf_gazebo_plugins` is at a stable version, i.e. `>= 1.0.0`.
The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]

`rmf_gazebo_plugins` does not have a public API.

### API Stability Policy [1.iv]

`rmf_gazebo_plugins` does not have a public API.

### ABI Stability Policy [1.v]

`rmf_gazebo_plugins` does not have a public API.

### API and ABI Stability Within a Released ROS Distribution [1.vi]

`rmf_gazebo_plugins` does not have a public API.

## Change Control Process [2]

`rmf_gazebo_plugins` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#package-requirements).

### Change Requests [2.i]

`rmf_gazebo_plugins` requires that all changes occur through a pull request.

### Contributor Origin [2.ii]

`rmf_gazebo_plugins` does not require a confirmation of contributor origin.

### Peer Review Policy [2.iii]

All pull requests must have at least 1 peer review.

### Continuous Integration [2.iv]

All pull requests must pass CI on all platforms supported by RMF.
The CI checks only that the package builds.
The most recent CI results can be seen on [the workflow page](https://github.com/osrf/rmf_demos/actions).

### Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation [3]

### Feature Documentation [3.i]

`rmf_gazebo_plugins` is documented in the [parent repository's README.md file](https://github.com/osrf/rmf_demos/blob/master/README.md).

### Public API Documentation [3.ii]

`rmf_gazebo_plugins` does not have a public API.

### License [3.iii]

The license for `rmf_gazebo_plugins` is Apache 2.0, the type is declared in the [package.xml](package.xml) manifest file, and a full copy of the license is in the repository level [LICENSE](../LICENSE) file.

### Copyright Statement [3.iv]

The copyright holders each provide a statement of copyright in each source code file in `rmf_demo_tasks`.

### Quality declaration document [3.v]

This quality declaration is linked in the [README file](README.md).

This quality declaration has not been externally peer-reviewed and is not registered on any Level 4 lists.

## Testing [4]

### Feature Testing [4.i]

`rmf_gazebo_plugins` does not have any tests.

### Public API Testing [4.ii]

`rmf_gazebo_plugins` does not have a public API.

### Coverage [4.iii]

`rmf_gazebo_plugins` does not track coverage statistics.

### Performance [4.iv]

`rmf_gazebo_plugins` does not have performance tests.

### Linters and Static Analysis [4.v]

`rmf_gazebo_plugins` does not use the standard linters and static analysis tools for its CMake code to ensure it follows the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#linters).

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]

`rmf_gazebo_plugins` has the following direct runtime ROS dependencies.

#### rclcpp

`rclcpp` is [**Quality Level 3**](https://github.com/ros2/rclcpp/blob/master/rclcpp/QUALITY_DECLARATION.md).

#### gazebo_ros_pkgs

`gazebo_ros_pkgs` does not declare a Quality Level.
It is assumed tobe at **Quality Level 4** based on its widespread use.

#### std_msgs

`std_msgs` is [**Quality Level 3**](https://github.com/ros2/common_interfaces/blob/master/std_msgs/QUALITY_DECLARATION.md).

#### rmf\_fleet\_msgs

`rmf_fleet_msgs` is [**Quality Level 3**](https://github.com/osrf/rmf_core/blob/master/rmf_fleet_msgs/QUALITY_DECLARATION.md).

#### rmf\_dispenser\_msgs

`rmf_dispenser_msgs` is [**Quality Level 3**](https://github.com/osrf/rmf_core/blob/master/rmf_dispenser_msgs/QUALITY_DECLARATION.md).

#### rmf\_ingestor\_msgs

`rmf_ingestor_msgs` is [**Quality Level 3**](https://github.com/osrf/rmf_core/blob/master/rmf_ingestor_msgs/QUALITY_DECLARATION.md).

#### building\_map\_msgs

`building_map_msgs` is [**Quality Level 3**](https://github.com/osrf/traffic_editor/blob/master/building_map_msgs/QUALITY_DECLARATION.md).

### Optional Direct Runtime ROS Dependencies [5.ii]

`rmf_gazebo_plugins` does not have any optional direct runtime ROS dependencies.

### Direct Runtime non-ROS Dependency [5.iii]

`rmf_gazebo_plugins` does not have any runtime non-ROS dependencies.

## Platform Support [6]

`rmf_gazebo_plugins` does not support all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers).
`rmf_gazebo_plugins` supports ROS Eloquent and ROS Foxy.

## Security [7]

### Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).
