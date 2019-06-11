# Sparse-Mapper

Sparse-Mapper stack for ROS.
Master Thesis in Biorobotics Laboratory UNAM.

## Description

Sparse-mapper is a novel architecture for robotic mapping with known poses
on 3D environments.
This approach uses unsupervised learning over unstructured point clouds to
automatically generate static topological maps suitable for robot navigation.
Unlike laser based occupancy grid-maps Sparse-Map maintains a sparse 3D representation
of the environment allowing to model much more complex spaces yet it has a smaller
memory footprint compared to other approaches such as [Octomap][1].
It runs on the popular Meta-Operative system for robots [ROS][2] and is compatible
with the point-cloud processing library [PCL][3], and can
run on GPU to accelerate its processing speed.

## Installation

This program depends on PCL 1.7.2 or higher, Eigen 3 or higher, CUDA runtime, boost and ROS.
This has been tested with ROS Kinetic but can work on Melodic and PCL 1.8.
You will need to install all of them for the compilation to work, there are functions designed
to work exclusively on CPU and non nvcc compilation is supported but not extensively tested.
You can
follow the ROS installation [tutorial](http://wiki.ros.org/ROS/Installation) which should
install a working version of PCL too. It is not recommended to install PCL separately
as the ROS installation is different and you might end with duplicate installations.

Once that is done simply compiling the package with catkin_make should suffice.

```bash
catkin_make
source devel/setup/bash
```

Also do not forget to specify you correct architecture in case of CUDA compilation.
By default it is set to compute_61 for GTX 1050, check the NVIDIA website to check for
your architecture and change:

```
CUDA_NVCC_FLAGS
${CUDA_NVCC_FLAGS};
-O3 -lineinfo -gencode arch=compute_61,code=sm_61 -std=c++11
)
```
To your device architecture on the CMakeList.txt for the space_quantization package.

## Known issues
For some reason despite being specified on the CMake files sparse_map_msgs is sometimes
compiled after the other packages. This causes the compilation to fail, if this happens to you:

```bash
catkin_make --pkg sparse_map_msgs
catkin_make
```



This will force to compile the msgs package before everything else.

## Usage

This pipeline is composed of several nodes each one of the accomplishes a different task.
The easiest way to use the sparse_mapper.launch file under the sparse_map_launch package.
You only need to remap a few parameters and topics to make it work with your robot:

```xml
  <remap from="/localization_pose" to="/your_robot/localization_pose"/>
  <remap from="/cloud_in" to="/your_robot/pointcloud_message"/>
  <param name="baseFrame" value="map/base/odom" type="string"/>
```

And then run using roslaunch:

```bash
roslaunch sparse_map_launch sparse_mapper.launch
```

Then move your robot around until you ensemble your map. You can then call the /make_graph
service to save the map to a plain text file. Or you can publish a previously ensembled
pointcloud reconstruction, from a 3D SLAM framework as [RTAB-Map][4] for example, to the same /cloud_in topic this will generate the sparse map from that cloud only.

Once the map is saved on disk you can run the map server to request paths for the robot using the map.
Again it is faster and easier to use the launch file provided.

```bash
roslaunch sparse_map_launch map_server.launch
```

And call the make_plan service to request a path. It is recommended that you also run a
AMCL localization as it both helps with visualization and if you need to actually follow
the paths you need some source of localization. For convenience a localization launch based
on AMCL is provided.

```bash
roslaunch sparse_map_launch robot_locator.launch
```

Finally some launches require Octomap to work. You can either install it via
 apt or use the supplied submodule to download and compile with everything else

You can find the description of the relevant parameters and topic on the roslaunch files.

Also in testGraphs you can find some previously ensembled occupancy grids, sparse maps
and graphs acquired by me in the Biorobotics Laboratory @ UNAM in case you can to try
 this package out of the box.

## HSR compatibility

This approach was tested using a Toyota HSR robot. Launch file named
hsrb_something.launch are HSR ready and should run out of the box.
Just check that file locations point to somewhere on your computer.

## Nodes

### Space_Quantization

This node is in charge of reading and processing every point cloud. It is comprised of 3 nodes
The pose checker, the preprocessor and the quantization:

#### pose_chck

Sparse_mapper builds a global map based on several different scenes. To prevent adding lots
redundant information when the robot is not moving we only push the point cloud into the pipeline
if the robot has move or rotated a certain amount. This node keeps track of the traveled distance and
pushes the point cloud when specified.

##### Subscribed Topics

-   localization_pose (geometry_msgs/PoseWithCovarianceStamped):  Output of localization system such as AMCL.
-   cloud_in (sensor_msgs/PointCloud2):  Data source message i.e Kinect.

##### Published Topics

-   cloud_out(sensor_msgs/PointCloud2):  Output point cloud only publishes once the robot has moved a certain amount.

##### Parameters

-   ~angular_update (float): Minimum angular motion required to push point cloud into pipeline
-   ~linear_update (float): Minimum linear motion required to push point cloud into pipeline

### Preprocessor

The raw point cloud should be preprocessed before quantization, this implies removing point
too far and too close, from the sensor as those have a lot of noise, then apply a voxelization
filter to downsample the cloud and finally transform all point to a more convenient reference frame,
usually the robot base or if available the map.

##### Subscribed topics

-   cloud (sensor_msgs/PointCloud2): The cloud to be processed .

##### Published topics

-   processed_cloud (sensor_msgs/PointCloud2): The processed point cloud ready for quantization.

##### Parameters

-   ~crop\_[max/min]\_distance <float>: Any point outside this range will be removed from the cloud.
-   ~voxelSize <float>: Voxel size for downsampling filter set to 0 if you do not want to downsample.

### Quantizator: segmentation_node

This takes the input point cloud and uses clustering algorithms to compress it into a codebook. It supports CUDA
acceleration and 3 different algorithms: Kmeans++, Kmeans and LBG. Not that not all of them are able to run both on CPU
and GPU.

#### Subscribed topics

-   cloud (sensor_msgs/PointCloud2): The cloud to be quantized.

#### Published topics

-   labeled_cloud (sensor_msgs/PointCloud2): Point Cloud where Evey point has a level corresponding to a code.
-   codebook(sparse_map_msgs/codebook): The generated codebook as array of Point Messages

#### Services provided

It is worth nothing that both these services were created for testing and are not
recommended to use while mapping. But are nevertheless provided.

-   segmentation_reconfigure (sparse_map_msgs/Reconfigure): Allows to change number of centroids and Lloyd iterations on runtime.
-   quantize_space(sparse_map_msgs/QuantizeCloud): Quantize a cloud via service.

#### Parameters

-   ~nClusters <int> : Number of clusters to compute. LBG only supports powers of 2.
-   ~iterations <int>: Number of times to run Lloyd algorithm on clustering.
-   ~method <string> : [inner/kpp/kmeansCPU/LBG] method to use.
-   ~publish_label_space <bool>; Whether to publish the labeled cloud, if set to false the program should run
    faster but it is helpful to see the cloud for debugging purposes.

### Mapper: sparse_mapper_node

Finally this node accumulates all codebooks to generate a global map using several scenes. It is a fairly simple
node that also separates space into free and occupied.

#### Subscribed topics

-   codebook (sparse_map_msgs/codebook): A codebook to add to the global map.
-   clear_graph (std_msgs/Empty): Publish here if you want to clear all codes for current map.

#### Published topics

-   centroids_marker(visualization_msgs/MarkerArray): Markers used to visualize current map.

#### Services provided

-   make_graph (sparse_map_msgs/SaveMap): Saves current map to disk as plain text

#### Parameters

-   ~free_thr <float> : All codes higher than this will be declared occupied, it represents largest
    valid height to be considered a free node.

### Sparse-Map server: map_server

This nodes is responsible for reading maps from disk, visualization on RVIZ
and serving path requests. It is worth noting this takes a few seconds to
load as it constructs the graph after reading the node from disk.

#### Subscribed topics

-   None

#### Published topics

-   centroids_marker(visualization_msgs/Marker): Markers used to visualize current nodes.
-   graph_marker(visualization_msgs/Marker): Markers used to visualize current map.
-   label_marker(visualization_msgs/Marker): Markers used to visualize node id.
-   terminal_marker(visualization_msgs/Marker): Marker used to represents robot initial and final state as cylinders.

#### Services provided

-   make_plan (sparse_map_msgs/MakePlan): Given start and end locations, known
    as terminals, compute a plan. Currently only supports points as terminals.
     Not poses.
-   save_graph (sparse_map_msgs/SaveMap): Save the processed sparse_map and edge list
    as a pcd file and a plaint text txt edges files. Simply provide the path to the
    edges file, no extension, and a pcd name will be generated with the same name.

#### Parameters

-   ~map_file <string> : Name of topological map file on disk can be legacy txt file or a
    pcd file .
-   ~map_frame <string> : Reference frame of map, default is "map".
-   ~safety_height <float> : Height of robot, all nodes over this height are ignored
-   ~safety_radius <float> : Radius of robot. Robot radius for 3D collision detection
-   ~max_dist <float> : Maximum permissible distance between neighbor nodes. If larger they won't connect
-   ~min_dist <float> : Minimum distance between free nodes, if smaller nodes are considered redundant and reduced to centroid.
-   ~max_dist_terminal <float> : Maximum distance between terminal and closest free node if larger planner will report a failure.
-   ~k_neighbors <int> : How many neighbors each node has.
-   ~visualize_terminals <bool> : Whether or not paint terminals on RVIz.
-   ~validate_terminals <bool> : Whether to do collision detection on start and final
    points when requesting a plan. If set to false won't do any collision detection
    and a crash on start or stop position might occur!!
-   ~use_existing_graph (bool) : Set to true if you want to provide
    existing .txt and pcd files for the graph and false if you want to ensemble the
     topological map from scratch
-   ~nodes_file (string) : path to pcd file containtning the sparse_map centroids.
-   ~edges_file (string) : path to plaintext file containtning graph edges.

[1]: https://octomap.github.io/

[2]: http://www.ros.org/

[3]: http://pointclouds.org/

[4]: http://introlab.github.io/rtabmap/
