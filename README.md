# Sparse-Mapper

Sparse-Mapper stack for ROS.
Master Thesis in Biorobotics Laboratory UNAM.

## Description

Sparse-mapper is a novel architecture for robotic mapping with known poses
on 3D enviroments.
This approach uses unsupervised learning over unstructured pointclouds to
automatically generate static topological maps suitable for robot Navigation.
Unlike laser based occupancy grid-maps Sparse-Map maintains a sparse 3D representation
of the enviroment allowing to model much more complex spaces yet it has a smaller
memory footprint compared to other apporaches such as [OctoMap][1].
It runs on the popular Meta-Operative system for robots [ROS][2] and is compatible
with the point-cloud processing library [PCL][3], and can
run on GPU to accelerate its processing speed.

## Installation

This program depends on PCL 1.7.2 or higher, Eigen 3 or higher, CUDA runtime boost and ROS.
It has been tested with ROS Kinetic but can work on Melodic
You will need to install all of them for the compilation to work, there are funtions designed
to work exclusivelly on CPU non-nvcc compilation is still not supported. You can
follow the ROS installation [tutorial](http://wiki.ros.org/ROS/Installation) wich should
install a working version of PCL too. It is not recommended to install PCL separately
as the ROS installation is different from the source and the dpkg install folder
resulting on duplicate installations.

Once that is done simply compiling the package with catkin_make should suffice.

```bash
catkin_make
source devel/setup/bash
```

## Usage

This pipeline is composed of several nodes each one of the acoomplishes a different task.
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
service to save the map to a plain text file.

Once the map is saved on disk you can run the map server to request paths for the robot using the map.
Again it is faster and easier to use the launch file provided.

```bash
roslaunch sparse_map_launch map_server.launch
```

And call the make_plan service to request a path. It is recommended that you also run a
AMCL localization as it both helps with visualization and if you need to actually follow
the paths you need a source of localization. For convenience a localization launch based
on AMCL is provided.

```bash
roslaunch sparse_map_launch robot_locator.launch
```

Finally some launches require Octomap to work. You can either install it via
 apt or use the supplied submodule to donwload and compile with everything else

You can find the description of the relevant parameters and topic on the roslaunch files.

## HSR compatibility

This apporach was tested using a Toyota HSR robot. Launch file named
hsrb_something.launch are HSR ready and should run out of the box.
Just check that file locations point to somewhere on your computer.

## Nodes

### Space_Quantization

This node is in charge of reading and processing every point cloud. It is comprised of 3 nodes
The pose checker, the preprocessor and the quantization:

#### pose_chck

Sparse_mapper build a global map based on several different scenes. To prevent adding lots
redundant information when the robot is not moving we only push the point cloud into the pipeline
if the robot has move or rotated a certain amount. This node keeps track of the traveled distance and
pushes the pointcloud when specified.

##### Subscribed Topics

-   localization_pose (geometry_msgs/PoseWithCovarianceStamped):  Output of localization system such as AMCL.
-   cloud_in (sensor_msgs/PointCloud2):  Data source message i.e Kinect.

##### Published Topics

-   cloud_out(sensor_msgs/PointCloud2):  Output point cloud only publishes once the robot has moved a certain amount.

##### Parameters

-   ~angular_update (float): Minimum angular motion required to push pointcloud into pipeline
-   ~linear_update (float): Minimum linear motion required to push pointcloud into pipeline

### Preprocessor

The raw point cloud should be preprocessed before quantization, this implies removing point
too far and too close, from the sensor as those have a lot of noise then apply a voxelization
filter to downsample the cloud and finally trasnform all point to a more convenient reference frame,
usually the robot base or if avalible the map.

##### Subscribed topics

-   cloud (sensor_msgs/PointCloud2): The cloud to be propcessed .

##### Published topics

-   processed_cloud (sensor_msgs/PointCloud2): The processed point cloud ready for quantization.

##### Parameters

-   ~crop\_[max/min]\_distance <float>: Any point outside this reange will be removed from the cloud.
-   ~voxelSize <float>: Voxel size for downsampling filter set to 0 if you do not want to downsample.

### Quantizator: segmentation_node

This takes the input point cloud and uses clustering algorithms to compress it into a codebook. It supports cuda
acceleration and 3 different algorithms: Kmeans++, Kmeans and LBG. Not that not all of them are able to run both on CPU
and GPU.

#### Subscribed topics

-   cloud (sensor_msgs/PointCloud2): The cloud to be quantized.

#### Published topics

-   labeled_cloud (sensor_msgs/PointCloud2): Point Cloud where evey point has a level corresponding to a code.
-   codebook(sparse_map_msgs/codebook): The generated codebook as array of Point Messages

#### Services provided

It is worth nothing that both these services were created for testing and are not
recommended to use while mapping. But are nevertheless provided.

-   segmentation_reconfigure (sparse_map_msgs/Reconfigure): Allows to change number of centroids and lloyd iterations on runtime.
-   quantize_space(sparse_map_msgs/QuantizeCloud): Quantize a cloud via service.

#### Parameters

-   ~nClusters <int> : Number of clusters to compute. LBG only supports powers of 2.
-   ~iterations <int>: Number of times to run Lloyd algorithm on clustering.
-   ~method <string> : [inner/kpp/kmeansCPU/LBG] method to use.
-   ~publish_label_space <bool>; Wether to publish the labeled cloud, if disable with false the program should run
    faster but it is helpfull to see the cloud for debuggin porpuses.

### Mapper: sparse_mapper_node

Finally this node acumulates all codebooks to generate a global map using several scenes. It is a fairly simple
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

This nodes is responsible for reading maps from disk, visualziation on RVIZ
and serving path requests. It is worth noting this takes a few seconds to
load as it constructs the graph after reading the node from disk.

#### Subscribed topics

-   None

#### Published topics

-   centroids_marker(visualization_msgs/Marker): Markers used to visualize current nodes.
-   graph_marker(visualization_msgs/Marker): Markers used to visualize current map.
-   label_marker(visualization_msgs/Marker): Markers used to visualize node id.
-   terminal_marker(visualization_msgs/Marker): Marker used to represents robot initial and final state.
-

#### Services provided

-   make_plan (sparse_map_msgs/MakePlan): Given start and end locations, known
    as terminals, compute a plan. Currently only supports points as terminals.
     Not poses.

#### Parameters

-   ~map_file <string> : Name of topological map file on disk.
-   ~map_frame <string> : Reference frame of map.
-   ~safety_height <float> : Height of robot, all nodes over this height are ignored
-   ~safety_radius <float> : Radius of robot. Robot radius for 3D collision detection
-   ~max_dist <float> : Maximum permisible distance between neighboor nodes. If larger they won't connect
-   ~min_dist <float> : Minimium distance between free nodes, if smaller nodes are considered redundant and reduced to centroid.
-   ~max_dist_terminal <float> : Maximum distance between terminal and closest free node if larger planner will report a failure.
-   ~k_neighboors <int> : How many neighboors each node has.
-   ~visualize_terminals <bool> : Wether or not paint terminals on RVIz.
-   ~validate_terminals <bool> : Wether to do collision detection on start and final points when requesting a plan. If set to false won't do any collision detection and a crash on start or stop might occurr!!

[1]: https://octomap.github.io/

[2]: http://www.ros.org/

[3]: http://pointclouds.org/
