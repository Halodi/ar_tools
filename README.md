# ar_tools

Uses:
- [halodi_msgs](https://github.com/Halodi/halodi-messages)
- OpenCV 4.4.0 + contrib
- numpy and scipy

Build: colcon build --packages-select ar_tools

This package publishes TFs for Aruco markers from cameras, currently supporting Zed (left eye) and image streams over gRPC. Nodes also publish transformed poses in halodi_msgs/msg/ARMarkers form.

Running aruco\_ nodes requires a config file; refer to config/aruco.json for an example.

#### aruco.json
- parent\_frame: frame to transform markers into
- camera\_frame: frame that markers are detected in
- broadcast\_transforms: if true, publishes camera -> marker TF msgs
- image\_scaling: fx, fy of [cv.resize](https://docs.opencv.org/master/da/d54/group__imgproc__transform.html#ga47a974309e9102f5f08231edc7e7529d)
- aruco\_dict: identifying string that follows OpenCV's pre-defined [dictionaries](https://docs.opencv.org/master/dc/df7/dictionary_8hpp.html), omitting DICT\_
- marker\_sizes: dictionary of numerical marker IDs to side lengths, used for pose estimation. Markers not defined here are discarded
- aruco\_params: public fields for a parameters [object](https://docs.opencv.org/master/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html#aca7a04c0d23b3e1c575e11af697d506c)

## AR marker streaming nodes

### stf\_server
ros2 run ar\_tools stf\_server FREQUENCY

Convenience node to fetch stamped transforms for timestamps which are not associated with a clock topic. Used by all aruco\_ nodes via halodi_msgs/srv/GetStampedTF. Due to use of Python's time.perf\_counter() for internal time-keeping, calls to this service should only be made from the machine running the service server.
For messages on /tf_static, it is assumed that non-zero timestamps represent an additional sensor delay to the timestamp in transform requests.

- FREQUENCY: rate throttling for subscribers, to help reduce CPU load and network traffic.

### aruco\_zed
ros2 run ar\_tools aruco\_zed PATH\_TO\_ARUCO\_CONFIG\_FILE STREAMING\_ADDRESS

- STREAMING\_ADDRESS: address of a stream to listen to, e.g. 10.0.0.1:30000. If no colon is detected in this argument, a non-network connection is assumed.

### aruco\_grpc
ros2 run ar\_tools aruco\_grpc PATH\_TO\_ARUCO\_CONFIG\_FILE GRPC\_IMAGE\_SERVER GRPC\_COMMON\_SERVER 

- GRPC\_IMAGE\_SERVER: address of a GRPC image server, e.g. 10.0.0.1:30000. Data structure follows single-image syntax of [azure_grpc](https://github.com/Halodi/azure_grpc)
- GRPC\_COMMON\_SERVER: address of a GRPC common server

## Extrinsic calibration nodes
Running calibration\_extrinsic\_ nodes requires a config file; refer to config/extrinsic\_calibration.json for an example. Requires a marker fixed in space e.g. on a wall, visible to the robot across a reasonable range of motion. Robot is assumed to be moving while data is being collected, e.g. through a calibration\_motion\_ node.

#### extrinsic\_calibration.json
- common: contains parameters used by all extrinsic calibration nodes
    - tf\_listener\_warmup\_duration: time for subscribing to TFs before also subscribing to marker msgs
    - data\_aggregation\_duration: time for subscribing to both TFs and marker msgs
    - data\_aggregation\_frequency: subscriber throttling rate
    - data\_aggregation\_samples\_n: reduce total marker samples to >= this value after collection
    - markers\_topic: topic for markers
    - static\_target\_ID: frame\_id for a marker that is stationary relative to the robot
    - static\_frame: a fixed TF frame relative to the robot, nominally the root of the TF tree
    - camera\_frame\_parent: the immediate parent frame of the camera
    - camera\_delay\_min/max: bounds of sensor delay for clipping purposes. Currently, only >=0 values are supported
    - outbound\_calibration\_topic: for publishing calibration info if optimization is successful
    - camera\_name: camera name for publishing
- head: parameters for calibrating a camera mounted in/on the head
    - camera\_delay: initial estimate of camera delay
    - pitch\_offset: initial estimate of head pitch offset
    - head\_to\_camera\_xyz\_ypr: initial estimate of a 6-tuple representing the static transform between the head and the camera

### calibration\_extrinsic\_head
ros2 run ar\_tools calibration\_extrinsic\_head PATH\_TO\_CALIB\_CONFIG\_FILE

### calibration\_motion\_head
ros2 run ar\_tools calibration\_motion\_head

Runs a looping routine that pivots the neck and pelvis.

## Launch files
Please fix argument fields, then cd to the workspace root and run "source install/setup.bash" prior to use.

- ros2 launch ar\_tools grpc.launch.py: aruco\_grpc + stf\_server
- ros2 launch ar\_tools zed.launch.py: aruco\_zed + stf\_server
