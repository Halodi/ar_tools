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
Running calibration\_extrinsic\_ nodes requires a config file; refer to config/extrinsic\_calibration.json for an example. Requires a marker fixed in space e.g. on a wall, visible to the robot across a reasonable range of motion with a streaming node already running, e.g. aruco\_zed. Robot is assumed to be moving while data is being collected, e.g. through a calibration\_motion\_ node.
Note that the first element of the parameter vector to be optimized is assumed to be a camera delay (>=0).

#### extrinsic\_calibration.json
- common: contains parameters used by all extrinsic calibration nodes
    - data\_collection\_duration: time for subscribing to both TFs and marker msgs
    - data\_collection\_frequency: subscriber throttling rate
    - data\_collection\_samples\_n: reduce total marker samples to >= this value after collection
    - markers\_topic: topic for markers
    - stationary\_target\_frame: frame ID for a marker that is stationary relative to the robot
    - static\_frame: a fixed TF frame ID relative to the robot, nominally the root of the TF tree
    - camera\_frame\_parent: the immediate parent frame ID of the camera
    - camera\_frame: frame ID of the camera
    - outbound\_calibration\_topic: for publishing calibration info if optimization is successful
    - camera\_name: camera name for publishing
    - de: some args for [differential evolution](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.differential_evolution.html)
- head: parameters for calibrating a camera mounted in/on the head
    - mean: [ camera\_delay, head\_pitch\_offset, head\_to\_camera\_xyz\_ypr ]
    - extents: range of mean. DE bounds are calculated by mean +/- extents
    - camera\_delay: initial estimate of camera delay

### calibration\_extrinsic\_head
ros2 run ar\_tools calibration\_extrinsic\_head PATH\_TO\_CALIB\_CONFIG\_FILE

For calibrating a camera mounted in or on the head (only movement relative to the robot is through the neck joint).

### calibration\_motion\_head
ros2 run ar\_tools calibration\_motion\_head

Runs a looping routine that moves the neck and pelvis.

## Launch files
Please adjust arguments in relevant launch/config files, then cd to the workspace root and run "source install/setup.bash" prior to use.

- ros2 launch ar\_tools grpc.launch.py: aruco\_grpc + stf\_server
- ros2 launch ar\_tools zed.launch.py: aruco\_zed + stf\_server
- ros2 launch ar\_tools extrinsic\_calibration\_head.launch.py: calibration\_motion\_head + calibration\_extrinsic\_head (required)
