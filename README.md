# ar_tools

Uses:
- [halodi_msgs](https://github.com/Halodi/halodi-messages)
- OpenCV 4.4.0 + contrib

Build: colcon build --packages-select ar_tools

This package publishes TFs for Aruco markers from cameras, currently supporting Zed (left eye) and image streams over gRPC. Nodes also publish transformed poses in halodi_msgs/msg/ARMarkers form.

Running these nodes uses a config file; refer to config.json for an example.

### config.json
- parent\_frame: frame to transform markers into
- camera\_frame: frame that markers are detected in
- spin\_period: sleep duration between consecutive calls to rclpy.spin_once(...). Too low a period may affect image retrieval
- image\_scaling: fx, fy of [cv.resize](https://docs.opencv.org/master/da/d54/group__imgproc__transform.html#ga47a974309e9102f5f08231edc7e7529d)
- aruco\_dict: identifying string that follows OpenCV's pre-defined [dictionaries](https://docs.opencv.org/master/dc/df7/dictionary_8hpp.html), omitting DICT\_
- marker\_sizes: dictionary of numerical marker IDs to side lengths, used for pose estimation. Markers not defined here are discarded
- aruco\_params: public fields for a parameters [object](https://docs.opencv.org/master/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html#aca7a04c0d23b3e1c575e11af697d506c)

## aruco\_zed
ros2 run ar\_tools aruco\_zed PATH\_TO\_CONFIG\_FILE STREAMING\_ADDRESS

- STREAMING\_ADDRESS (optional): address of a stream to listen to, e.g. 10.0.0.1:30000

## aruco\_grpc
ros2 run ar\_tools aruco\_grpc PATH\_TO\_CONFIG\_FILE GRPC\_IMAGE\_SERVER GRPC\_COMMON\_SERVER 

- GRPC\_IMAGE\_SERVER: address of a GRPC image server, e.g. 10.0.0.1:30000. Data structure follows single-image syntax of [azure_grpc](https://github.com/Halodi/azure_grpc)
- GRPC\_COMMON\_SERVER: address of a GRPC common server
