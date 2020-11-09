# ar_tools

Build: colcon build --packages-select ar_tools

## aruco\_zed
ros2 run ar\_tools aruco\_zed WORLD\_FRAME\_ID CAMERA\_FRAME\_ID

This node publishes TFs for Aruco markers from the left eye of a streaming ZED camera. Currently only USB is supported, additional arguments for listening to a network stream coming soon. The node also publishes a Path msg, with each pose corresponding to that of a detected single marker transformed into WORLD\_FRAME\_ID. Assumes tf lookup is available for WORLD\_FRAME\_ID to CAMERA\_FRAME\_ID.

- WORLD\_FRAME\_ID: world or parent frame
- CAMERA\_FRAME\_ID: camera frame

## aruco\_grpc
ros2 run ar\_tools aruco\_grpc WORLD\_FRAME\_ID CAMERA\_FRAME\_ID GRPC\_IMAGE\_SERVER

Please refer to documentation for the aruco\_zed node.

- GRPC\_IMAGE\_SERVER: address for a GRPC image server, designed to follow syntax of [azure_grpc](https://github.com/Halodi/azure_grpc)
