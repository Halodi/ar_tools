# ar_tools

Build: colcon build --packages-select ar_tools

## aruco\_zed
ros2 run ar\_tools aruco\_zed WORLD\_FRAME\_ID CAMERA\_FRAME\_ID STREAMING\_ADDRESS

This node publishes TFs for Aruco markers from the left eye of a ZED camera.
The node also publishes a Path msg on the /aruco/CAMERA\_FRAME\_ID topic, with each pose corresponding to that of a detected single marker transformed into WORLD\_FRAME\_ID. Assumes tf lookup is available for WORLD\_FRAME\_ID to CAMERA\_FRAME\_ID.

- WORLD\_FRAME\_ID: world or parent frame
- CAMERA\_FRAME\_ID: camera frame
- STREAMING\_ADDRESS (optional): address of a stream to listen to, e.g. 10.0.0.1:30000

## aruco\_grpc
ros2 run ar\_tools aruco\_grpc WORLD\_FRAME\_ID CAMERA\_FRAME\_ID GRPC\_IMAGE\_SERVER

Please refer to documentation for the aruco\_zed node.

- GRPC\_IMAGE\_SERVER: address of a GRPC image server, e.g. 10.0.0.1:30000. Data structure follows single-image syntax of [azure_grpc](https://github.com/Halodi/azure_grpc)
