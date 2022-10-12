# SME-week-6

## Writing a simple service and client (Python)
### 1. Creating a Package

Recall that packages should be created in the __src__ directory, not the root of the workspace. Navigate into __ros2_ws/src__ and create a new package:

```python
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces
```
