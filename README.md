# SME-week-6

## Writing a simple service and client (Python)
### 1. Creating a Package

Recall that packages should be created in the __src__ directory, not the root of the workspace. Navigate into __ros2_ws/src__ and create a new package:

```python
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces
```

1.1 Update __package.xml__

Make sure to add the description, maintainer email and name, and license information to __package.xml__

```python
<description>Python client server tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

1.1 Update __setup.py__

Add the same information to the __setup.py__ file for the __maintainer__, __maintainer_email__, __description__ and __license__ fields:


```python
maintainer='Your Name',
maintainer_email='you@email.com',
description='Python client server tutorial',
license='Apache License 2.0',
```
