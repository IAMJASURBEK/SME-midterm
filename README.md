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

1.2 Update __setup.py__

Add the same information to the __setup.py__ file for the __maintainer__, __maintainer_email__, __description__ and __license__ fields:


```python
maintainer='Your Name',
maintainer_email='you@email.com',
description='Python client server tutorial',
license='Apache License 2.0',
```
![simple1](https://user-images.githubusercontent.com/90182787/196325923-27c70616-1518-4406-be2c-e672cddc9019.jpg)



### 2. Writing the service node

Inside the __ros2_ws/src/py_srvcli/py_srvcli__ directory, create a new file called __service_member_function.py__ and paste the following code within:

```python
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
2.2 Adding an entry point

open __setup.py__ , add the following line between the __'console_scripts'__: brackets
```python
'service = py_srvcli.service_member_function:main',
```

### 3. Writing the client node

Inside the __ros2_ws/src/py_srvcli/py_srvcli__ directory, create a new file called __client_member_function.py__ and paste the following code within

```python
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

3.2 Adding an entry point
Open __setup.py__ , chnage it so should look like this
```python
entry_points={
    'console_scripts': [
        'service = py_srvcli.service_member_function:main',
        'client = py_srvcli.client_member_function:main',
    ],
},
```


![3dan keyin](https://user-images.githubusercontent.com/90182787/196326008-4bee51f3-fda8-482d-9d1b-db9924f1d6b2.jpg)



### 4. Building and Run
Check for missing dependencies

```python
rosdep install -i --from-path src --rosdistro humble -y
```

Navigate back to the root of your workspace, __ros2_ws__, and build your new package

```python
colcon build --packages-select py_srvcli
```

Open a new terminal, navigate to __ros2_ws__, and source the setup files

```python
. install/setup.bash
```

Now run the service node
```python
ros2 run py_srvcli service
```

If you chose __2__ and __3__, for example, the client would receive a response like this


![dff](https://user-images.githubusercontent.com/90182787/196326160-e4eaabb8-2ca6-4226-9ed3-4b0a87edea85.jpg)

Return to the terminal where your service node is running. You will see that it published log messages when it received the request


![oxigisi](https://user-images.githubusercontent.com/90182787/196326258-39eefa99-eb60-4da9-95b6-390ff094b19a.jpg)

Enter __Ctrl+C__ in the server terminal to stop the node from spinning
