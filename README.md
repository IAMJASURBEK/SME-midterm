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



# Creating custom msg and srv files

### 1. Creating a new package

Since we will use the pub/sub and service/client packages created in earlier tutorials, make sure you are in the same workspace as those packages (ros2_ws/src), and then run the following command to create a new package

```python
ros2 pkg create --build-type ament_cmake tutorial_interfaces
```

It is good practice to keep __.msg__ and __.srv__ files in their own directories within a package. Create the directories in __ros2_ws/src/tutorial_interfaces__

```python
mkdir msg

mkdir srv
```

### 2. Creating customer definitions

#### 2.1 Definiton of msg
In the __tutorial_interfaces/msg__ directory you just created, make a new file called __Num.msg__ with one line of code declaring its data structure

```python
int64 num
```

Also in the __tutorial_interfaces/msg__ directory you just created, make a new file called __Sphere.msg__ with the following content

```python
geometry_msgs/Point center
float64 radius
```

### 2.2 Definition of srv

Back in the __tutorial_interfaces/srv__ directory you just created, make a new file called __AddThreeInts.srv__ with the following request and response structure

```python
int64 a
int64 b
int64 c
---
int64 sum
```

### 3. CMakelists.txt

To convert the interfaces you defined into language-specific code (like C++ and Python) so that they can be used in those languages, add the following lines to __CMakeLists.txt__

```python
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
```

### 4. package.xml

Add the following lines to __package.xml__

```python
<depend>geometry_msgs</depend>

<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```


### 5. Building the __tutorial_interfaces__ package

Now that all the parts of your custom interfaces package are in place, you can build the package. In the root of your workspace (__~/ros2_ws__), run the following command

```python
colcon build --packages-select tutorial_interfaces
```

### 6. Confirming msg and srv creation

In a new terminal, run the following command from within your workspace (__ros2_ws__) to source it

```python
. install/setup.bash
```
Now you can confirm that your interface creation worked by using the __ros2 interface show__ command

```python
ros2 interface show tutorial_interfaces/msg/Num
```
And

```python
ros2 interface show tutorial_interfaces/msg/Sphere
```

And

```python
ros2 interface show tutorial_interfaces/srv/AddThreeInts
```

![fes1](https://user-images.githubusercontent.com/90182787/196334235-e5ea756b-496f-4adf-8dc0-8cc0b5787a9c.jpg)

### 7. Test the new interfaces


### 7.1 Testing Num.msg with pub/sub

Open the publisher file and change it accordingly

```python
import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Num                            # CHANGE


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Num, 'topic', 10)  # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Num()                                                # CHANGE
        msg.num = self.i                                           # CHANGE
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.num)       # CHANGE
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```


Open the subscriber file and change it accordingly
```python
import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Num                        # CHANGE


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Num,                                               # CHANGE
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
            self.get_logger().info('I heard: "%d"' % msg.num)  # CHANGE


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

in the __package.xml__ add the following line

```python
<exec_depend>tutorial_interfaces</exec_depend>
```
After making the above edits and saving all the changes, build the package

```python
colcon build --packages-select py_pubsub
```

Then open two new terminals, source __ros2_ws__ in each, and run

```python
ros2 run py_pubsub talker
```

```python
ros2 run py_pubsub listener
```

![sdw1](https://user-images.githubusercontent.com/90182787/196334147-f57f8c09-c07b-4466-8884-dbc79a9d349c.jpg)

![sdw2](https://user-images.githubusercontent.com/90182787/196334165-bf5dd043-1018-44b7-ac27-1b6f568829f4.jpg)

### 7.2 Testing AddThreeInts.srv with service/client

Open the Service file and change it as following

```python
from tutorial_interfaces.srv import AddThreeInts                                                           # CHANGE

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback)       # CHANGE

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b + request.c                                                   # CHANGE
        self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c))  # CHANGE

        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Client file

```python
from tutorial_interfaces.srv import AddThreeInts                            # CHANGE
import sys
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddThreeInts.Request()                                   # CHANGE

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.req.c = int(sys.argv[3])                                       # CHANGE
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_three_ints: for %d + %d + %d = %d' %                                # CHANGE
                    (minimal_client.req.a, minimal_client.req.b, minimal_client.req.c, response.sum))  # CHANGE
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Add the following line to package.xml

```python
<exec_depend>tutorial_interfaces</exec_depend>
```

After making the above edits and saving all the changes, build the package

```python
colcon build --packages-select py_srvcli
```

Then open two new terminals, source __ros2_ws__ in each, and run

```python
ros2 run py_srvcli service
```

```python
ros2 run py_srvcli client 2 3 1
```

![photo_2022-10-18_13-30-00](https://user-images.githubusercontent.com/90182787/196335805-239a7755-199e-4e46-8062-1a4529ba9873.jpg)

![photo_2022-10-18_13-30-05](https://user-images.githubusercontent.com/90182787/196335841-12191277-c54e-43f5-b79a-be8bf8ee0c53.jpg)
