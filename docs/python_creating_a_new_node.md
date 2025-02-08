# Creating a New Node
This will help you create a new node package, set up any dependencies, and get your node to be launched with the rest of the codebase. 

## `Creating a Package`
1. Make sure you are inside your Docker container. See [getting_started.md](./getting_started.md) if you do not know how

2. Change your directory to `/src`
First determine what directory you are currently in using `pwd` (print working directory). 

    Type `ls` (list) to determine what files and directories are in your current working directory
Type `cd <name of folder>` (change directory) in order to move into a directory, aka a folder

    Do this until you reach `/urc-2023/src`

3. Create a new Python node package
Run the following command `ros2 pkg create --build-type ament_python <package_name>`

    I ran `ros2 pkg create --build-type ament_python example_node`

## `Setting Up the Node`
4. Make a new Python file under your node's src location. Call it whatever you want, I called it `example.py`

    For me it is under `src/example_node/example_node`. Yes, it is a bit funky why
    the two directories have the same name. ¯\\\_(ツ)\_/¯

5. Remove the `test_flake8.py` and `test_pep257.py` files under your node's `/test`. We have another linter, so these would interfere

    For me it was in `src/example_node/test`

6. Open up the `setup.py` file. Leave it open for now, we will get to it later

## `Writing a Basic Node to Say "Hello World"`
7. You will need to import the ROS Python library, create a class representing your node, create an entry point, and write code to "gracefully-ish" shutdown the node when we press `ctrl-c`. Make sure to read the [formatting.md](./formatting.md) document to make sure you are writing good readable code. 

    An important detail to remember is the name of your node:
   
    `super().__init__("this_is_the_name_of_the_node")`

```
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lib.color_codes import ColorCodes, colorStr


class ExampleNode(Node):
    
    def __init__(self) -> None:
        super().__init__("my_example_node")
        self.get_logger().info(colorStr("Launching example_node node", ColorCodes.BLUE_OK))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        example = ExampleNode()
        rclpy.spin(example)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        example.get_logger().info(colorStr("Shutting down example_node node", ColorCodes.BLUE_OK))
        example.destroy_node()
        sys.exit(0)
```

## `Adding to the List of Launch-able Nodes`
8. Go back to your `setup.py` file. Look at the `console_scripts`, it should look this right now

    ` 'console_scripts': [],`

    Add an entry that points towards your `main()` function found in your Python file that you added. The syntax is the following

    `'console_scripts': ["<name of executable> = <name of package>.<name of the python file that has the main() function>:main"],`

    For me it would be this: 

    `'console_scripts': ["myExampleNode = example_node.example:main"],`

9. Open the file `robot.launch.py` under the folder `/src/viator_launch`. Create a new variable to store your node. The `package` parameter should be the same name as your node package name. The `executable` parameter should be the variable name you chose in the `setup.py` file. The `name` is the name of your node you wrote in the code in the `super().__init__()` method step 7. 
 
    For me, I wrote the following.

    `example_node = Node(package="example_node", executable="myExampleNode", name="my_example_node")`

11. In the same `robot.launch.py` file, add it to the array in the `generate_launch_description()` function. 

    ```
    def generate_launch_description() -> launch.LaunchDescription:  # pylint: disable=invalid-name
    return launch.LaunchDescription(
        [can_moteus_node, drivebase_node, mission_control_updater_node, arm_node, example_node] # <- I added it to the end of the array
    )
    ```

12. In the `/src/viator_launch` folder, open up the `package.xml` file. Add a new `<exec_depend>` entry to tell that your package should be included during build. The content is the name of your node package. 

    For me it is this.

    `<exec_depend>example_node</exec_depend>`

## `Build and Execute Code`
12. All while in the Docker container, execute the shell scripts `./build.sh` and `./launch.sh` in the command line. You might have to delete the `/build` and `/install` folders. 

13. Congrats, you have made a new node ! Yippie! Further examples of how to use ROS timers, subscribers, and publishers are found in the `/src/example_node/example_node` node; go ahead open it up and read the code. 




