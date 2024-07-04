# The [`can_moteus`](https://github.com/TrickfireRobotics/urc-2023/tree/main/src/can_moteus) Package


## **What is it?**
This is a **ROS Node** that is responsible for communication between the [**Moteus motor controllers**](https://github.com/mjbots/moteus) (via a CAN-FD Bus) and **ROS** that the rest of the codebase is written in. 

Motors are created that represent their real-life counterparts that are connected to a Moteus controller. Each motor has a subscriber that gets data from the rest of the codebase (position, velocity, etc); and a publisher that sends data from the motor to the rest of the codebase (torque, power usage, voltage, etc). The data contained in these topics is a String in the format of a JSON.

The only structure inthe whole codebase that should be directing sending and reading the JSONs to the `can_moteus` would be the `RobotInterface` and the `RobotInfo`. See [`code_overview.md`](./code_overview.md) for a diagram. 


## **How to use it**
In order to add a new motor, make sure that the motor has a Moteus controller connected to it and is on the same CAN-FD bus. Modify `ros_moteus_bridge.py` in order to add a motor under the `createMoteusMotors()` method. See the following example.

```
self.threadManager.addMotor(<CANID>, "NAME OF MOTOR")
```

That's it. Simple as. Simple is.

## **JSON Data**
There are two different JSON formatted data that is used; one for input and one for output.

### Input [`moteus_data_in_json_helper.py`](../src/utility/moteus_data_in_json_helper.py)
The data contained here is used to set the data that the motor should be sending to the Moteus controller. By default, everything is set to `None` with the exception of `setStop` which is set to `True`. The parameter `setStop` disables the controller, thus allowing the motor to freely spin.  

### Output [`moteus_data_out_json_helper.py`](../src/utility/moteus_data_out_json_helper.py)
The data contained here is used to set the results from the Moteus motors. By default, all variables are set to `None`. This should be used by the `RobotInfo` to process the raw data.

Note: All data **must** be in the units that the Moteus controller expects. Reference the Moteus documentation on their Github to find these units. 


## **Deep Dive - How is `can_moteus` Implemented?**
The Moteus library uses Python's [asynchronous I/O](https://docs.python.org/3/library/asyncio.html) features, but ROS does not work well with Python's async. As such, we make a new thread ([`moteus_thread`](../src/can_moteus/can_moteus/moteus_thread_manager.py)) to handle the async functions in parallel with the ROS thread. 

The user adds new motors by calling the `addMotor()` method in the `MoteusThreadManager` class. `MoteusThreadManager` then creates a `MoteusMotor` object that is mapped to the name given to the motor by `addMotor()`. 

When the user calls `self.threadManager.start()`, the `MoteusThreadManager` creates a new `Thread` object with the name `"moteus_thread"`. This thread points to the method `threadEntry` which creates and executes the asynchronous loop. 

The main async loop initially calls the method `connectToMoteusControllers()` in order
to connect to the Moteus controllers. Inside of this method we create a `moteus.Controller` object which comes from the Moteus library allowing us to interact with the motor controller. We call the controller's `query()` method with a timeout - if it takes longer for the method to return than the timeout, we assume that we do not have connection to the controller. If it does return, we map this `moteus.controller` object to the motor name given in `addMotor()`. 

In the main async loop, we go through each key value pair of the `_nameToMoteusController` dictionary. We try to `query()` the motor, and if a timeout occurs we remove this motor from the dictionary and notify the user in the cmd. If it passes this, we check for any faults from the result of this query. When a fault does exist, we print it out and try to `set_stop()` the motor. If stopping the motor fails, we remove the motor from the dictionary. After these checks pass, we send the data found in the `_nameToMoteusMotor` dictionary to the Moteus controller. The results returned are then published to the ROS network via the shared `MoteusMotor` object. 

The way we exit the async loop is having the ROS thread set the `_shouldMoteusThreadLoop` flag to false. We then go through the `_nameToMoteusController` one more time to call `set_stop()` on each of the motors. We can attempt to reconnect to the moteus motors by setting the `_shouldReconnect` false as well. 

Here is a diagram showing off the structure of this package.

Note 7/2/2024: The mutex is not implemented correctly - as in it protects nothing and essentially does nothing. 


![Can Moteus Overview](can_moteus_overview.png)

