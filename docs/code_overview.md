# **Code Overview**

Most of the codebase is written in Pyton due as most of the other components are coupled (depend on) the `robot_interface` class to control the robot. However, Python is easier to develop in and has more third-party libraries compared to C++. 

## **Diagram**
![what](./resources/program_structure2.png)

## **Components of the CodeBase**
### **CAN Related**

### **`moteus_can`**
[Docs link](./moteus_can.md)

This is responsible for sending/recieving data from the [moteus controllers](https://github.com/mjbots/moteus) that are connected via a CANFD to USB adapter. This will also send/recieve moetus-formatted data from `robot_interface` python class. 

This package will publish certain piece of data to ROS from the moteus controllers. It will also subscribe to data that is sent from the `robot_interface` class. 


### **`robot_interface`**
This is responsible for converting human formatted units into units that the moteus controllers can understand through a long list of methods that correspond to functionality on the actual rover. 

For example, each wheel on the drivebase will have a method that will take in some sort of human-readable value, convert it to a moteus-readable value, and then publish it to the correct topic. The `moteus_can` will then actually send it to the hardware. 

This is not a ROS node, but instead will be created as an object via a ROS node. 


### **Robot Functionality**

### **`antenna`**
This is responsible for pointing the robot antenna to always point to the base station's antenna to secure the srongest WiFi signal.

### **`drivebase`**
This is responsible for implementing *how* a drivebase should work depending on user input. 

Through a subscriber, it will recieve signals to when execute a high level control scheme: "move left side", "move right side", "move forwards", etc.

### **`arm`**
This is responsible for controlling the arm via high level control schemes using inverse-kinematics. 

### **`life_detection`**
This detects life somehow. The physical design is still in progress

### **Base Station**
### **`TODO: add docs for the base station package`**
This will recieve controller data from the browser and publish it to the ROS network.

### **Miscellaneous**

### **`viator_launch`**
This launches all of the ROS nodes in our code. If you want to add a node, make sure to update the [`robot.launch.py`](../src/viator_launch/launch/robot.launch.py) and the [`package.xml`](../src/viator_launch/setup.py) files. 

### **`can`**
This is resonsible for using the CANOpen protocol using the [ros2_canopen](https://github.com/ros-industrial/ros2_canopen) library to communicate between a few parts of the rover which cannot use the moteus controllers. 


### **`monitering`**
This is planned to be utilized as a logging system. This might or might not be removed



