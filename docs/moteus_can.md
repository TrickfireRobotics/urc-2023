# The [`moteus_can`](https://github.com/TrickfireRobotics/urc-2023/tree/main/src/can_moteus) Package


## **What is it?**
This is a **ROS Node** that is responsible for communication between the **Moteus motor controllers** (via a FDCAN/CAN Bus) and the **ROS network** that the rest of the codebase is written in. 

Motors are created that represent their real-life counterparts. Each motor subscribes to a topic that contains  data for the motors to use depending on the motor's mode; i.e. a motor in  VELOCITY mode would subscribe to data that would set that motor's velocity. Each motor can publish data to the ROS netwrok that it reads from the Moteus controllers depending on what data the programmer wanted that motor to publish. 

The only structure in the whole codebase that should be directly sending data for these motors to subscribe to is the `RobotInterface`; see `code_overview.md` for a diagram. 

## **How to use it**

### **Adding a motor in code**
As each Moteus controller is connected to only a single motor, each motor is represented as a `MoteusMotor` object. However, the `MoteusMultiprocess` object, of which only one should exist, handles these `MoteusMotors`. As such, motors are added via the `MoteusMultiprocess` object.

To create a motor, we need to pass in the following parameters in this order:
- The CAN ID of the motor. This is an **integer**
- The name of the motor. This will be used when creating ROS publishers and subscribers. This is a **String**
- The motor mode. This is a **moteus_motor.Mode**
- A list of data to publish to the ROS network. This is an **array of moteus.Register**

Here is an example of adding a motor

```
moteusPubList = [moteus.Register.VELOCITY, moteus.Register.POSITION]
        moteusMultiprocess.addMotor(
            3,
            "topmotor",
            moteus_motor.Mode.VELOCITY,
            moteusPubList,
        )
```
This motor has a CAN ID of **three** with the name **topmotor**. The motor is running in **VELOCITY** mode, and as such expects incoming data intended for the motor to set its velocity in moteus formatted units. Each time the motor reads data from the Moteus controller, it will publish its **VELOCITY** and **POSITION** values to the ROS network in Moteus formatted units. 

### **Inputs**
**NOTE: All subscribers and publishers handle data in std_msgs.msg.Float32**

When a motor is created, it will create a ROS subscriber to the following topic:

```<name of the motor>_<the data being recieved as per the mode>_from_robot_interface``` 

For example, using the motor above, it would subscribe to the topic of: 

```topmotor_velocity_from_robot_interface```

The data sent to this topic will control the particular motor that it is sent to. Keep in mind that this data has to be formatted in Moteus units. 

Every cycle (every 0.02 seconds), the motor will send the last recieved data to send to the Moteus controller. The default is a value of **zero.** 

### **Outputs**
**NOTE: All subscribers and publishers handle data in std_msgs.msg.Float32**

**NOTE: The data in the array must match up to the possible data that the Moteus controller can return; which is precicely the regsters from `moteus.Register`!**

When a motor is created, it will subscribe to the following topic depending on the **moteus.Register array** that was passed in:

```<name of the motor>_<the data being sent via moteus.register>_from_can```

For example, using the motor above, it would subscribe to the topics of:

```/topmotor_velocity_from_can```

```/topmotor_position_from_can```

Every cycle (every 0.02 seconds), the motor would read data it recieved from the Moteus controller and publish it. 

## **How it is implemented**

### **Overview**
The core idea is that there are two distinct Python processes running that communicate with each other via several different multiprocess queues. The first process is the "main" process where ROS lives and does whatever it needs to do. The second process is started by us and is used to send and read data to the Moteus controllers. 

### **Queue from motors to multiprocess**

Each motor's callback function for their subscribers write to the `_queueToMoteus` queue (created in `moteus_multiprocess.py`), where only one such queue exists. The data that is sent is an array of length two in the following format:

```
[0] = CANID
[1] = data
```

### **Queue from multiprocess to each motor**
Each motor creates its own multiprocess queue, `toPublisherQueue`, which is populated by the Moteus multiprocess for each corresponding motor. The data that is sent is dictated by the `moteusPubList` array as seen earilier in the example motor. The data that is sent is an array in the following format:

```
[0] = moteus.Register
[1] = data
```

### **The Multiprocess Cycle**
Each cycle, 0.02 seconds, the moteus multiprocess reads the head of the `_queueToMoteus` and updates the set value for the corresponding motor. It then goes through each motor that was succesfully connected to the CAN/CANFD bus and sends its data. After it sends the data, it recieves the Moteus controller and fills each motor's `toPublisherQueue`. 

Here is a diagram of the process

![what](./resources/moteus_docs.png)


## What is a Moteus Unit?
The documentation for Moteus units are found [here](https://github.com/mjbots/moteus/blob/main/docs/reference.md) under the **`A.2 Register Usage`** heading. Depending ont he type of data being handled, the unit will differ. As such, any reference of a "Moteus unit" references this section. 





