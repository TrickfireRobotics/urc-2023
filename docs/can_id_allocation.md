
## **CAN IDs**

Each Moteus controller is identified with a CAN ID.

Moteus controllers with lower CAN IDs have higher priority and will get messages before motors with higher CAN ID numbers.


The CAN IDs we use are as follows:

| Arm | Drivebase | Antenna | Life Detection |
|:---:|:---------:|:-------:|:--------------:|
| 1-9 |   20-29   |  40-49  |     60-69      |


The Arm IDs start at 1 because Moteus controllers don't support CAN ID 0.

Each block is 10 IDs wide because we don't expect to use more than 9 motors for any part of the rover.

We left 10 block gaps between ID sections in case we need to add another subsystem to the rover.