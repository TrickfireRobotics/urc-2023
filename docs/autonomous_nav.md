# Autonomous Navigation Package

The Autonomous Navigation Package (`src/autonomous_nav/`) consists of all high-level nodes required for the autonomous navigation mission. The contents of each node, described below, reflect the functionality required to complete the mission.

## Node Architecture

### Navigation Node

**Role:** Manages waypoint navigation, path planning.

**Functionality**: Integrates with map data (either local or global) and collaborates with the Decision Making Node for route adjustments.

**_Optional:_** Separate mapping into a Mapping Node

### Sensor Processing Node

**Role**: Process all sensor data, primarily camera images, for object and obstacle detection.

- Subscribe to and process camera images with OpenCV

**Functionality:**

- Subscribe to images and process for obstacle & object recognition with OpenCV.
- Outputs data to the Decision Making and Localization Nodes as needed.

**_Optional:_** split camera processing and other sensor processing (Lidar, etc.) into separate nodes

### Control Node

**Role**: Controls rover actuation (movement commands).

**Functionality**: Constructs & executes actuation commands using information from the Navigation and Decision Making Nodes, handling speed, orientation, and stopping.

- Only accept processed command signals to minimize node complexity

### Localization Node

**Role**: Tracks the rover’s position using GNSS, IMU, or other localization sensors.

**Functionality**: Integrates data from the sensor suite, excluding object detection camera data.

- May need to incorporate sensor fusion between GNSS & IMUs.

### Decision Making Node

**Role**: Acts as the high-level controller.

**Functionality**:

- Receives inputs from Navigation, Sensor Processing, and Localization.
- Makes decisions on stopping, rerouting, or changing navigation based on sensor or localization feedback.

**_Optional:_** May need to break decision types into submodules if it becomes complex. Examples:

- Collision Handling
- Target Reacquisition
- Rerouting?
