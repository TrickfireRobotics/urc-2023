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
- Outputs data to the Decision Making and GPS Anchor Nodes as needed.

**_Optional:_** split camera processing and other sensor processing (Lidar, etc.) into separate nodes

### GPS Anchor Node

**Role**: Sets the anchor point for the rover from the initial GPS fix.

**Functionality**:

- Subscribes to the /fix topic (GPS).
- Ignores any fixes for 15s after startup to allow GPS to stabilize.
- Captures the first valid fix after that 15s window.
- Publishes the anchor lat/lon/alt on a transient local topic, ensuring late subscribers still receive the message.
- Only sets the anchor once; subsequent messages or time do not affect it.

### Decision Making Node

**Role**: Acts as the high-level controller.

**Functionality**:

- Receives inputs from Navigation and Sensor Processing.
- Makes decisions on stopping, rerouting, or changing navigation based on sensor or localization feedback.

**_Optional:_** May need to break decision types into submodules if it becomes complex. Examples:

- Collision Handling
- Target Reacquisition
- Rerouting?
