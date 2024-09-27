# custom_twist_mux

## Description
`custom_twist_mux` is a ROS 2 package that implements a node to read multiple `cmd_vel` topics and publish the highest priority command to an output topic. The node subscribes to various `geometry_msgs/Twist` messages from different sources and determines which command to publish based on predefined priorities.

## Features
- **Multiple source subscriptions**: The node can subscribe to multiple `cmd_vel` topics.
- **Priority-based command selection**: Commands are published based on their assigned priorities, allowing for flexible control over the robot's movement.
- **Configurable parameters**: Sources and their priorities can be configured via launch files or parameter files.

## Node Overview
### `twist_mux.py`
The main functionality of the `custom_twist_mux` node is implemented in the `twist_mux.py` file. The node:
1. Subscribes to specified `cmd_vel` sources.
2. Keeps track of the latest received messages and their timestamps.
3. Publishes the highest priority `Twist` message to the output topic based on the defined logic.

#### Parameters:
- **sources**: A list of input topics (e.g., `cmd_vel_source1`, `cmd_vel_source2`).
- **priorities**: A list of priorities corresponding to each source (lower numbers indicate higher priority).
- **cmd_vel_out**: The output topic for the selected command velocity message.

### Example Structure of the Node:
- **Input Topics**: Subscribes to topics such as `cmd_vel` and `cmd_vel_joy`.
- **Output Topic**: Publishes to the topic defined by the `cmd_vel_out` parameter.

## Launch File Overview
The `twist_mux.launch.py` launch file is used to start the `custom_twist_mux` node with specific parameters.

### Launch File Structure:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='custom_twist_mux',
            executable='twist_mux',
            name='custom_twist_mux',
            parameters=[{
                'sources': ['cmd_vel', 'cmd_vel_joy'],
                'priorities': [2, 1]
            }]
        )
    ])
```

### Parameters:
- **sources**: Specifies which `cmd_vel` topics the node should listen to.
- **priorities**: Sets the priorities for each source. Lower numbers indicate higher priority.

## Installation

1. **Clone the repository**:
   ```bash
   git clone <repository_link>
   cd custom_twist_mux
   ```

2. **Build the workspace**:
   Make sure to build the package in your ROS 2 workspace:
   ```bash
   colcon build
   ```

3. **Source the workspace**:
   After building, source the setup file:
   ```bash
   source install/setup.bash
   ```

## Usage

To launch the `custom_twist_mux` node, use the following command:
```bash
ros2 launch custom_twist_mux twist_mux.launch.py
```

### Command Priority Example:
- If `cmd_vel` publishes a `Twist` message with priority `2` and `cmd_vel_joy` publishes with priority `1`, the node will select the `Twist` message from `cmd_vel_joy` to publish to the output topic because it has the higher priority (lower number).

## Dependencies
- [ROS 2](https://docs.ros.org/en/rolling/Installation.html)
- `geometry_msgs` for `Twist` message types

## License
This project is licensed under the [MIT License](LICENSE).

## Contributors
- **Axel NIATO** - Lead Developer
