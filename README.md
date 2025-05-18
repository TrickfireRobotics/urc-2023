# Trickfire Robotics University Rover Challenge Software
Software developed by [Trickfire Robotics](https://www.trickfirerobotics.org/) for the Mars Society's [University Rover Challenge](https://urc.marssociety.org/), written for [ROS2](https://design.ros2.org/articles/why_ros2.html).

We develop using [Docker](https://en.wikipedia.org/wiki/Docker_(software)) and [Visual Studio Code](https://code.visualstudio.com/). Docker containers keep the development environment well-defined and repeatable: no more mysterious missing dependencies. 

## `Getting Started`
Refer to our [Notion wiki](https://www.notion.so/trickfire/Getting-Started-With-Viator-Rover-1491fd41ff5b801485e0f6ad57e0a0aa) for more information.


## `Quick Reference (VS Code)`
Open/close the terminal in VS Code with `` Ctrl + ` `` (backtick `` ` `` is on the same key as tilde `~`).

| Action             | Terminal command | VS Code shortcut                          |
| ------------------ | ---------------- | ----------------------------------------- |
| Build all packages | `./build.sh`     | `Ctrl+Shift+B`                            |
| Launch nodes       | `./launch.sh`    |                                           |
| Connect to Docker Container | `./connect_to_container.sh` |
| Restart to Docker Container | `./container_launch.sh` |

Where available, use the VS Code shortcuts because they come with in-editor features such as problem matchers.