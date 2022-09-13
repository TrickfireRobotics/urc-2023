# Trickfire Robotics University Rover Challenge Software
Software developed by [Trickfire Robotics'](https://www.linkedin.com/company/trickfire-robotics/) for the Mars' Society [University Rover Challenge](https://www.nasa.gov/offices/education/centers/kennedy/technology/nasarmc.html), written for [ROS2](https://design.ros2.org/articles/why_ros2.html).

## Getting Started
We develop using [Docker](https://en.wikipedia.org/wiki/Docker_(software)) and [Visual Studio Code](https://code.visualstudio.com/). Docker containers keep the development environment well-defined and repeatable: no more mysterious missing dependencies. VS Code is a customizable text editor that has extensions for Docker and all of our programming languages.

Get started on [Windows](docs/install_on_windows.md) | [Linux](docs/install_on_linux.md) | [Mac](docs/install_on_mac.md)

## Quick Reference (VS Code)
Open/close the terminal in VS Code with `` Ctrl+` `` (backtick `` ` `` is on the same key as tilde `~`).

| Action             | Terminal command | VS Code shortcut                          |
| ------------------ | ---------------- | ----------------------------------------- |
| Build all packages | `./build.sh`     | `Ctrl+Shift+B`                            |
| Test all packages  | `./test.sh`      | `Ctrl+P`, type `task test`, press `Enter` |
| Launch nodes       | `./launch.sh`    |                                           |

Where available, use the VS Code shortcuts because they come with in-editor features such as problem matchers.