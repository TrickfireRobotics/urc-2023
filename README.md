# Trickfire Robotics University Rover Challenge Software
Software developed by [Trickfire Robotics'](https://www.linkedin.com/company/trickfire-robotics/) for the Mars' Society [University Rover Challenge](https://urc.marssociety.org/), written for [ROS2](https://design.ros2.org/articles/why_ros2.html).

## Getting Started
We develop using [Docker](https://en.wikipedia.org/wiki/Docker_(software)) and [Visual Studio Code](https://code.visualstudio.com/). Docker containers keep the development environment well-defined and repeatable: no more mysterious missing dependencies. VS Code is a customizable text editor that has extensions for Docker and all of our programming languages.

A new member with no clue what to do? Get started [here!](../urc-2023/docs/getting_started.md)

Already know how ROS, Git, and Docker work? Then setup your workspace! [Windows](docs/install_on_windows.md) | [Linux](docs/install_on_linux.md) 

## Quick Reference (VS Code)
Open/close the terminal in VS Code with `` Ctrl+` `` (backtick `` ` `` is on the same key as tilde `~`).

| Action             | Terminal command | VS Code shortcut                          |
| ------------------ | ---------------- | ----------------------------------------- |
| Build all packages | `./build.sh`     | `Ctrl+Shift+B`                            |
| Test all packages  | `./test.sh`      | `Ctrl+P`, type `task test`, press `Enter` |
| Launch nodes       | `./launch.sh`    |                                           |
| Build Docker Image | `./build_only_docker.sh` | 
| Launch Docker Image| `./launch_only_docker.sh`| 

Where available, use the VS Code shortcuts because they come with in-editor features such as problem matchers.