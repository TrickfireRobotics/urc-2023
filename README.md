# Trickfire Robotics University Rover Challenge Software
Software developed by [Trickfire Robotics'](https://www.trickfirerobotics.org/) for the Mars' Society [University Rover Challenge](https://urc.marssociety.org/), written for [ROS2](https://design.ros2.org/articles/why_ros2.html).

We develop using [Docker](https://en.wikipedia.org/wiki/Docker_(software)) and [Visual Studio Code](https://code.visualstudio.com/). Docker containers keep the development environment well-defined and repeatable: no more mysterious missing dependencies. 

## `Getting Started`
**A new member with no clue what to do? Go read the [getting_started.md](./docs/getting_started.md) docs!**

Already know how ROS, Git, and Docker work? Then setup your workspace! [Windows](docs/install_on_windows.md) | [Linux](docs/install_on_linux.md) | [Mac](docs/installing_on_mac.md)

Read how the codebase is organized by reading the [code_overview.md](./docs/code_overview.md) docs.

Make sure you understand how to format your Python code. Go read the [formatting.md](./docs/formatting.md) docs.



## `Quick Reference (VS Code)`
Open/close the terminal in VS Code with `` Ctrl + ` `` (backtick `` ` `` is on the same key as tilde `~`).

| Action             | Terminal command | VS Code shortcut                          |
| ------------------ | ---------------- | ----------------------------------------- |
| Build all packages | `./build.sh`     | `Ctrl+Shift+B`                            |
| Launch nodes       | `./launch.sh`    |                                           |
| Build Docker Image | `./build_only_docker.sh` | 
| Launch Docker Image| `./launch_only_docker.sh`| 
| Connect to Docker Container | `./connect_to_container.sh`

Where available, use the VS Code shortcuts because they come with in-editor features such as problem matchers.