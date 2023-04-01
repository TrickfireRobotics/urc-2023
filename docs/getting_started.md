# **Getting Started**

Welcome to the software subteam! We hope that you will stick around and learn quite a bit of things about robots! However, before you can do that, you should review the resources given in this document to familarize yourself. 


## **Libraries/Tools/Technologies that We Utilize**
In this particular codebase, we use the following libraries/tools/technologies
- Docker (To standardize software development and deployment)
- ROS2 (Robot Operating System)
- Python
- C++ 
- HTML/Javascript/CSS/Front end frameworks
- Git/Github

## **What is Docker?**
Docker allows us to create "virtual machines" called **containers** without actually running a proper virtual machine. To create a **container**, we use a **docker image**. To create

This [Stackoverflow](https://stackoverflow.com/questions/28089344/docker-what-is-it-and-what-is-the-purpose) post explains it quite well.

## **What is ROS?**
The **Robot Operating System (ROS)** is not an operating system, but instead a collection of libaries and tools that allow software developers to write  code more focussed on robots. We use [**ROS2 Humble**](https://docs.ros.org/en/humble/index.html)

The core component of ROS are **Nodes** and its **Observer Pattern** in the form of **Subscribers/Publishers**. Each **node** is essentially its own little program doing its own thing. If you have two, or perhaps more,  **nodes** that you want to communicate, how do you do that? Here comes the **publisher** - it allows a **node** to publish ("yell out") data on a certain topic (like a frequency for a radio). To recieve this data, a node can subscribe ("listen to") to a topic published by another node. 

Here is a good video that talks more about ROS and its other communicate protocols

[![IMAGE ALT TEXT](https://img.youtube.com/vi/7TVWlADXwRw/0.jpg)](https://www.youtube.com/watch?v=7TVWlADXwRw "Video Title")


## **Documentation**
All of the documentation for this codebase is stored inside this `docs` folder. Here, you will find a markdown file for each ROS Node (packge/folder) under the `src` folder that explains its workings in more detail. 

## **What Should I do Now?**
After reading this document, you should follow the instructors for [install_on_windows.md](./install_on_windows.md) if you will be developing code on a Windows machine or [install_on_linux.md](./install_on_linux.md) if you will be developing on Linux.